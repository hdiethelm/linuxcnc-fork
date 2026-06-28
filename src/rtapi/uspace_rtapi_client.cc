/* Copyright (C) 2006-2014 Jeff Epler <jepler@unpythonic.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <vector>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits>
#include <limits.h>
#include <stdint.h>
#include "config.h"
#define rtapi_print_msg(lvl, args...) fprintf(stderr, args)
/*
 * Fully checked send/recv
 * Will retry on EINTR, so to abort a send_data/recv_data on a
 * signal, change to something like while(remaining > 0 && !exit_flag)
 * and set exit_flag in signal handler
 */
static ssize_t send_data(int fd, const void *buf, size_t n, int flags) {
    if(n > SSIZE_MAX){
        errno=EFBIG;
        return -1;
    }
    const uint8_t *ptr = (const uint8_t *)buf;
    size_t n_rem = n;
    while (n_rem > 0) {
        ssize_t n_ret = send(fd, ptr, n_rem, flags);
        if (n_ret == -1) {
            if (errno == EINTR) {
                // Retry
            } else {
                return -1; // Other error, fail
            }
        } else if (n_ret == 0) {
            return (n - n_rem); // No more data
        } else {
            ptr += n_ret;
            n_rem -= n_ret;
        }
    }
    return n; // All sent
}

static ssize_t recv_data(int fd, void *buf, size_t n, int flags) {
    if(n > SSIZE_MAX){
        errno=EFBIG;
        return -1;
    }
    uint8_t *ptr = (uint8_t *)buf;
    size_t n_rem = n;
    while (n_rem > 0) {
        ssize_t n_ret = recv(fd, ptr, n_rem, flags);
        if (n_ret == -1) {
            if (errno == EINTR) {
                // Retry
            } else {
                return -1; // Other error, fail
            }
        } else if (n_ret == 0) {
            return (n - n_rem); // No more data
        } else {
            ptr += n_ret;
            n_rem -= n_ret;
        }
    }
    return n; // All read
}

/*
 * Protocol:
 * 
 * client->master: std::vector<std::string> args
 * master processes the args and returns result
 * master->client: int result
 *
 * Packing:
 * args are serialized as:
 * uint16_t size (full package size including the size field)
 * uint16_t n_args
 * n_args times:
 * {
 *      uint16_t arg_size
 *      char[arg_size] argument
 * }
 * 
 * result is serialized as:
 * int
 */

static bool recv_result(int fd, int *result) {
    ssize_t res = recv_data(fd, result, sizeof(int), 0);
    if (res != sizeof(int)) {
        if (res == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: recv_result failed: %s\n", strerror(errno));
        } else {
            rtapi_print_msg(
                RTAPI_MSG_ERR, "rtapi_app: recv_result failed, recv only %li of %li bytes\n", res, sizeof(int)
            );
        }
        return false;
    } else {
        return true;
    }
}

static void push_uint16(std::vector<char> &buf, uint16_t value) {
    buf.push_back((char)(0xff & (value >> 0)));
    buf.push_back((char)(0xff & (value >> 8)));
}

static bool send_args(int fd, const std::vector<std::string> &args) {
    //Calculate size
    size_t buff_size = 0;
    buff_size += 2 * sizeof(uint16_t);
    for (size_t i = 0; i < args.size(); i++) {
        buff_size += sizeof(uint16_t);
        buff_size += args[i].size();
    }

    //Check uint16_t conversions
    //Buffer size is > sum(args[i].size()) so they don't need a separate check
    if (buff_size > std::numeric_limits<uint16_t>::max()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: send_args: args to big, size = %li!\n", buff_size);
        return false;
    }
    //Edge case: One could in theory send many size zero args
    if (args.size() > std::numeric_limits<uint16_t>::max()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: send_args: arg count to big, size = %li!\n", args.size());
        return false;
    }

    //Serialize
    std::vector<char> buf;
    buf.reserve(buff_size);
    push_uint16(buf, (uint16_t)buff_size);
    push_uint16(buf, (uint16_t)args.size());
    for (size_t i = 0; i < args.size(); i++) {
        push_uint16(buf, (uint16_t)args[i].size());
        buf.insert(buf.end(), args[i].begin(), args[i].end());
    }
    if (buf.size() != buff_size) {
        rtapi_print_msg(
            RTAPI_MSG_ERR, "rtapi_app: Bug send_args: buf.size() %li != buff_size %li\n", buf.size(), buff_size
        );
        return false;
    }

    //Send
    ssize_t res = send_data(fd, buf.data(), buf.size(), 0);
    if (res != (ssize_t)buf.size()) {
        if (res == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: send_args failed: %s\n", strerror(errno));
        } else {
            rtapi_print_msg(
                RTAPI_MSG_ERR, "rtapi_app: send_args failed, sent only %li of %li bytes\n", res, buf.size()
            );
        }
        return false;
    }
    return true;
}

static int slave(int fd, const std::vector<std::string> &args) {
    if (!send_args(fd, args)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: failed to write to master\n");
        return -1;
    }

    int result = -1;
    if (!recv_result(fd, &result)) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: failed to read from master\n");
        return -1;
    } else {
        return result;
    }
}

static std::string get_fifo_path() {
    std::string s;
    if (getenv("RTAPI_FIFO_PATH")) {
        s = getenv("RTAPI_FIFO_PATH");
    } else if (getenv("HOME")) {
        s = std::string(getenv("HOME")) + "/.rtapi_fifo";
    } else {
        rtapi_print_msg(
            RTAPI_MSG_ERR, "rtapi_app: RTAPI_FIFO_PATH and HOME are unset.  rtapi fifo creation is unsafe.\n"
        );
    }
    return s;
}

static bool get_fifo_path_to_addr(struct sockaddr_un *addr) {
    const std::string s = get_fifo_path();
    if (s.empty()) {
        return false;
    }
    if (s.size() + 2 > sizeof(addr->sun_path)) {
        rtapi_print_msg(
            RTAPI_MSG_ERR,
            "rtapi_app: rtapi fifo path is too long (arch limit %zd): %s\n",
            sizeof(sockaddr_un::sun_path),
            s.c_str()
        );
        return false;
    }
    //See: https://www.man7.org/linux/man-pages/man7/unix.7.html abstract
    //sun_path[0] is a null byte ('\0')
    addr->sun_path[0] = 0;
    strncpy(addr->sun_path + 1, s.c_str(), sizeof(addr->sun_path) - 2);
    return true;
}

static double diff_timespec(const struct timespec *time1, const struct timespec *time0) {
    return (double)(time1->tv_sec - time0->tv_sec) + (double)(time1->tv_nsec - time0->tv_nsec) / 1000000000.0;
}

static int create_socket(){
    int fd = socket(PF_UNIX, SOCK_STREAM, 0);
    if (fd == -1) {
        perror("socket");
        return fd;
    }

    int enable = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    return fd;
}

static int run_slave_cmd(struct sockaddr_un *addr, int fd, const std::vector<std::string> &args){
    int result = -1;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    clock_gettime(CLOCK_MONOTONIC, &now);
    srand48(start.tv_sec ^ start.tv_nsec);
    while (diff_timespec(&now, &start) < 3.0) {
        result = connect(fd, (sockaddr *)addr, sizeof(*addr));
        if (result == 0)
            break;

        usleep((useconds_t)(lrand48() % 100000) + 100); //Random sleep min 100us max 100100us
        clock_gettime(CLOCK_MONOTONIC, &now);
    }
    if (result < 0 && errno == ECONNREFUSED) {
        fprintf(stderr, "Waited 3 seconds for master.  giving up.\n");
        close(fd);
        return 1;
    }
    if (result < 0) {
        fprintf(stderr, "connect %s: %s", addr->sun_path, strerror(errno));
        return 1;
    }
    return slave(fd, args);
}

int main(int argc, char **argv) {
    std::vector<std::string> args;
    for (int i = 1; i < argc; i++) {
        args.push_back(std::string(argv[i]));
    }

    struct sockaddr_un addr;
    memset(&addr, 0x0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (!get_fifo_path_to_addr(&addr))
        exit(1);

    int fd = create_socket();
    if (fd < 0) {
        exit(1);
    }

    // plus one because we use the abstract namespace, it will show up in
    // /proc/net/unix prefixed with an @
    int result = bind(fd, (sockaddr *)&addr, sizeof(addr));

    if (result == 0) {
        //If exit is called and master is not running, just give a warning
        if (args.size() == 1 && args[0] == "exit") {
            rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_app: exit received while not running\n");
            return 0;
        }

        fprintf(stderr, "WARNING: Deprecated: No master found. Use \"realtime start\" to start one.\n"
                "  A master is started automatically.\n"
                "  If this appears while using halcmd: Use halrun instead.\n"
                "  halcmd should only be used with an already running realtime environment.\n"
                "  halrun creates a realtime environment and tears it down at exit.\n");
        //Close the socket, start master and reopen.
        //This socket is allready bound, so master will refuse to start.
        close(fd);
        const char *argv[3];
        argv[0] = EMC2_BIN_DIR "/rtapi_master";
        argv[1] = "start";
        argv[2] = NULL;
        pid_t pid = fork();
        if (pid < 0){
            perror("fork");
            exit(1);
        }
        if(pid == 0){
             result = execvp(argv[0], (char * const *)argv);
             if(result < 0){
                perror("execvp");
             }
        }else{
            int fd = create_socket();
            if (fd < 0) {
                exit(1);
            }
        }
    } 
    
    return run_slave_cmd(&addr, fd, args);
}
