#include <rtapi.h>		/* RTAPI realtime OS API */
#include <rtapi_app.h>	/* RTAPI realtime module decls */
#include <hal.h>		/* HAL public API decls */

typedef struct {
    hal_s32_t *status;
} realtime_t;

realtime_t *inst;

static int comp_id;		/* component ID */

int rtapi_app_main(void)
{
    int ret;
    comp_id = hal_init("realtime");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "REALTIME: ERROR: hal_init() failed\n");
        return -1;
    }

    // allocate shared memory for the base struct
    inst = hal_malloc(sizeof(realtime_t));
    if (inst == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "REALTIME: Out of Memory\n");
        hal_exit(comp_id);
        return -1;
    }

    ret = hal_pin_s32_newf(HAL_OUT, &(inst->status), comp_id, "realtime.status");
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "REALTIME: ERROR: pin_newf() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    *inst->status = rtapi_is_realtime();

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}