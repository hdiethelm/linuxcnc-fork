/********************************************************************
* Description: soft_limits.c
*   soft_limits for HAL
*
*   ToDo
*
*********************************************************************
*
* Author: Hannes Diethelm
* License: GPL Version 2
* Created on: March 9, 2025
* System: Linux
*
* Copyright (c) 2025 All rights reserved.
*
********************************************************************/

#include <rtapi.h>          /* RTAPI realtime OS API */
#include <rtapi_app.h>      /* RTAPI realtime module decls */
#include <hal.h>            /* HAL public API decls */

/* module information */
MODULE_AUTHOR("Hannes Diethelm");
MODULE_DESCRIPTION("Multiple joints soft_limits for EMC HAL");
MODULE_LICENSE("GPL");
int num_joints=-1;
RTAPI_MP_INT(num_joints, "Number of joints");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* Data needed for each input */
typedef struct {
    //HAL
    hal_float_t acc_max;
    hal_float_t min_pos_limit;
    hal_float_t max_pos_limit;
    
    hal_float_t *pos_cmd_in;
    hal_float_t *vel_cmd_in;
    hal_float_t *pos_fb_in;
    hal_bit_t *pos_lim_sw_in;
    hal_bit_t *neg_lim_sw_in;

    hal_float_t *pos_cmd_out;
    hal_float_t *vel_cmd_out;
    hal_float_t *pos_fb_out;
    hal_bit_t *pos_lim_sw_out;
    hal_bit_t *neg_lim_sw_out;

    hal_bit_t *fault_out;

    //Other
    int state;
} soft_limits_joint_t;

#define MAX_JOINTS    8

/* Base data for a weighted summer. */
typedef struct {
  hal_bit_t *estop_out;
  int state;
} soft_limits_data_t;

/* other globals */
static int comp_id;        /* component ID */
soft_limits_joint_t *joints;
soft_limits_data_t *data;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void process(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

static void cleanup(void){
    hal_exit(comp_id);
}

int rtapi_app_main(void)
{
    int n, retval;


    /* check that there's at least one valid input requested */
    if (num_joints<1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: must specify at least one joints\n");
        return -1;
    }

    /* but not too many */
    if (num_joints > MAX_JOINTS) {
        rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: too many joints requested (%d > %d)\n", num_joints, MAX_JOINTS);
        return -1;
    }

    /* have good config info, connect to the HAL */
    comp_id = hal_init("soft_limits");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: hal_init() failed (Return code %d)\n", comp_id);
        return -1;
    }

    /* allocate shared memory for soft_limits global and pin info */
    data = hal_malloc(sizeof(soft_limits_data_t));
    if (data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: hal_malloc() for common data failed\n");
        cleanup();
        return -1;
    }
    data->state=0;

    joints = hal_malloc(num_joints * sizeof(soft_limits_joint_t));
    if (joints == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: hal_malloc() for input pins failed\n");
        cleanup();
        return -1;
    }

    /* export pins/params for all joints */
    for (n = 0; n < num_joints; n++) {
        retval=hal_param_float_newf(HAL_RW, &(joints[n].acc_max), comp_id, "soft_limits.%d.acc-max", n);
        if (retval != 0) {return -1;}
        retval=hal_param_float_newf(HAL_RW, &(joints[n].min_pos_limit), comp_id, "soft_limits.%d.min-pos-limit", n);
        if (retval != 0) {return -1;}
        retval=hal_param_float_newf(HAL_RW, &(joints[n].max_pos_limit), comp_id, "soft_limits.%d.max-pos-limit", n);
        if (retval != 0) {return -1;}

        retval=hal_pin_float_newf(HAL_IN, &(joints[n].pos_cmd_in), comp_id, "soft_limits.%d.pos-cmd-in", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joints[n].vel_cmd_in), comp_id, "soft_limits.%d.vel-cmd-in", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joints[n].pos_fb_in), comp_id, "soft_limits.%d.pos-fb-in", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_bit_newf(HAL_IN, &(joints[n].pos_lim_sw_in), comp_id, "soft_limits.%d.pos-lim-sw-in", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_bit_newf(HAL_IN, &(joints[n].neg_lim_sw_in), comp_id, "soft_limits.%d.neg-lim-sw-in", n);
        if (retval != 0) {return -1;}

        retval=hal_pin_float_newf(HAL_OUT, &(joints[n].pos_cmd_out), comp_id, "soft_limits.%d.pos-cmd-out", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joints[n].vel_cmd_out), comp_id, "soft_limits.%d.vel-cmd-out", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joints[n].pos_fb_out), comp_id, "soft_limits.%d.pos-fb-out", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_bit_newf(HAL_OUT, &(joints[n].pos_lim_sw_out), comp_id, "soft_limits.%d.pos-lim-sw-out", n);
        if (retval != 0) {return -1;}
        retval=hal_pin_bit_newf(HAL_OUT, &(joints[n].neg_lim_sw_out), comp_id, "soft_limits.%d.neg-lim-sw-out", n);
        if (retval != 0) {return -1;}

        retval=hal_pin_bit_newf(HAL_OUT, &(joints[n].fault_out), comp_id, "soft_limits.%d.fault-out", n);
        if (retval != 0) {return -1;}

        //ToDo: Init!
        joints[n].state=0;
    }

    /* export "global" pins */
    retval=hal_pin_bit_newf(HAL_OUT, &(data->estop_out), comp_id, "soft_limits.estop-out");
    if (retval != 0) {return -1;}
    
    /* export functions */
    retval = hal_export_funct("soft_limits.process", process, joints, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: process funct export failed\n");
        cleanup();
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "soft_limits: installed soft_limits with %d joints\n", num_joints);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    cleanup();
}

/***********************************************************************
*                     REALTIME FUNCTIONS                               *
************************************************************************/

static void process(void *arg, long period)
{
    (void)arg;
    (void)period;
    int n;
    if(data->state == 0){
        for (n = 0; n < num_joints; n++) {
            double v = *joints[n].vel_cmd_in;
            double stop_dist = v * v / ( 2 * joints[n].acc_max );

            *joints[n].pos_cmd_out = *joints[n].pos_cmd_in;
            *joints[n].vel_cmd_out = *joints[n].vel_cmd_in;
            *joints[n].pos_fb_out = *joints[n].pos_fb_in;
            *joints[n].pos_lim_sw_out = *joints[n].pos_lim_sw_in;
            *joints[n].neg_lim_sw_out = *joints[n].neg_lim_sw_in;

            if (v > 0 && *joints[n].pos_cmd_in > joints[n].max_pos_limit - stop_dist + 0.000000000001) {
                if ( joints[n].state == 0 ) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: StopDist = %f pos %f vel %f limit %f joint %d\n", stop_dist, *joints[n].pos_cmd_in, *joints[n].vel_cmd_in, joints[n].max_pos_limit, n);
                }
                joints[n].state=1;
                *joints[n].fault_out=1;
                data->state=1;
            }
            else if (v < 0 && *joints[n].pos_cmd_in < joints[n].min_pos_limit + stop_dist - 0.000000000001) {
                if ( joints[n].state == 0 ) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: StopDist = %f pos %f vel %f limit %f joint %d\n", stop_dist, *joints[n].pos_cmd_in, *joints[n].vel_cmd_in, joints[n].min_pos_limit, n);
                }
                joints[n].state=1;
                *joints[n].fault_out=1;
                data->state=1;
            }
        }
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/
