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
#include <math.h>

/* module information */
MODULE_AUTHOR("Hannes Diethelm");
MODULE_DESCRIPTION("Multiple joints soft_limits for EMC HAL");
MODULE_LICENSE("GPL");
int num_joints=-1;
RTAPI_MP_INT(num_joints, "Number of joints");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef enum{
    STATE_OK,
    STATE_BREAKING,
    STATE_STOPPED,
    STATE_FAULT
}state_t;

typedef enum{
    STATE_JOINT_OK,
    STATE_JOINT_BREAKING,
    STATE_JOINT_FAULT
}joint_state_t;

/* Data needed for each input */
typedef struct {
    //HAL
    hal_float_t acc_max;
    hal_float_t min_pos_limit;
    hal_float_t max_pos_limit;
    
    hal_bit_t *homed;
    hal_float_t *motor_offset;
    hal_float_t *backlash_filt;

    hal_float_t *motor_pos_cmd_in;
    hal_float_t *motor_pos_fb_in;
    hal_float_t *pos_cmd_in;
    hal_float_t *vel_cmd_in;
    hal_float_t *acc_cmd_in;
    hal_float_t *pos_fb_in;
    hal_bit_t *pos_lim_sw_in;
    hal_bit_t *neg_lim_sw_in;

    hal_float_t *motor_pos_cmd_out;
    hal_float_t *motor_pos_fb_out;
    hal_float_t *pos_cmd_out;
    hal_float_t *vel_cmd_out;
    hal_float_t *acc_cmd_out;
    hal_float_t *pos_fb_out;
    hal_bit_t *pos_lim_sw_out;
    hal_bit_t *neg_lim_sw_out;

    hal_bit_t *fault_out;

    //Other
    joint_state_t state;
    hal_float_t *pos_lp;
    hal_float_t *vel_est;
} soft_limits_joint_t;

#define MAX_JOINTS    8

typedef struct {
    soft_limits_joint_t *joints;
    hal_bit_t *estop_out;
    state_t state;
    int comp_id;
} soft_limits_data_t;

static soft_limits_data_t *data;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void process(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

static void cleanup(int comp_id){
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
    int comp_id = hal_init("soft_limits");
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
        cleanup(comp_id);
        return -1;
    }
    data->state=STATE_OK;
    data->comp_id=comp_id;

    data->joints = hal_malloc(num_joints * sizeof(soft_limits_joint_t));
    if (data->joints == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: hal_malloc() for input pins failed\n");
        cleanup(comp_id);
        return -1;
    }

    /* export pins/params for all joints */
    for (n = 0; n < num_joints; n++) {
        soft_limits_joint_t *joint = &data->joints[n];

        retval=hal_param_float_newf(HAL_RW, &(joint->acc_max), comp_id, "soft_limits.%d.acc-max", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_param_float_newf(HAL_RW, &(joint->min_pos_limit), comp_id, "soft_limits.%d.min-pos-limit", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_param_float_newf(HAL_RW, &(joint->max_pos_limit), comp_id, "soft_limits.%d.max-pos-limit", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        retval=hal_pin_bit_newf(HAL_IN, &(joint->homed), comp_id, "soft_limits.%d.homed", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->motor_offset), comp_id, "soft_limits.%d.motor-offset", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->backlash_filt), comp_id, "soft_limits.%d.backlash-filt", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        retval=hal_pin_float_newf(HAL_IN, &(joint->motor_pos_cmd_in), comp_id, "soft_limits.%d.motor-pos-cmd-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->motor_pos_fb_in), comp_id, "soft_limits.%d.motor-pos-fb-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->pos_cmd_in), comp_id, "soft_limits.%d.pos-cmd-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->vel_cmd_in), comp_id, "soft_limits.%d.vel-cmd-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->acc_cmd_in), comp_id, "soft_limits.%d.acc-cmd-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_IN, &(joint->pos_fb_in), comp_id, "soft_limits.%d.pos-fb-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_bit_newf(HAL_IN, &(joint->pos_lim_sw_in), comp_id, "soft_limits.%d.pos-lim-sw-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_bit_newf(HAL_IN, &(joint->neg_lim_sw_in), comp_id, "soft_limits.%d.neg-lim-sw-in", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        retval=hal_pin_float_newf(HAL_OUT, &(joint->motor_pos_cmd_out), comp_id, "soft_limits.%d.motor-pos-cmd-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->motor_pos_fb_out), comp_id, "soft_limits.%d.motor-pos-fb-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->pos_cmd_out), comp_id, "soft_limits.%d.pos-cmd-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->vel_cmd_out), comp_id, "soft_limits.%d.vel-cmd-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->acc_cmd_out), comp_id, "soft_limits.%d.acc-cmd-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->pos_fb_out), comp_id, "soft_limits.%d.pos-fb-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_bit_newf(HAL_OUT, &(joint->pos_lim_sw_out), comp_id, "soft_limits.%d.pos-lim-sw-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_bit_newf(HAL_OUT, &(joint->neg_lim_sw_out), comp_id, "soft_limits.%d.neg-lim-sw-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        retval=hal_pin_bit_newf(HAL_OUT, &(joint->fault_out), comp_id, "soft_limits.%d.fault-out", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        retval=hal_pin_float_newf(HAL_OUT, &(joint->pos_lp), comp_id, "soft_limits.%d.pos-lp", n);
        if (retval != 0) {cleanup(comp_id); return -1;}
        retval=hal_pin_float_newf(HAL_OUT, &(joint->vel_est), comp_id, "soft_limits.%d.vel-est", n);
        if (retval != 0) {cleanup(comp_id); return -1;}

        //ToDo: Init more?
        joint->state=STATE_JOINT_OK;
        *joint->pos_lp=0;
        *joint->vel_est=0;
    }

    /* export "global" pins */
    retval=hal_pin_bit_newf(HAL_OUT, &(data->estop_out), comp_id, "soft_limits.estop-out");
    if (retval != 0) {cleanup(comp_id); return -1;}
    
    /* export functions */
    retval = hal_export_funct("soft_limits.process", process, data, 1, 0, comp_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "soft_limits: ERROR: process funct export failed\n");
        cleanup(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "soft_limits: installed soft_limits with %d joints\n", num_joints);
    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    cleanup(data->comp_id);
}

/***********************************************************************
*                     REALTIME FUNCTIONS                               *
************************************************************************/

static void process(void *arg, long period)
{
    (void)arg;
    int n;
    const double tol1 = 0.000000000001;
    const double tol2 = 1.0; //ToDo: and for inch machines?
    const double dt = period * 1e-9;

    if(dt < 1e-8){
        rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: WARNING: dt = %f us\n", dt*1e6);
        return;
    }

    for (n = 0; n < num_joints; n++) {
        soft_limits_joint_t *joint = &data->joints[n];
        //Ignore unhomed joints
        if(!*joint->homed){
            break;
        }
        if ((data->state == STATE_OK && *joint->pos_cmd_in > joint->max_pos_limit + tol2 ) || 
            *joint->pos_cmd_out > joint->max_pos_limit + tol2 ||
            *joint->pos_fb_in > joint->max_pos_limit + tol2
        ) {
            if ( joint->state != STATE_JOINT_FAULT ) {
                rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: limits are exceided (max_pos_limit)! pos in %f pos out %f pos fb %f limit %f joint %d\n", 
                    *joint->pos_cmd_in, *joint->pos_cmd_out, *joint->pos_fb_in, joint->max_pos_limit, n);
            }
            joint->state=STATE_JOINT_FAULT;
            *joint->fault_out=1;
            *joint->pos_lim_sw_out=1;
            data->state=STATE_FAULT;
        }
        if ((data->state == STATE_OK && *joint->pos_cmd_in < joint->min_pos_limit - tol2 ) || 
            *joint->pos_cmd_out < joint->min_pos_limit - tol2 ||
            *joint->pos_fb_in < joint->min_pos_limit - tol2
        ) {
            if ( joint->state != STATE_JOINT_FAULT ) {
                rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: limits are exceided (min_pos_limit)! pos in %f pos out %f pos fb %f limit %f joint %d\n", 
                    *joint->pos_cmd_in, *joint->pos_cmd_out, *joint->pos_fb_in, joint->min_pos_limit, n);
            }
            joint->state=STATE_JOINT_FAULT;
            *joint->neg_lim_sw_out=1;
            data->state=STATE_FAULT;
        }
    }

    if(data->state == STATE_OK){
        //Pass trough all
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            *joint->motor_pos_cmd_out = *joint->motor_pos_cmd_in;
            *joint->motor_pos_fb_out = *joint->motor_pos_fb_in;
            *joint->pos_cmd_out = *joint->pos_cmd_in;
            *joint->vel_cmd_out = *joint->vel_cmd_in;
            *joint->acc_cmd_out = *joint->acc_cmd_out;
            *joint->pos_fb_out = *joint->pos_fb_in;
            *joint->pos_lim_sw_out = *joint->pos_lim_sw_in;
            *joint->neg_lim_sw_out = *joint->neg_lim_sw_in;
        }

        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            //Ignore unhomed joints
            if(!*joint->homed){
                continue;
            }

            //Estimate velocity based of position to verify linuxcnc is not lying
            double k_lp=0.005/dt; //tau/dt
            double pos_lp_new = (*joint->pos_lp*k_lp + *joint->pos_cmd_in)/(1.0+k_lp);
            *joint->vel_est = (pos_lp_new - *joint->pos_lp)/dt;
            *joint->pos_lp = pos_lp_new;
            //ToDo: And now, what to do with it?

            //Check if motor and joint commands are in sync
            //Code in control.c:
            // joint->motor_pos_cmd = joint->pos_cmd + joint->backlash_filt + joint->motor_offset;
            //-> joint->motor_pos_cmd - joint->pos_cmd - joint->backlash_filt - joint->motor_offset = 0
            double pos_error = *joint->motor_pos_cmd_in - *joint->pos_cmd_in - *joint->backlash_filt - *joint->motor_offset;
            if(fabs(pos_error) > 1e-6){
                if ( joint->state != STATE_JOINT_BREAKING ) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: motor pos inconsistent! pos_error = %f pos in %f motor in %f motor offset %f joint %d\n", 
                        pos_error, *joint->pos_cmd_in, *joint->motor_pos_cmd_in, *joint->motor_offset, n);
                }
                joint->state=STATE_JOINT_BREAKING;
                *joint->fault_out=1;
                data->state=STATE_BREAKING;
            }

            double v = *joint->vel_cmd_in;
            double stop_dist = v * v / ( 2 * joint->acc_max );

            //Only pos_cmd_in / pos_fb_in needs to be checked due to ^^
            double max_pos_vel = joint->max_pos_limit - stop_dist + tol1;
            if (v > 0 && (*joint->pos_cmd_in > max_pos_vel || *joint->pos_fb_in > max_pos_vel)) {
                if ( joint->state != STATE_JOINT_BREAKING ) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: limits will be exceided (max_pos_limit)! stop dist = %f pos in %f pos fb %f vel %f limit %f joint %d\n", 
                        stop_dist, *joint->pos_cmd_in, *joint->pos_fb_in, *joint->vel_cmd_in, joint->max_pos_limit, n);
                }
                joint->state=STATE_JOINT_BREAKING;
                *joint->fault_out=1;
                data->state=STATE_BREAKING;
            }
            double min_pos_vel = joint->min_pos_limit + stop_dist - tol1;
            if (v < 0 && (*joint->pos_cmd_in < min_pos_vel || *joint->pos_fb_in < min_pos_vel)) {
                if ( joint->state != STATE_JOINT_BREAKING ) {
                    rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: ERROR: limits will be exceided (min_pos_limit)! stop dist = %f pos in %f pos fb %f vel %f limit %f joint %d\n", 
                        stop_dist, *joint->pos_cmd_in, *joint->pos_fb_in, *joint->vel_cmd_in, joint->min_pos_limit, n);
                }
                joint->state=STATE_JOINT_BREAKING;
                *joint->fault_out=1;
                data->state=STATE_BREAKING;
            }
        }
    }else if(data->state == STATE_BREAKING){
        //Pass trough only limit switches
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            *joint->pos_lim_sw_out = *joint->pos_lim_sw_in;
            *joint->neg_lim_sw_out = *joint->neg_lim_sw_in;

            //Pretend motor is doing what is expected, so there is no following error
            //ToDo: Shows wrong position in UI but a following error will trigger a shutdown so we can not break any more
            *joint->motor_pos_fb_out = *joint->motor_pos_cmd_in;
        }

        bool done = true;
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            if(joint->state==0){
                continue;
            }
            double acc_cmd_out_old = *joint->acc_cmd_out;
            if(*joint->vel_cmd_out > 0){
                    *joint->acc_cmd_out = - joint->acc_max;
            }else{
                    *joint->acc_cmd_out = + joint->acc_max;
            }
            *joint->vel_cmd_out += *joint->acc_cmd_out * dt;
            //Check for sign change: If old and nev acc have different sign, we are done
            if( acc_cmd_out_old * *joint->acc_cmd_out < 0){
                joint->state=STATE_JOINT_OK;
                *joint->acc_cmd_out = 0;
                *joint->vel_cmd_out = 0;
            }else{
                done = false;
            }
            *joint->motor_pos_cmd_out += *joint->vel_cmd_out * dt;
            *joint->pos_cmd_out += *joint->vel_cmd_out * dt;
        }
        if(done){
            rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: stop finalized\n");
            data->state = STATE_STOPPED;
        }
    }else if(data->state == STATE_STOPPED){
        //Pass trough only limit switches
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            *joint->pos_lim_sw_out = *joint->pos_lim_sw_in;
            *joint->neg_lim_sw_out = *joint->neg_lim_sw_in;

            //Pretend motor is doing what is expected, so there is no following error
            //ToDo: Shows wrong position in UI but a following error will trigger a shutdown so we can not break any more
            *joint->motor_pos_fb_out = *joint->motor_pos_cmd_in;
        }

        //Wait for being unhomed to reset
        bool homed=false;
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            if(*joint->homed){
                homed=true;
            }
        }
        if(!homed){
            for (n = 0; n < num_joints; n++) {
                soft_limits_joint_t *joint = &data->joints[n];
                *joint->fault_out=0;
            }
            data->state = STATE_OK;
            rtapi_print_msg(RTAPI_MSG_ERR, "soft_limits: reset fault\n");
        }
    }else if(data->state == STATE_FAULT){
        //Pass trough only limit switch rising edge
        for (n = 0; n < num_joints; n++) {
            soft_limits_joint_t *joint = &data->joints[n];
            if(*joint->pos_lim_sw_in){
                *joint->pos_lim_sw_out = *joint->pos_lim_sw_in;
            }
            if(*joint->neg_lim_sw_in){
                *joint->neg_lim_sw_out = *joint->neg_lim_sw_in;
            }
        }

        *data->estop_out = 1;
    }
}

/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/
