//
// Created by QQZZ on 2026/11/2.
//
#include "pid.h"
#include<stddef.h>

void PID_Init(PID_t *pid,float kp,float ki,float kd,
    float max_output,float integral_limit) {
    if (pid == NULL ) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target =0.0f;
    pid->feedback=0.0f;

    pid->error=0.0f;
    pid->last_error=0.0f;

    pid->integral =0.0f;
    pid->output = 0.0f;

    pid->max_output = max_output;
    pid->integral_limit =integral_limit;
    pid->first_calc=1;

}

float PID_Calc(PID_t *pid,float dt) {
    if (pid == NULL || dt<=0.0f)
        return 0.0f;
    pid->error = pid->target-pid->feedback;
    float p_out = pid->kp * pid->error;

    pid->integral += pid->ki * pid->error*dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    float d_out;
    if (pid->first_calc!=1) {
        d_out=pid->kd *
(pid->error - pid->last_error)/dt;
    }
    else {
        d_out=0.0f;
        pid->first_calc=0;
    }

    pid->output = p_out + pid->integral + d_out;
    pid->last_error = pid->error;
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    }
    if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    return pid->output;

}

void PID_Reset(PID_t *pid) {
    if (pid == NULL ) {
        return;
    }

    pid->target =0.0f;
    pid->feedback=0.0f;

    pid->error=0.0f;
    pid->last_error=0.0f;

    pid->integral =0.0f;
    pid->output = 0.0f;

    pid->first_calc=1;
}