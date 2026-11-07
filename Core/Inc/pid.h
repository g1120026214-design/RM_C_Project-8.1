//
// Created by QQZZ on 2026/11/2.
//

#ifndef RM_C_PROJECT1_PID_H
#define RM_C_PROJECT1_PID_H
#include "stdint.h"
typedef struct {
    float kp;
    float ki;
    float kd;

    float target;
    float feedback;

    float error;
    float last_error;

    float max_output;
    float integral_limit;

    float integral;
    float output;
    uint8_t first_calc;

}PID_t;

void PID_Init(PID_t *pid,float kp,float ki,float kd,
    float max_output,float integral_limit);
float PID_Calc(PID_t *pid,float dt);
void PID_Reset(PID_t *pid);
#endif //RM_C_PROJECT1_PID_H
