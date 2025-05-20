#ifndef __PID_BSP_H_
#define __PID_BSP_H_

#include "stdio.h"

#define LIMIT_MIN_MAX(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

typedef struct
{
    float kp, ki, kd;
    float mesure;//测量值
    float desire;//期望值
    float error;//误差
    float last_error;//上一次误差
    float last_last_error;//上上次误差
    float integral;//积分
    float derivative;//微分
    float output;//输出
    float max_output;//最大输出
    float min_output;//最小输出
    float integral_max;//积分限幅
    float integral_min;
}pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float min_output, float integral_max, float integral_min);
void pid_update_position(pid_t *pid, float measure, float desire, float dt);
void pid_update_increment(pid_t *pid, float measure, float desire, float dt);

#endif
