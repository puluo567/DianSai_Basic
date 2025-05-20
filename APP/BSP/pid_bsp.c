#include "pid_bsp.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float min_output, float integral_max, float integral_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
}

/*
 * @brief: pid控制器 位置式
 * @param: pid: pid结构体指针
 * @param: desire: 目标值
 * @param: dt: 采样时间
 * @return: 无
 */
void pid_update_position(pid_t *pid, float measure, float desire, float dt)
{
    if (dt <= 0.0f) return; // 检查 dt 是否有效
    pid->mesure = measure;//测量值更新
    pid->desire = desire;
    pid->error = desire - pid->mesure;//当前误差计算

    // 增积分项（带死区）如果误差太小（例如 <0.01），可以不继续积分，避免慢慢积累“残余误差”
    // if(fabsf(pid->error) > 0.01f)
        pid->integral += pid->error * dt;
    
    LIMIT_MIN_MAX(pid->integral, pid->integral_min, pid->integral_max);//限制积分范围

    pid->derivative = (pid->error - pid->last_error) / dt;//一阶差分的微分
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;//输出

    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;

    LIMIT_MIN_MAX(pid->output, pid->min_output, pid->max_output);//限制输出范围
}



/*
 * @brief: pid控制器 增量式 
 * @param: pid: pid结构体指针
 * @param: desire: 目标值
 * @param: dt: 采样时间
 * @return: 无
 */
void pid_update_increment(pid_t *pid, float measure, float desire, float dt)
{
    if (dt <= 0.0f) return; // 检查 dt 是否有效
    pid->mesure = measure;//测量值更新
    pid->desire = desire;
    pid->error = desire - pid->mesure;//当前误差计算

    // 增积分项（带死区）如果误差太小（例如 <0.01），可以不继续积分，避免慢慢积累“残余误差”
    // if(fabsf(pid->error) > 0.01f)
        pid->integral += pid->error * dt;

    LIMIT_MIN_MAX(pid->integral, pid->integral_min, pid->integral_max);//限制积分范围

    //这是一个二阶差分公式，理论上可以提高微分的精度，但在实际应用中可能会对噪声较为敏感，尤其是当 dt 较小时，噪声会被放大。为了减少噪声的影响，通常会使用一阶差分公式代替二阶差分公式
    // pid->derivative = (pid->error + pid->last_last_error - 2.0f * pid->last_error) / dt;//二阶差分的微分
    pid->derivative = (pid->error - pid->last_error) / dt;//一阶差分的微分
    //通常，增量式 PID 的积分项是直接累加误差的增量
    pid->output += pid->kp * (pid->error - pid->last_error) + pid->ki * pid->integral + pid->kd * pid->derivative;//输出

    // 更新误差
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;

    LIMIT_MIN_MAX(pid->output, pid->min_output, pid->max_output);//限制输出范围
}
