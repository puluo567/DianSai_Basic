#ifndef __ECOMOTOR_BSP_H_
#define __ECOMOTOR_BSP_H_

#include "tim.h"
#include "pid_bsp.h"
#include "receive_task.h"

typedef struct 
{
    int16_t speed;                      // ����ٶ�
    int16_t count;                      // ����������ֵ
    TIM_HandleTypeDef *htim;            // ������ʹ�õĶ�ʱ���ľ��
    GPIO_TypeDef *GPIO1, *GPIO2;        // ����������ŵ�GPIO�˿�
    uint16_t GPIO_Pin1, GPIO_Pin2;      // ����������ŵ�GPIO�˿����ź�
}ecoMotor_t;

//������������A,B
extern ecoMotor_t ecoMotor_inf_A, ecoMotor_inf_B;

extern pid_t pid_motor_a, pid_motor_b;
extern pid_t pid_car_angle;// yaw pid
void ecoMotor_init(void);
void ecoMotor_setSpeed(ecoMotor_t *motor, int16_t speed);
void pid_control_motor_speed(pid_t *pid_motor, ecoMotor_t *motor, int16_t desire);
void pid_control_motor_angle(pid_t *pid, float desire_angle);
void HAL_TIM_PeriodCallback_IT(TIM_HandleTypeDef *htim);

#endif
