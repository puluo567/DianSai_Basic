#ifndef __VOFA_BSP_H_
#define __VOFA_BSP_H_

#include "main.h"
#include "usart.h"
#include "receive_task.h"
#include "pid_bsp.h"

#define VOFA_USE_PORT_COM    uart6_inf // 使用的串口
#define PID_CHANGE_1         pid_motor_a // 需要调的pid
#define PID_CHANGE_2         pid_car_angle

extern pid_t pid_test1, pid_test2;//测试用的pid

void vofa_data_analysis(void);

#endif
