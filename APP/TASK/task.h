#ifndef __TASK_H_
#define __TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "receive_task.h"
#include "pid_bsp.h"

extern struct tskTaskControlBlock* ecoMotor_Task_Handle;// 把 struct tskTaskControlBlock* 换为TaskHandle_t会报错

void ecoMotor_Task(void *param);
void pid_log(pid_t *pid, uint8_t num);

#endif
