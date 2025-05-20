#ifndef __DRIVER_TIM_H_
#define __DRIVER_TIM_H_

#include "tim.h"

#define DLY_TIM_Handle (&htim7)	
 
 /*微秒级别延时*/
void Tims_delay_us(uint16_t nus);

#endif
