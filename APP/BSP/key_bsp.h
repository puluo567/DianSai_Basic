#ifndef __KEY_BSP_H_
#define __KEY_BSP_H_

#include "gpio.h"
#include "stdio.h"

#include "OLED.h"

typedef struct{
    GPIO_TypeDef *GPIOX;
    uint16_t GPIO_Pin;
    uint8_t value;//值
    uint8_t flag;//标志位
    uint8_t count;//短按次数
    uint32_t releas_time;//松开时间
    uint32_t press_time;//按下时间
}Key_t;

extern Key_t KEY[4];

void key_scan(void);
void KEY_LOG(uint8_t num);

#endif
