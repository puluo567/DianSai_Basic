#ifndef __KEY_BSP_H_
#define __KEY_BSP_H_

#include "gpio.h"
#include "stdio.h"

#include "OLED.h"

typedef struct{
    GPIO_TypeDef *GPIOX;
    uint16_t GPIO_Pin;
    uint8_t value;//ֵ
    uint8_t flag;//��־λ
    uint8_t count;//�̰�����
    uint32_t releas_time;//�ɿ�ʱ��
    uint32_t press_time;//����ʱ��
}Key_t;

extern Key_t KEY[4];

void key_scan(void);
void KEY_LOG(uint8_t num);

#endif
