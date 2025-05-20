#ifndef __UARTLCD_BSP_H_
#define __UARTLCD_BSP_H_

#include "string.h"
#include "stdio.h"
#include "receive_task.h"

#define UARTLCD_USE_PORT    uart1_inf // 使用的串口
#define USE_KEY_NUM         6         // 使用的按键数量
#define USE_DMA             1         // 是否使用DMA发生数据


typedef struct 
{
    uint8_t key[USE_KEY_NUM]; // 按键值
    uint8_t flag;             // 串口屏连接状态
    uint8_t index;            // 当前页面索引
    uint8_t show_index;       // 显示索引
    UART_HandleTypeDef *huart;// 串口句柄
    char data[20];            // 接收数据
}uLcd_t;

extern uLcd_t uLcd_inf;

void uLcd_analysis(void);
void uLcd_transmit(void);

#endif
