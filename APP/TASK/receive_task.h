#ifndef __RECEIVE_TASK_H_
#define __RECEIVE_TASK_H_

#include "usart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"

#include "OLED.h"
#include "key_bsp.h"
#include "vofa_bsp.h"
#include "uartLcd_bsp.h"
#include "ecoMotor_bsp.h"
#include "task.h"

extern TaskHandle_t receive_task_handle;

typedef struct
{
    uint8_t rx_buffer[50];
    uint8_t rx_data[50];
    uint8_t flag;
}usart_t;

extern usart_t uart1_inf, uart2_inf, uart6_inf;

/********************************************** USART1 START ******************************************************/
void USART1_receive_IT(void);
/********************************************** USART1 END ******************************************************/

/********************************************** JY901S START ******************************************************/
#define USART2_RX_SIZE 44// JY901S 数据包长度,回传速率设置的200HZ

enum{
    Ax = 0,
    Ay,
    Az,
    ACC_NUM,
    Wx = 0,
    Wy,
    Wz,
    GYRO_NUM,
    ROLL = 0,
    PITCH,
    YAW,
    ANGLE_NUM,
};

typedef struct
{
    int value_init[ANGLE_NUM];//初始化角度
    int value_change[ANGLE_NUM];//基于初始化角度的，角度变化
    volatile uint8_t flag_init[ANGLE_NUM];//是否初始化
}angle_t;
extern angle_t angle_inf;

typedef struct {
    float acc[ACC_NUM];     // 加速度，单位：g
    float gyro[GYRO_NUM];   // 角速度，单位：°/s
    float angle[ANGLE_NUM]; // 角度，  单位：°
    float mag[3];           // 磁场，  单位：uT
    float temperature;      // 温度，  单位：°C
} JY901_Data_t;

extern JY901_Data_t jy901_data;

void JY901S_receive_IT(void);
void JY901_LOG(uint8_t num);
/********************************************** JY901S END ******************************************************/

/********************************************** USART6 START ******************************************************/
void USART6_receive_IT(void);
/********************************************** USART6 END ******************************************************/
void receive_init(void);
void receive_Task(void *param);


/********************************************** OTHER_RECEIVE START ******************************************************/
extern TaskHandle_t other_receive_TASk_handle;

void other_receive_TASk(void *param);
/********************************************** OTHER_RECEIVE END ******************************************************/

#endif
