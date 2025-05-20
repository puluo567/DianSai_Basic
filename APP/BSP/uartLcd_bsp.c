#include "uartLcd_bsp.h"

/*
    串口屏传输格式：帧头 (0x55 0x01) + 数据 + 帧尾 (0xff)
    波特率 115200
*/

uLcd_t uLcd_inf = {.huart = &huart1, .flag = 1, .index = 0};//记得修改为使用的串口句柄

//串口屏数据解析
void uLcd_analysis(void)
{
    if(UARTLCD_USE_PORT.rx_data[0] == 0x55 && UARTLCD_USE_PORT.rx_data[1] == 0x01) //排除非法包头
    {
        uint8_t len;
        for(uint8_t i = 0; i < sizeof(UARTLCD_USE_PORT.rx_data); i++)
        {
            if(UARTLCD_USE_PORT.rx_data[i + 2] == 0xff)//检测到帧尾
            {
                len = i;
                break;
            }
        }
        if(len <= 2)
            return;
        //这才开始解析数据
        memcpy(uLcd_inf.data, UARTLCD_USE_PORT.rx_data + 2, len);
        memset(UARTLCD_USE_PORT.rx_data, 0x00, sizeof(UARTLCD_USE_PORT.rx_data));
        uLcd_inf.data[len + 1] = '\0'; //将数据末尾置为'\0'
        //获取当前页面索引
        if(sscanf(uLcd_inf.data, "page:%d", &uLcd_inf.index))
        {
            uLcd_inf.flag = 1;
            memset(uLcd_inf.data, 0x00, len);
            return;
        }
        //获取按键值
        uint8_t key_index;
        if(sscanf(uLcd_inf.data, "KEY%d", &key_index)) //判断是否为按键按下
        {
            if(key_index > USE_KEY_NUM || key_index < 1) //按键索引越界
                return;
            uLcd_inf.key[key_index - 1] = 1; //按键按下
            memset(uLcd_inf.data, 0x00, len);
            return;
        }
    }
}

static void basic_transmit(char *str1, char *str2)
{
    char uLcd_str[50];
    sprintf(uLcd_str, "%s=\"%s\"\xff\xff\xff", str1, str2);
#if(USE_DMA)
    HAL_UART_Transmit_DMA(uLcd_inf.huart, (uint8_t *)uLcd_str, strlen(uLcd_str));
#else
    HAL_UART_Transmit(uLcd_inf.huart, (uint8_t *)uLcd_str, strlen(uLcd_str), 10);
#endif
    vTaskDelay(3);
}

//串口屏数据发送
void uLcd_transmit(void)
{
    // if(!uLcd_inf.flag)//必须等刷新过后再发送
    //     return;
    // vTaskDelay(5);
    // if(uLcd_inf.index == 0) //当前页面为主页面
    // {
    //     //1.连接状态发送
    //     basic_transmit("page0.t3.txt", "connect ok");

    //     //2.角度值发送
    //     char angle_str[30];
    //     sprintf(angle_str, "ROLL = %d", angle_inf.value_change[ROLL]);
    //     basic_transmit("page0.t0.txt", angle_str);
    //     sprintf(angle_str, "PITCH = %d", angle_inf.value_change[PITCH]);
    //     basic_transmit("page0.t1.txt", angle_str);
    //     sprintf(angle_str, "YAW = %d", angle_inf.value_change[YAW]);
    //     basic_transmit("page0.t2.txt", angle_str);
    // }
    // else if(uLcd_inf.index == 1) //当前页面为PID页面
    // {
    //     //3.PID值发送
    //     char pid_str[30];
    //     sprintf(pid_str, "PID1.P = %.2f", PID_CHANGE_1.kp);
    //     basic_transmit("page1.t0.txt", pid_str);
    //     sprintf(pid_str, "PID1.I = %.2f", PID_CHANGE_1.ki);
    //     basic_transmit("page1.t1.txt", pid_str);
    //     sprintf(pid_str, "PID1.D = %.2f", PID_CHANGE_1.kd);
    //     basic_transmit("page1.t2.txt", pid_str);
    //     sprintf(pid_str, "PID2.P = %.2f", PID_CHANGE_2.kp);
    //     basic_transmit("page1.t3.txt", pid_str);
    //     sprintf(pid_str, "PID2.I = %.2f", PID_CHANGE_2.ki);
    //     basic_transmit("page1.t6.txt", pid_str);
    //     sprintf(pid_str, "PID2.D = %.2f", PID_CHANGE_2.kd);
    //     basic_transmit("page1.t7.txt", pid_str);
    // }

    if(uLcd_inf.show_index == 0)
    {
        //1.连接状态发送
        basic_transmit("page0.t3.txt", "connect ok");
        basic_transmit("page0.t4.txt", "JY901 DATA");
        //2.角度值发送
        char angle_str[30];
        sprintf(angle_str, "ROLL = %d", angle_inf.value_change[ROLL]);
        basic_transmit("page0.t0.txt", angle_str);
        sprintf(angle_str, "PITCH = %d", angle_inf.value_change[PITCH]);
        basic_transmit("page0.t1.txt", angle_str);
        sprintf(angle_str, "YAW = %d", angle_inf.value_change[YAW]);
        basic_transmit("page0.t2.txt", angle_str);
    }
    else if(uLcd_inf.show_index == 1)
    {
        basic_transmit("page0.t4.txt", "PID1 DATA");
        //3.PID值发送
        char pid_str[30];
        sprintf(pid_str, "PID1.P = %.2f", PID_CHANGE_1.kp);
        basic_transmit("page0.t0.txt", pid_str);
        sprintf(pid_str, "PID1.I = %.2f", PID_CHANGE_1.ki);
        basic_transmit("page0.t1.txt", pid_str);
        sprintf(pid_str, "PID1.D = %.2f", PID_CHANGE_1.kd);
        basic_transmit("page0.t2.txt", pid_str);
    }
    else if(uLcd_inf.show_index == 2)
    {
        basic_transmit("page0.t4.txt", "PID2 DATA");
        //3.PID值发送
        char pid_str[30];
        sprintf(pid_str, "PID2.P = %.2f", PID_CHANGE_2.kp);
        basic_transmit("page0.t0.txt", pid_str);
        sprintf(pid_str, "PID2.I = %.2f", PID_CHANGE_2.ki);
        basic_transmit("page0.t1.txt", pid_str);
        sprintf(pid_str, "PID2.D = %.2f", PID_CHANGE_2.kd);
        basic_transmit("page0.t2.txt", pid_str);
    }
}
