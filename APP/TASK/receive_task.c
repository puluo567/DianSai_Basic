#include "receive_task.h"

TaskHandle_t receive_task_handle = NULL;

usart_t uart1_inf, uart2_inf, uart6_inf;

void receive_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, uart1_inf.rx_buffer, sizeof(uart1_inf.rx_buffer));
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, uart2_inf.rx_buffer, USART2_RX_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, uart6_inf.rx_buffer, sizeof(uart6_inf.rx_buffer));
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
}

/********************************************** USART1 START ******************************************************/
void USART1_receive_IT(void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        uart1_inf.flag = 1;
        HAL_UART_DMAStop(&huart1);
        memcpy(uart1_inf.rx_data, uart1_inf.rx_buffer, sizeof(uart1_inf.rx_buffer));
        // uint8_t len = sizeof(uart1_inf.rx_buffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        // uart1_inf.rx_data[len] = '\0';
        memset(uart1_inf.rx_buffer, 0x00, sizeof(uart1_inf.rx_buffer));
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_Receive_DMA(&huart1, uart1_inf.rx_buffer, sizeof(uart1_inf.rx_buffer));
    }
}
/********************************************** USART1 END ******************************************************/

/********************************************** JY901S START ******************************************************/

/***********************************************
 * 
    说明：angle_inf 结构体获取的是相对于初始角度的变化量 范围 +180 ~ -180 ， 也可以直接使用JY901S的角度数据
    例如：初始角度为 0°，当前角度为 90°，则 angle_inf.value_change = 90°

***********************************************/

JY901_Data_t jy901_data;//陀螺仪数据结构体
angle_t angle_inf;//角度数据结构体

// JY901S 数据解析 回传速率设置的200Hz
static int16_t combie_data(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static void JY901S_check(void)
{
    if(uart2_inf.rx_data[0] == 0x55 && uart2_inf.rx_data[11] == 0x55 && uart2_inf.rx_data[22] == 0x55 && uart2_inf.rx_data[33] == 0x55)//排除非法包头
        uart2_inf.flag = 1;//接收到有效数据后再给标志位置1
}

/*'''
　　┏┓　　　┏┓+ +
　┏┛┻━━━┛┻┓ + +
　┃　　　　　　　┃ 　
　┃　　　━　　　┃ ++ + + +
 ████━████ ┃+
　┃　　　　　　　┃ +
　┃　　　┻　　　┃
　┃　　　　　　　┃ + +
　┗━┓　　　┏━┛
　　　┃　　　┃　　　　　　　　　　　
　　　┃　　　┃ + + + +                    JY901S    周围远离一切导线 ！！！！！！！！！！！！！！
　　　┃　　　┃                            JY901S    周围远离一切导线 ！！！！！！！！！！！！！！
　　　┃　　　┃ +  神兽保佑                 JY901S    周围远离一切导线 ！！！！！！！！！！！！！！
　　　┃　　　┃    代码无bug　　
　　　┃　　　┃　　+　　　　　　　　　
　　　┃　 　　┗━━━┓ + +
　　　┃ 　　　　　　　┣┓
　　　┃ 　　　　　　　┏┛
　　　┗┓┓┏━┳┓┏┛ + + + +
　　　　┃┫┫　┃┫┫
　　　　┗┻┛　┗┻┛+ + + +

'''*/

static void JY901S_analysis(void)
{
    uint8_t DATA[11];
    for(uint8_t i = 0; i < 4; i++)
    {
        memcpy(DATA, &uart2_inf.rx_data[i * 11], 11);
        if(DATA[0] == 0x55)//排除非法包头
        {
            uint16_t data[4];
            uint16_t check_sum = 0;
            
            for(uint8_t i = 0; i < 10; i++)
            {
                check_sum += DATA[i];
            }
            if((uint8_t)(check_sum & 0xFF) == DATA[10])//校验和取低八位
            {
                data[0] = combie_data(DATA[2], DATA[3]);
                data[1] = combie_data(DATA[4], DATA[5]);
                data[2] = combie_data(DATA[6], DATA[7]);
                data[3] = combie_data(DATA[8], DATA[9]);
                switch(DATA[1])
                {
                    case 0x51: // 加速度
                        jy901_data.acc[0] = (float)data[0] / 32768.0f * 16.0f;
                        jy901_data.acc[1] = (float)data[1] / 32768.0f * 16.0f;
                        jy901_data.acc[2] = (float)data[2] / 32768.0f * 16.0f;
                        jy901_data.temperature = (float)data[3] / 100.0f;//温度
                        break;
                    case 0x52: // 角速度
                        jy901_data.gyro[0] = (float)data[0] / 32768.0f * 2000.0f;
                        jy901_data.gyro[1] = (float)data[1] / 32768.0f * 2000.0f;
                        jy901_data.gyro[2] = (float)data[2] / 32768.0f * 2000.0f;
                        break;
                    case 0x53: // 姿态角
                        jy901_data.angle[0] = (float)data[0] / 32768.0f * 180.0f;//ROLL
                        jy901_data.angle[1] = (float)data[1] / 32768.0f * 180.0f;//PITCH
                        jy901_data.angle[2] = (float)data[2] / 32768.0f * 180.0f;//YAW
                        break;
                    case 0x54: // 地磁
                        jy901_data.mag[0] = (float)data[0] * 0.00833f;//X
                        jy901_data.mag[1] = (float)data[1] * 0.00833f;//Y
                        jy901_data.mag[2] = (float)data[2] * 0.00833f;//Z
                        jy901_data.temperature = (float)data[3] / 100.0f;//温度
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

static int calculate_angle_change(int current_angle, int reference_angle) 
{
    int diff = current_angle - reference_angle;
    // 处理跨越±180°的情况
    if (diff > 180) {
        diff -= 360;
    } else if (diff < -180) {
        diff += 360;
    }
    return diff;
}

//相对角度方向采用右手螺旋法则
void Angle_calculate(void) 
{
    for(uint8_t i = 0; i < ANGLE_NUM; i++)
    {
        if(!angle_inf.flag_init[i])//第一次接收数据时初始化角度
        {
            angle_inf.value_init[i] = jy901_data.angle[i];
            angle_inf.flag_init[i] = 1;
        }
        else//后续数据计算
        {
            angle_inf.value_change[i] = calculate_angle_change(jy901_data.angle[i], angle_inf.value_init[i]);
        }
    }
}

void JY901S_receive_IT(void)
{
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        memcpy(uart2_inf.rx_data, uart2_inf.rx_buffer, USART2_RX_SIZE);
        JY901S_check();//检测数据是否有效
        memset(uart2_inf.rx_buffer, 0x00, sizeof(uart2_inf.rx_buffer));
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        HAL_UART_Receive_DMA(&huart2, uart2_inf.rx_buffer, USART2_RX_SIZE);
    }
}



void JY901_LOG(uint8_t num)
{
    char str[20];
    switch(num)
    {
        case 0:
            printf("%.2f,%.2f,%.2f\r\n", jy901_data.acc[Ax], jy901_data.acc[Ay], jy901_data.acc[Az]);
            break;
        case 1:
            printf("%.2f,%.2f,%.2f\r\n", jy901_data.gyro[Wx], jy901_data.gyro[Wy], jy901_data.gyro[Wz]);
            break;
        case 2:
            printf("%.2f,%.2f,%.2f\r\n", jy901_data.angle[ROLL], jy901_data.angle[PITCH], jy901_data.angle[YAW]);
            break;
        case 3:
            printf("%d, %d, %d\r\n", angle_inf.value_change[ROLL], angle_inf.value_change[PITCH], angle_inf.value_change[YAW]);
            break;
        case 4:
            OLED_Clear();
            sprintf(str, "%d %d %d", angle_inf.value_change[ROLL], angle_inf.value_change[PITCH], angle_inf.value_change[YAW]);
            OLED_ShowString(0, 0, str, OLED_8X16);
            sprintf(str, "%d %d %d", angle_inf.value_init[ROLL], angle_inf.value_init[PITCH], angle_inf.value_init[YAW]);
            OLED_ShowString(0, 16, str, OLED_8X16);
            OLED_Update();
            break;
        default:
            break;
    }
}
/********************************************** JY901S END ******************************************************/

/********************************************** USART6 START ******************************************************/
void USART6_receive_IT(void)
{
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        uart6_inf.flag = 1;
        HAL_UART_DMAStop(&huart6);
        memcpy(uart6_inf.rx_data, uart6_inf.rx_buffer, sizeof(uart6_inf.rx_buffer));
        uint8_t len = sizeof(uart6_inf.rx_buffer) - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        uart6_inf.rx_data[len] = '\0';
        memset(uart6_inf.rx_buffer, 0x00, sizeof(uart6_inf.rx_buffer));
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_Receive_DMA(&huart6, uart6_inf.rx_buffer, sizeof(uart6_inf.rx_buffer));
    }
}
/********************************************** USART6 END ******************************************************/

void receive_Task(void *param) //200HZ的回传速率
{
    while(1)
    {
        if(uart2_inf.flag) //接收到有效数据后再进行后面的操作
        {
            uart2_inf.flag = 0;
            JY901S_analysis();
            Angle_calculate();//如果需要在途中更改初始值，仅需令 angle_inf[i].flag_init = 0 即可
        }
//        JY901_LOG(4);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
/********************************************** OTHER_RECEIVE START ******************************************************/
TaskHandle_t other_receive_TASk_handle = NULL;

void other_receive_TASk(void *param)
{
    while(1)
    {
        vofa_data_analysis();
        uLcd_analysis();
        uLcd_transmit();
        if(KEY[0].value == 1)
        {
            KEY[0].value = 0;
            uLcd_inf.show_index++;
            if(uLcd_inf.show_index > 2)
                uLcd_inf.show_index = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/********************************************** OTHER_RECEIVE END ******************************************************/


