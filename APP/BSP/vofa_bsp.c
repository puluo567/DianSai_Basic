#include "vofa_bsp.h"

pid_t pid_test1, pid_test2;//测试用的pid

//vofa上位机传输格式 P1=%.2f! I1=%.2f! D1=%.2f! P2=%.2f! I2=%.2f! D2=%.2f!
typedef struct
{
    uint8_t num;
    char index;    // PID的索引
    float kp, kd, ki;
}vofa_t;

vofa_t vofa_inf;

void vofa_data_analysis(void)
{
    if(!VOFA_USE_PORT_COM.flag)
        return;
    char vofa_data[50];
    memcpy(vofa_data, VOFA_USE_PORT_COM.rx_data, sizeof(VOFA_USE_PORT_COM.rx_data));

    vofa_inf.num = vofa_data[1] - '0'; // 1 ~ 2   检查数据帧是否正确
    if(vofa_inf.num > 2 || vofa_inf.num < 1)
        return;

    vofa_inf.index = vofa_data[0];     // P I D
    if(vofa_inf.index != 'P' && vofa_inf.index != 'I' && vofa_inf.index != 'D')
        return;

    if(vofa_inf.num == 1)
    {
        switch (vofa_inf.index)
        {
            case 'P':
                sscanf(vofa_data, "P1=%f", &PID_CHANGE_1.kp);
                break;
            case 'I':
                sscanf(vofa_data, "I1=%f", &PID_CHANGE_1.ki);
                break;
            case 'D':
                sscanf(vofa_data, "D1=%f", &PID_CHANGE_1.kd);
                break;
            default:
                break;
        }
    }
    else if(vofa_inf.num == 2)
    {
        switch (vofa_inf.index)
        {
            case 'P':
                sscanf(vofa_data, "P2=%f", &PID_CHANGE_2.kp);
                break;
            case 'I':
                sscanf(vofa_data, "I2=%f", &PID_CHANGE_2.ki);
                break;
            case 'D':
                sscanf(vofa_data, "D2=%f", &PID_CHANGE_2.kd);
                break;
            default:
                break;
        }
    }
}
