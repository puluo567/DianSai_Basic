#include "task.h"

TaskHandle_t ecoMotor_Task_Handle = NULL;


void ecoMotor_Task(void *param)
{
    while(1)
    {
//        pid_control_motor_speed(&pid_motor_a, &ecoMotor_inf_A,  5);
//        pid_control_motor_speed(&pid_motor_b, &ecoMotor_inf_B,  5);
        if(KEY[1].value == 1)
            pid_control_motor_angle(&pid_car_angle, 90);
        if(uLcd_inf.key[0] == 1)
        {
            KEY[1].value = 0;
            uLcd_inf.key[0] = 0;
        }
        pid_log(&pid_car_angle, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void pid_log(pid_t *pid, uint8_t num)
{
    switch(num)
    {
        case 1:
            printf("%.2f, %.2f, %.2f\r\n", pid->mesure, pid->desire, pid->output);
            break;
        default:
            break;
    }
}
