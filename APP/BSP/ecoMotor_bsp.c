#include "ecoMotor_bsp.h"

/*

使用的定时器5 通道 1，2  频率 1KHz  ARR -> 1000  TB6612 A: C1 C0, B: C2,A4

    左电机A 右电机B

*/

#define TIM_ARR                999          //定时器的自动重装值
#define EC_MOTOR_A_TIM_CHANNEL TIM_CHANNEL_1 //使用的定时器通道 
#define EC_MOTOR_B_TIM_CHANNEL TIM_CHANNEL_2 //使用的定时器通道

//定义编码器电机A,B
ecoMotor_t ecoMotor_inf_A, ecoMotor_inf_B;

pid_t pid_motor_a, pid_motor_b; //电机PID结构体
pid_t pid_car_angle;// yaw pid

void ecoMotor_init(void)
{
    //初始化相关的GPIO引脚，定时器等
    ecoMotor_inf_A.htim = &htim3;
    ecoMotor_inf_B.htim = &htim4;
    ecoMotor_inf_A.GPIO1 = GPIOC;
    ecoMotor_inf_A.GPIO2 = GPIOC;
    ecoMotor_inf_A.GPIO_Pin1 = GPIO_PIN_1;
    ecoMotor_inf_A.GPIO_Pin2 = GPIO_PIN_0;
    ecoMotor_inf_B.GPIO1 = GPIOA;
    ecoMotor_inf_B.GPIO2 = GPIOC;
    ecoMotor_inf_B.GPIO_Pin1 = GPIO_PIN_4;
    ecoMotor_inf_B.GPIO_Pin2 = GPIO_PIN_2;

	HAL_TIM_Encoder_Start(ecoMotor_inf_A.htim,TIM_CHANNEL_ALL );
	HAL_TIM_Encoder_Start(ecoMotor_inf_B.htim,TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim5, EC_MOTOR_A_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim5, EC_MOTOR_B_TIM_CHANNEL);
    __HAL_TIM_CLEAR_FLAG(&htim11,TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim11); //开启定时器11的中断

    pid_init(&pid_motor_a, 98.0f, 0.0f, 0.0f, 1000.0f, -1000.0f, 50.0f, -50.0f);
    pid_init(&pid_motor_b, 98.0f, 0.0f, 0.0f, 1000.0f, -1000.0f, 50.0f, -50.0f);
    pid_init(&pid_car_angle, -8.0f, -0.8f, -0.006f, 1000.0f, -1000.0f, 50.0f, -50.0f);
}

void ecoMotor_setSpeed(ecoMotor_t *motor, int16_t speed)
{
    LIMIT_MIN_MAX(speed, -TIM_ARR, TIM_ARR); //限制速度范围
    if(speed > 0)
    {
        HAL_GPIO_WritePin(motor->GPIO1, motor->GPIO_Pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->GPIO2, motor->GPIO_Pin2, GPIO_PIN_RESET);
    }
    else if(speed < 0)
    {
        HAL_GPIO_WritePin(motor->GPIO1, motor->GPIO_Pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->GPIO2, motor->GPIO_Pin2, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->GPIO1, motor->GPIO_Pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->GPIO2, motor->GPIO_Pin2, GPIO_PIN_RESET);
    }
    
    if(motor == &ecoMotor_inf_A)
    {
        __HAL_TIM_SET_COMPARE(&htim5, EC_MOTOR_A_TIM_CHANNEL, abs(speed));
    }
    else if(motor == &ecoMotor_inf_B)
    {
        __HAL_TIM_SET_COMPARE(&htim5, EC_MOTOR_B_TIM_CHANNEL, abs(speed));
    }
}

//速度环控制
void pid_control_motor_speed(pid_t *pid_motor, ecoMotor_t *motor, int16_t desire)
{
    pid_update_increment(pid_motor, (float)motor->speed, desire, 0.01f);//10ms的采样周期
    ecoMotor_setSpeed(motor, pid_motor->output);
}

//角度环,小车车身角度环  角度范围 +180 ~ -180
void pid_control_motor_angle(pid_t *pid, float desire_angle)
{
    pid_update_position(pid, angle_inf.value_change[YAW], desire_angle, 0.01f);//10ms的采样周期
    ecoMotor_setSpeed(&ecoMotor_inf_A, pid->output);
    ecoMotor_setSpeed(&ecoMotor_inf_B, -1.0f * pid->output);
}

//10ms中断，计算速度
void HAL_TIM_PeriodCallback_IT(TIM_HandleTypeDef *htim)
{
    if(htim == &htim11)
    {
        ecoMotor_inf_A.count = (short)(__HAL_TIM_GET_COUNTER(ecoMotor_inf_A.htim));
        __HAL_TIM_SET_COUNTER(ecoMotor_inf_A.htim, 0);
        ecoMotor_inf_A.speed = ecoMotor_inf_A.count;//先假定速度为10ms的计数值

        ecoMotor_inf_B.count = (short)(__HAL_TIM_GET_COUNTER(ecoMotor_inf_B.htim)) * -1.0;//统一电机方向
        __HAL_TIM_SET_COUNTER(ecoMotor_inf_B.htim, 0);
        ecoMotor_inf_B.speed = ecoMotor_inf_B.count;//先假定速度为10ms的计数值
//        printf("%d, %d\r\n", ecoMotor_inf_A.count, ecoMotor_inf_B.count);
    }
}

