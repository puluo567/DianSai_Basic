#include "key_bsp.h"

#define USE_DOUBLE_CLICK 0 //是否使用双击功能

Key_t KEY[4] = {
    {GPIOB, GPIO_PIN_5, 0, 0, 0, 0, 0},
    {GPIOB, GPIO_PIN_4, 0, 0, 0, 0, 0},
    {GPIOC, GPIO_PIN_12, 0, 0, 0, 0, 0},
    {GPIOD, GPIO_PIN_2, 0, 0, 0, 0, 0}
};

void key_scan(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        uint32_t now_time = HAL_GetTick();
        if(HAL_GPIO_ReadPin(KEY[i].GPIOX, KEY[i].GPIO_Pin) == RESET)
        {
            if(KEY[i].flag == 0)
            {
                KEY[i].press_time = now_time;
                KEY[i].flag = 1;
            }
            if(KEY[i].flag == 1)
            {
                if(now_time - KEY[i].press_time >= 10)
                {
                    KEY[i].press_time = now_time;
                    KEY[i].flag = 2;
                }
            }
            if(KEY[i].flag == 2)
            {
                if(now_time - KEY[i].press_time >= 1000)
                {
                    KEY[i].flag = 3;
                    KEY[i].value = 2;
                    KEY[i].count = 0;
                }
            }
        }
        else
        {
            if(KEY[i].flag == 2)
            {
#if (!USE_DOUBLE_CLICK)
                KEY[i].value = 1;
#endif
                KEY[i].releas_time = now_time;
                KEY[i].count++;
            }
            KEY[i].flag = 0;
        }
        
#if (USE_DOUBLE_CLICK)
       if((KEY[i].count > 0) && (now_time - KEY[i].releas_time >= 100))
       {
           switch(KEY[i].count)
           {
               case 1:
                   KEY[i].value = 1;break;
               case 2:
                   KEY[i].value = 3;break;
               case 3:
                   KEY[i].value = 4;break;
           }
           KEY[i].count = 0;
           KEY[i].flag = 0;
       }
#endif
    }
}

void KEY_LOG(uint8_t num)
{
    char str[20];
    switch(num)
    {
        case 0:
            printf("KEY: %d, %d, %d, %d\r\n", KEY[0].value, KEY[1].value, KEY[2].value, KEY[3].value);
            break;
        case 1:
            sprintf(str, "%d %d %d %d", KEY[0].value, KEY[1].value, KEY[2].value, KEY[3].value);
            OLED_ShowString(0, 0, str, OLED_8X16);
            OLED_Update();
            break;
        default:
            break;
    }
}
