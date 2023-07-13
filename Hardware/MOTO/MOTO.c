#include "stm32f4xx_hal.h"
#include "main.h"
#include "MOTO.h"
#include "HX711.h"
#include "LCD1602.h"






// L6219步进电机驱动模块初始化函数
void L6219_Init(void)
{
    // 初始化步进电机控制引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin =IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(L6219_GPIO_Port, &GPIO_InitStruct);

    // 初始化按键引脚
    GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);
}


/*
 * PHA = 0;                      PHB =1;                     A1=0;        A2= O;                 B1=1;                B2=1; delay(time);//A负
 * PHA = 0;                      PHB = i;                    A1=0;        A2=O;                  B1=O;                B2 = O; delay(time);//A负B正
 * PHA = 1;                       PHB = 1;                   A1=1         A2 = 1;               B1 =Q;                 B2 = 0; delay(time);//B正
 * PHA = 1;                      PHB = 1;                    A1=0         A2=O;                B1 = O;                B2 =0;delay(time);//B正A正
 * PHA = 1;                      PHB = 0;                    A1=0         A2 =C;               B1 =1;                 B2 = 1; delay(time);//A正
 * PHA = 1;                      PHB = O;                    A1=0         A2=O;               B1 =0;               B2  =O; delay(time);//A正B负
 * PHA = 0;                       PHB = 0;                   A1=1         A2=1;                B1= O;               B 2= 0; delay(time);//B负
 * PHA = 0;                       PHB = O;                   A1=0         A2=0;               B1=O;              B2   =o; delay(time);//B负A负
 * */


void Step_CounterClockwise(void)
{
    static uint8_t step = 0;
    step = (step +1) % 4;
   // HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin, GPIO_PIN_RESET);
    switch (step)
    {
        case 0: // 01
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L6219_GPIO_Port,  PHASEB_Pin, GPIO_PIN_SET);

            break;
        case 1: // 11
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_SET);

            break;
        case 2: // 10
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
            break;
        case 3: // 00
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
            break;
    }
    //delay_us(100);

   HAL_Delay(20); // 延时2毫秒

}

void Step_Clockwise(void)//往机器方向
{
    static uint8_t step = 0;
    step = (step + 1) % 4;
    //HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin, GPIO_PIN_RESET);
    switch (step)
    {
        case 0: // 00
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
            break;
        case 1: // 10
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
            break;
        case 2: // 11
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_SET);
            break;
        case 3: // 01
            HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(L6219_GPIO_Port,  PHASEB_Pin, GPIO_PIN_SET);
            break;
    }


   HAL_Delay(200); // 延时2毫秒
 //   HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
    // delay_us(1);
}



// 停止步进电机旋转
void L6219_Stop(void)
{
    HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin |IN2_Pin | IN3_Pin | IN4_Pin, GPIO_PIN_RESET);
}





