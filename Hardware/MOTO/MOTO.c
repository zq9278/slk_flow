#include "stm32f4xx_hal.h"
#include "main.h"
#include "MOTO.h"
#include "HX711.h"
#include "LCD1602.h"






// L6219�����������ģ���ʼ������
void L6219_Init(void)
{
    // ��ʼ�����������������
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin =IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(L6219_GPIO_Port, &GPIO_InitStruct);

    // ��ʼ����������
    GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);
}


/*
 * PHA = 0;                      PHB =1;                     A1=0;        A2= O;                 B1=1;                B2=1; delay(time);//A��
 * PHA = 0;                      PHB = i;                    A1=0;        A2=O;                  B1=O;                B2 = O; delay(time);//A��B��
 * PHA = 1;                       PHB = 1;                   A1=1         A2 = 1;               B1 =Q;                 B2 = 0; delay(time);//B��
 * PHA = 1;                      PHB = 1;                    A1=0         A2=O;                B1 = O;                B2 =0;delay(time);//B��A��
 * PHA = 1;                      PHB = 0;                    A1=0         A2 =C;               B1 =1;                 B2 = 1; delay(time);//A��
 * PHA = 1;                      PHB = O;                    A1=0         A2=O;               B1 =0;               B2  =O; delay(time);//A��B��
 * PHA = 0;                       PHB = 0;                   A1=1         A2=1;                B1= O;               B 2= 0; delay(time);//B��
 * PHA = 0;                       PHB = O;                   A1=0         A2=0;               B1=O;              B2   =o; delay(time);//B��A��
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

   HAL_Delay(20); // ��ʱ2����

}

void Step_Clockwise(void)//����������
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


   HAL_Delay(200); // ��ʱ2����
 //   HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEA_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(L6219_GPIO_Port, PHASEB_Pin, GPIO_PIN_RESET);
    // delay_us(1);
}



// ֹͣ���������ת
void L6219_Stop(void)
{
    HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin |IN2_Pin | IN3_Pin | IN4_Pin, GPIO_PIN_RESET);
}





