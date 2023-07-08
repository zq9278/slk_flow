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

// ��ʱ����ת�������
// ˳ʱ����ת
void Step_CounterClockwise(void )
{
    static uint8_t step = 0;
    step = (step + 1) % 8;

    switch (step)
    {
        case 0:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin|IN4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
            break;
        case 7:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN4_Pin|IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
            break;
    }

    delay_us(20);
}

// ��ʱ����ת
void Step_Clockwise( void)
{
    static uint8_t step = 0;
    step = (step + 1) % 8;

    switch (step)
    {
        case 0:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin|IN3_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN4_Pin|IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN2_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 5:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin|IN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 6:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
        case 7:
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN2_Pin|IN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(L6219_GPIO_Port, IN3_Pin|IN4_Pin, GPIO_PIN_RESET);
            break;
    }

    delay_us(20);
}



// ֹͣ���������ת
void L6219_Stop(void)
{
    HAL_GPIO_WritePin(L6219_GPIO_Port, IN1_Pin |IN2_Pin | IN3_Pin | IN4_Pin, GPIO_PIN_RESET);
}





