#include "stm32f4xx_hal.h"
#include "main.h"
#include "MOTO.h"
#include "HX711.h"
#include "LCD1602.h"

// L6219�������Ŷ���


// �������Ŷ���
void L6219_Init(void)
{
    // ��ʼ��L6219��GPIO����
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ����GPIO����
    GPIO_InitStruct.Pin = MOTOR_PIN_A | MOTOR_PIN_B | MOTOR_PIN_C | MOTOR_PIN_D;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStruct);

    // ����GPIO����
    GPIO_InitStruct.Pin = BUTTON1_PIN | BUTTON2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUTTON1_GPIO_PORT, &GPIO_InitStruct);
    // �������ų�ʼ״̬Ϊ�͵�ƽ
    HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A | MOTOR_PIN_B | MOTOR_PIN_C | MOTOR_PIN_D, GPIO_PIN_RESET);
}
void Rotate_StepperMotor_Clockwise( )
{
    // ��ת˳��Ϊ��A -> AB -> B -> BC -> C -> CD -> D -> DA
    uint8_t sequence[8] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
    static uint8_t step_index = 0;

    // Ԥ��ѹ��������ֵ���ɸ�����Ҫ����
    float preset_value = 10.0;
    float weight=0.0;

    // ��ѹ����������ֵС��Ԥ��ֵʱ��������ת
    while ( (( weight = HX711_GetForce()) < preset_value)| (HAL_GPIO_ReadPin(BUTTON2_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET))
    {

            LCD1602_SetCursorPosition(0, 10);
            LCD1602_PrintString("Value:");
            LCD1602_SetCursorPosition(1, 10);
            LCD1602_PrintFloat(weight, 2);
            step_index = (step_index + 1) % 8;
            //  HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A | MOTOR_PIN_B | MOTOR_PIN_C | MOTOR_PIN_D, sequence[step_index]);
            uint8_t binary_sequence = sequence[step_index];
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A, (binary_sequence & 0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_B, (binary_sequence & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_C, (binary_sequence & 0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_D, (binary_sequence & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            delay_us(1);

    }
}

void Rotate_StepperMotor_CounterClockwise(  )
{
    // ��ת˳��Ϊ��A -> DA -> D -> CD -> C -> BC -> B -> AB
    uint8_t sequence[8] = {0x01, 0x09, 0x08, 0x0C, 0x04, 0x06, 0x02, 0x03};
    //0x01 = 0b0001
    //0x09 = 0b1001
    //0x08 = 0b1000
    //0x0C = 0b1100
    //0x04 = 0b0100
    //0x06 = 0b0110
    //0x02 = 0b0010
    //0x03 = 0b0011
    static uint8_t step_index = 0;

    // Ԥ��ѹ��������ֵ���ɸ�����Ҫ����
    float weight=0.0;
    float preset_value = 10.0;

    // ��ѹ����������ֵС��Ԥ��ֵʱ��������ת
    while ( ( (weight = HX711_GetForce()) < preset_value)| (HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET))
    {
        while (HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN) == GPIO_PIN_SET);
        {LCD1602_SetCursorPosition(0, 10);
        LCD1602_PrintString("Value:");
        LCD1602_SetCursorPosition(1, 10);
        LCD1602_PrintFloat(weight, 2);
        step_index = (step_index + 1) % 8;
//        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A | MOTOR_PIN_B | MOTOR_PIN_C | MOTOR_PIN_D, sequence[step_index]);
        uint8_t binary_sequence = sequence[step_index];
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A, (binary_sequence & 0b0001) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);

        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_B, (binary_sequence & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);

        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_C, (binary_sequence & 0b0100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(5);

        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_D, (binary_sequence & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  //      HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_A | MOTOR_PIN_B | MOTOR_PIN_C | MOTOR_PIN_D, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_B,  GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_C,  GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_D,  GPIO_PIN_RESET);
       // delay_us (1000);
        HAL_Delay(5);}
    }
}