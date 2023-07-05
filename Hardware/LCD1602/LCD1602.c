//
// Created by 15026 on 2023/7/4.
//

#include "LCD1602.h"
#include "main.h"
#include <stdio.h>


// ˽�к���ԭ������
void LCD1602_Send4Bits(uint8_t data);
void LCD1602_SendCommand(uint8_t cmd);
void LCD1602_SendData(uint8_t data);

// ��ʼ��LCD1602
void LCD1602_Init(void) {
    // ����GPIOΪ���ģʽ
    // RS����
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS_GPIO_Port, &GPIO_InitStruct);

    // RW����
    GPIO_InitStruct.Pin = RW_Pin;
    HAL_GPIO_Init(RW_GPIO_Port, &GPIO_InitStruct);

    // EN����
    GPIO_InitStruct.Pin = EN_Pin;
    HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

    // D4-D7��������
    GPIO_InitStruct.Pin = D4_Pin | D5_Pin | D6_Pin | D7_Pin;
    HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

    // ��ʼ��LCD1602
    HAL_Delay(40); // �ȴ�LCD��Դ������������ѹ
    LCD1602_SendCommand(0x33); // ��ʼ��ָ��
    LCD1602_SendCommand(0x32); // ����4λ����ģʽ
    LCD1602_SendCommand(0x28); // ���ýӿڳ��Ⱥ������С��4λ��2�У�5x8����
    LCD1602_SendCommand(0x0C); // ��ʾ�������رգ�����˸
    LCD1602_SendCommand(0x06); // ��������ģʽ���Զ����������ƶ���ʾ��
    LCD1602_SendCommand(0x01); // �����ʾ
    HAL_Delay(2); // �ȴ�����������
}

// ��ӡ������
void LCD1602_PrintFloat(float number, uint8_t digits) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.*f", digits, number);
    LCD1602_PrintString(buffer);
}

// ��ӡ����
void LCD1602_PrintInt(int32_t number) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%ld", number);
    LCD1602_PrintString(buffer);
}

// ��ӡ�ַ���
void LCD1602_PrintString(const char *str) {
    for (; *str != '\0'; str++) {
        LCD1602_SendData((uint8_t)*str);
    }
}

// ˽�к���ʵ��
void LCD1602_Send4Bits(uint8_t data) {
    // �����������ŵ�״̬
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // ����ʹ������
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    delay_us(1); // �ȴ�����450ns
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
    delay_us(50); // �ȴ�����37us
}

void LCD1602_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET); // ����RSΪ0����ʾָ��ģʽ
    HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET); // ����RWΪ0����ʾдģʽ
    LCD1602_Send4Bits(cmd >> 4); // ���͸�4λ
    LCD1602_Send4Bits(cmd); // ���͵�4λ
}

void LCD1602_SendData(uint8_t data) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET); // ����RSΪ1����ʾ����ģʽ
    HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET); // ����RWΪ0����ʾдģʽ
    LCD1602_Send4Bits(data >> 4); // ���͸�4λ
    LCD1602_Send4Bits(data); // ���͵�4λ
}
void LCD1602_SetCursorPosition(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? col : (0x40 + col);
    LCD1602_SendCommand(0x80 | address);
}
