//
// Created by 15026 on 2023/7/4.
//

#include "LCD1602.h"
#include "main.h"
#include <stdio.h>


// 私有函数原型声明
void LCD1602_Send4Bits(uint8_t data);
void LCD1602_SendCommand(uint8_t cmd);
void LCD1602_SendData(uint8_t data);

// 初始化LCD1602
void LCD1602_Init(void) {
    // 设置GPIO为输出模式
    // RS引脚
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS_GPIO_Port, &GPIO_InitStruct);

    // RW引脚
    GPIO_InitStruct.Pin = RW_Pin;
    HAL_GPIO_Init(RW_GPIO_Port, &GPIO_InitStruct);

    // EN引脚
    GPIO_InitStruct.Pin = EN_Pin;
    HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);

    // D4-D7数据引脚
    GPIO_InitStruct.Pin = D4_Pin | D5_Pin | D6_Pin | D7_Pin;
    HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

    // 初始化LCD1602
    HAL_Delay(40); // 等待LCD电源上升至工作电压
    LCD1602_SendCommand(0x33); // 初始化指令
    LCD1602_SendCommand(0x32); // 设置4位操作模式
    LCD1602_SendCommand(0x28); // 设置接口长度和字体大小（4位，2行，5x8点阵）
    LCD1602_SendCommand(0x0C); // 显示开，光标关闭，不闪烁
    LCD1602_SendCommand(0x06); // 设置输入模式（自动增量，不移动显示）
    LCD1602_SendCommand(0x01); // 清除显示
    HAL_Delay(2); // 等待清除操作完成
}

// 打印浮点数
void LCD1602_PrintFloat(float number, uint8_t digits) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.*f", digits, number);
    LCD1602_PrintString(buffer);
}

// 打印整数
void LCD1602_PrintInt(int32_t number) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%ld", number);
    LCD1602_PrintString(buffer);
}

// 打印字符串
void LCD1602_PrintString(const char *str) {
    for (; *str != '\0'; str++) {
        LCD1602_SendData((uint8_t)*str);
    }
}

// 私有函数实现
void LCD1602_Send4Bits(uint8_t data) {
    // 设置数据引脚的状态
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // 生成使能脉冲
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    delay_us(1); // 等待至少450ns
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
    delay_us(50); // 等待至少37us
}

void LCD1602_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET); // 设置RS为0，表示指令模式
    HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET); // 设置RW为0，表示写模式
    LCD1602_Send4Bits(cmd >> 4); // 发送高4位
    LCD1602_Send4Bits(cmd); // 发送低4位
}

void LCD1602_SendData(uint8_t data) {
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET); // 设置RS为1，表示数据模式
    HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET); // 设置RW为0，表示写模式
    LCD1602_Send4Bits(data >> 4); // 发送高4位
    LCD1602_Send4Bits(data); // 发送低4位
}
void LCD1602_SetCursorPosition(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? col : (0x40 + col);
    LCD1602_SendCommand(0x80 | address);
}
