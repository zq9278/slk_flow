


#include "stm32f4xx_hal.h"

#ifndef __LCD1602_H
#define __LCD1602_H

#define RS_Pin GPIO_PIN_0
#define RS_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_1
#define RW_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_7 // 将 EN 引脚从 PB2 更改为 PB7
#define EN_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_3
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_5
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_6
#define D7_GPIO_Port GPIOB


// 函数原型声明
void LCD1602_Init(void);
void LCD1602_PrintFloat(float number, uint8_t digits);
void LCD1602_PrintInt(int32_t number);
void LCD1602_PrintString(const char *str);
void LCD1602_SetCursorPosition(uint8_t row, uint8_t col);

#endif // __LCD1602_H
