#ifndef __SY_TFT240LCD_H__
#define __SY_TFT240LCD_H__

#include "stm32f4xx_hal.h"
#define  TFT_DC_GPIO_Port GPIOA
#define  TFT_DC_Pin GPIO_PIN_5

void SY_TFT240LCD_Init(void);
void SY_TFT240LCD_Clear(uint16_t color);
void SY_TFT240LCD_PutPixel(uint16_t x, uint16_t y, uint16_t color);
void SY_TFT240LCD_PrintFloat(uint16_t x, uint16_t y, float number, uint8_t digits, uint16_t textColor, uint16_t bgColor);

#endif // __SY_TFT240LCD_H__