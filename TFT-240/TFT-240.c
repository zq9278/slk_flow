//
// Created by 15026 on 2023/7/5.
//

#include "TFT-240.h"
#include "main.h"
#include <stdio.h>

#define ILI9341_RESET 0x01
#define ILI9341_SLEEP_OUT 0x11
#define ILI9341_DISPLAY_OFF 0x28
#define ILI9341_DISPLAY_ON 0x29
#define ILI9341_COLUMN_ADDR 0x2A
#define ILI9341_PAGE_ADDR 0x2B
#define ILI9341_GRAM 0x2C
#define ILI9341_MAC 0x36
#define ILI9341_PIXEL_FORMAT 0x3A
#define ILI9341_WDB 0x51
#define ILI9341_WCD 0x53
#define ILI9341_RGB_INTERFACE 0xB0
#define ILI9341_FRC 0xB1
#define ILI9341_BPC 0xB5
#define ILI9341_DFC 0xB6
#define ILI9341_Entry_Mode_Set 0xB7
#define ILI9341_POWER1 0xC0
#define ILI9341_POWER2 0xC1
#define ILI9341_VCOM1 0xC5
#define ILI9341_VCOM2 0xC7
#define ILI9341_POWERA 0xCB
#define ILI9341_POWERB 0xCF
#define ILI9341_PGAMMA 0xE0
#define ILI9341_NGAMMA 0xE1
#define ILI9341_DTCA 0xE8
#define ILI9341_DTCB 0xEA
#define ILI9341_POWER_SEQ 0xED
#define ILI9341_3GAMMA_EN 0xF2
#define ILI9341_INTERFACE 0xF6
#define ILI9341_PRC 0xF7

// 向ILI9341发送命令
void ILI9341_SendCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
}

// 向ILI9341发送数据
void ILI9341_SendData(uint8_t data) {
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
}

void ILI9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ILI9341_SendCommand(ILI9341_COLUMN_ADDR);
    ILI9341_SendData(x0 >> 8);
    ILI9341_SendData(x0 & 0xFF);
    ILI9341_SendData(x1 >> 8);
    ILI9341_SendData(x1 & 0xFF);

    ILI9341_SendCommand(ILI9341_PAGE_ADDR);
    ILI9341_SendData(y0 >> 8);
    ILI9341_SendData(y0 & 0xFF);
    ILI9341_SendData(y1 >> 8);
    ILI9341_SendData(y1 & 0xFF);

    ILI9341_SendCommand(ILI9341_GRAM);
}

void SY_TFT240LCD_Init(void) {
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);

    ILI9341_SendCommand(ILI9341_RESET);
    HAL_Delay(5);

    ILI9341_SendCommand(ILI9341_DISPLAY_OFF);

    // 在这里添加初始化代码

    ILI9341_SendCommand(ILI9341_DISPLAY_ON);

    SY_TFT240LCD_Clear(0xFFFF); // 设置背景为白色
}

void SY_TFT240LCD_Clear(uint16_t color) {
    ILI9341_SetAddrWindow(0, 0, 240 - 1, 320 - 1);

    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;

    for (uint32_t i = 0; i < 240 * 320; i++) {
        HAL_SPI_Transmit(&hspi1, &color_high, 1, 10);
        HAL_SPI_Transmit(&hspi1, &color_low, 1, 10);
    }
}

void SY_TFT240LCD_PutPixel(uint16_t x, uint16_t y, uint16_t color) {
    ILI9341_SetAddrWindow(x, y, x, y);

    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;

    HAL_SPI_Transmit(&hspi1, &color_high, 1, 10);
    HAL_SPI_Transmit(&hspi1, &color_low, 1, 10);
}

void SY_TFT240LCD_PrintFloat(uint16_t x, uint16_t y, float number, uint8_t digits, uint16_t textColor, uint16_t bgColor) {
    // 在此处添加代码以实现浮点数打印功能
    // 您可以使用现有的库，例如u8g2或自己编写一个简单的字符渲染函数
}