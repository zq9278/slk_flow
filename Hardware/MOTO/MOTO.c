#include "stm32f4xx_hal.h"
#include "main.h"
#include "MOTO.h"

// L6219驱动引脚定义


// 按键引脚定义

// 初始化步进电机引脚
void L6219_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 开启GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置步进电机驱动引脚
    GPIO_InitStruct.Pin = IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置按键输入引脚
    GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置外部中断
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


// 按键1外部中断处理函数
void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
}

// 按键2外部中断处理函数
void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
}

// 外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY1_Pin) {
        // 按键1按下，逆时针旋转
        L6219_CCW_Step();
    } else if (GPIO_Pin == KEY2_Pin) {
        // 按键2按下，顺时针旋转
        L6219_CW_Step();
    }
}



// 延迟函数


// 全步驱动序列
uint8_t step_sequence[][4] = {
    {1, 0, 0, 1},
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1}
};

// 序列长度
const uint8_t sequence_length = sizeof(step_sequence) / sizeof(step_sequence[0]);

// 当前步骤
uint8_t current_step = 0;

// 设置步进电机的相位
void set_phase(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 逆时针旋转步进电机
void L6219_CCW_Step(void) {
    current_step = (current_step + 1) % sequence_length;
    set_phase(step_sequence[current_step][0], step_sequence[current_step][1],
              step_sequence[current_step][2], step_sequence[current_step][3]);
    delay_us(1000); // 延迟1ms，可根据实际需求调整
}

// 顺时针旋转步进电机
void L6219_CW_Step(void) {
    current_step = (current_step - 1 + sequence_length) % sequence_length;
    set_phase(step_sequence[current_step][0], step_sequence[current_step][1],
              step_sequence[current_step][2], step_sequence[current_step][3]);
    delay_us(1000); // 延迟1ms，可根据实际需求调整
}