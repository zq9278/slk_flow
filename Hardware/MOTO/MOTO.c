#include "stm32f4xx_hal.h"
#include "main.h"
#include "MOTO.h"

// L6219�������Ŷ���


// �������Ŷ���

// ��ʼ�������������
void L6219_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ����GPIOAʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // ���ò��������������
    GPIO_InitStruct.Pin = IN1_Pin | IN2_Pin | IN3_Pin | IN4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ���ð�����������
    GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // �����ⲿ�ж�
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


// ����1�ⲿ�жϴ�����
void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
}

// ����2�ⲿ�жϴ�����
void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
}

// �ⲿ�жϻص�����
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY1_Pin) {
        // ����1���£���ʱ����ת
        L6219_CCW_Step();
    } else if (GPIO_Pin == KEY2_Pin) {
        // ����2���£�˳ʱ����ת
        L6219_CW_Step();
    }
}



// �ӳٺ���


// ȫ����������
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

// ���г���
const uint8_t sequence_length = sizeof(step_sequence) / sizeof(step_sequence[0]);

// ��ǰ����
uint8_t current_step = 0;

// ���ò����������λ
void set_phase(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ��ʱ����ת�������
void L6219_CCW_Step(void) {
    current_step = (current_step + 1) % sequence_length;
    set_phase(step_sequence[current_step][0], step_sequence[current_step][1],
              step_sequence[current_step][2], step_sequence[current_step][3]);
    delay_us(1000); // �ӳ�1ms���ɸ���ʵ���������
}

// ˳ʱ����ת�������
void L6219_CW_Step(void) {
    current_step = (current_step - 1 + sequence_length) % sequence_length;
    set_phase(step_sequence[current_step][0], step_sequence[current_step][1],
              step_sequence[current_step][2], step_sequence[current_step][3]);
    delay_us(1000); // �ӳ�1ms���ɸ���ʵ���������
}