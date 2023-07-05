#ifndef __L6219_MOTOR_H__
#define __L6219_MOTOR_H__

#include "stm32f4xx_hal.h"

// L6219�������Ŷ���
#define IN1_Pin GPIO_PIN_6
#define IN1_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOC
#define IN3_Pin GPIO_PIN_8
#define IN3_GPIO_Port GPIOC
#define IN4_Pin GPIO_PIN_9
#define IN4_GPIO_Port GPIOC

// �������Ŷ���
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA

// ��ʼ��L6219�������
void L6219_Init(void);

// ��ʱ����ת�������
void L6219_CCW_Step(void);


// ˳ʱ����ת�������
void L6219_CW_Step(void);

#endif // __L6219_MOTOR_H__