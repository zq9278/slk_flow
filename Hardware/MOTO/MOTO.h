#ifndef __L6219_MOTOR_H__
#define __L6219_MOTOR_H__

#include "stm32f4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Motor GPIO Pins -----------------------------------------------------------*/

#define IN1_Pin GPIO_PIN_9
#define IN2_Pin GPIO_PIN_10
#define IN3_Pin GPIO_PIN_11
#define IN4_Pin GPIO_PIN_12
#define PHASEA_Pin GPIO_PIN_13
#define PHASEB_Pin GPIO_PIN_14
#define L6219_GPIO_Port GPIOD

// 定义按键引脚
#define KEY1_Pin GPIO_PIN_12
#define KEY2_Pin GPIO_PIN_11
#define KEY_GPIO_Port GPIOB



/* Function prototypes -------------------------------------------------------*/
void L6219_Init(void);
void Step_CounterClockwise(void);
void Step_Clockwise(void);
void L6219_Stop(void);

#endif // __L6219_MOTOR_H__