#ifndef __L6219_MOTOR_H__
#define __L6219_MOTOR_H__

#include "stm32f4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Motor GPIO Pins -----------------------------------------------------------*/

#define IN1_Pin GPIO_PIN_9
#define IN2_Pin GPIO_PIN_13
#define IN3_Pin GPIO_PIN_14
#define IN4_Pin GPIO_PIN_12
#define PHA GPIO_PIN_6
#define PHB GPIO_PIN_15
#define L6219_GPIO_Port GPIOD

// 定义按键引脚
#define KEY1_Pin GPIO_PIN_12
#define KEY2_Pin GPIO_PIN_11
#define KEY_GPIO_Port GPIOB



#define MOTOR_PIN_A GPIO_PIN_9
#define MOTOR_PIN_B GPIO_PIN_13
#define MOTOR_PIN_C GPIO_PIN_14
#define MOTOR_PIN_D GPIO_PIN_12
#define MOTOR_GPIO_PORT GPIOD

/* Button GPIO Pins ----------------------------------------------------------*/
#define BUTTON1_PIN GPIO_PIN_12
#define BUTTON1_GPIO_PORT GPIOB
#define BUTTON2_PIN GPIO_PIN_11
#define BUTTON2_GPIO_PORT GPIOB

/* Function prototypes -------------------------------------------------------*/
void L6219_Init(void);
void Step_CounterClockwise(void);
void Step_Clockwise(void);
void L6219_Stop(void);

#endif // __L6219_MOTOR_H__