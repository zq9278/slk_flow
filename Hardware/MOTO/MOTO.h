#ifndef __L6219_MOTOR_H__
#define __L6219_MOTOR_H__

#include "stm32f4xx_hal.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Motor GPIO Pins -----------------------------------------------------------*/
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
void Rotate_StepperMotor_Clockwise( );
void Rotate_StepperMotor_CounterClockwise( );


#endif // __L6219_MOTOR_H__