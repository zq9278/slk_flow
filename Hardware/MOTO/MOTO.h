#ifndef __TMC5160_H__
#define __TMC5160_H__
#include "stm32f4xx_hal.h"
#include "main.h"


#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA

//���尴��gpio
#define KEY1_Pin GPIO_PIN_12
#define KEY2_Pin GPIO_PIN_11
#define KEY_GPIO_Port GPIOB
//����dir��en�Լ�step
// �� main.h �ļ���������º궨��


void sendData(unsigned long address,long long datagram);
void SPI_SendByte(char data);
char SPI_ReceiveByte(void);
unsigned long ReadData(long address);
void tmc5160_init(void);
#endif // __TMC5160_H__