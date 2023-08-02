#include <stdbool.h>
#include "MOTO.h"
#include "main.h"
#include "stm32f4xx_hal.h"


// 定义SPI句柄
extern SPI_HandleTypeDef hspi1;
unsigned char	cmd[8];
unsigned char	ReceiveData;

void tmc5160_init(void){
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);

//TMC5160 SET




//    sendData(0xEC,0x000100C3); 				//PAGE43:CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadcycle)
//    sendData(0x90,0x00061F0A); 				//PAGE33:IHOLD_IRUN: IHOLD=10, IRUN=31 (max.current), IHOLDDELAY=6
//    sendData(0x91,0x0000000A);				//PAGE33:TPOWERDOWN=10:电机静止到电流减小之间的延时
//    sendData(0x80,0x00000004);				//PAGE27:EN_PWM_MODE=1，使能
//    sendData(0xF0,0x000C0000);				//PAGE43:PWMCONF
//    sendData(0x93,0x000001F4);				//PAGE33:TPWM_THRS=500,对应切换速度35000=ca.30RPM






//	sendData(0xA4,1000);     					//PAGE35:A1=1000 第一阶段加速度
//	sendData(0xA5,50000);     				//PAGE35:V1=50000加速度阀值速度V1
//	sendData(0xA6,500);     					//PAGE35:AMAX=500大于V1的加速度
//	sendData(0xA7,200000);     				//PAGE35:VMAX=200000
//	sendData(0xA8,700);								//PAGE35:DMAX=700大于V1的减速度
//	sendData(0xAA,1400);     					//PAGE35:D1=1400小于V1的减速度
//	sendData(0xAB,10);     						//PAGE35:VSTOP=10停止速度，接近于0
//	sendData(0xA0,0x00000001); 				//PAGE35:RAMPMODE=1速度模式到正VMAX使用AMAX加速度








//    sendData(0xA4,6000);     					//PAGE35:A1=6000 第一阶段加速度
//    sendData(0xA5,150000);     				//PAGE35:V1=150000加速度阀值速度V1
//    sendData(0xA6,3000);     					//PAGE35:AMAX=3000大于V1的加速度
//    sendData(0xA7,600000);     				//PAGE35:VMAX=600000
//    sendData(0xA8,4200);							//PAGE35:DMAX=4200大于V1的减速度
//    sendData(0xAA,8400);     					//PAGE35:D1=8400小于V1的减速度
//    sendData(0xAB,10);     						//PAGE35:VSTOP=10停止速度，接近于0



    sendData(0x24,0xA4000003E8);
    sendData(0x25,0xA50000C350);
    sendData(0x26, 0xA6000001F4);
    sendData(0x27,0xA700030D40);
    sendData(0x28,0xA8000002BC);
    sendData(0x2A,0xAA00000578);
    sendData(0x2B,0xAB0000000A);
    sendData(0x20,0xA000000000);
   // sendData(,);
    //sendData(,);
}

void SPI_SendByte(char data)
{
    cmd[0] = data;
    if(HAL_SPI_Transmit(&hspi1,cmd,1,1000) == HAL_OK)
    {
        if(HAL_SPI_Receive(&hspi1,cmd,1,1000) == HAL_OK)
        {
            ReceiveData	=cmd[0];
        }
        else;
    }
    else;
}

char SPI_ReceiveByte(void)
{
    cmd[0] = 0;
    if(HAL_SPI_Transmit(&hspi1,cmd,1,1000) == HAL_OK)
    {
        if(HAL_SPI_Receive(&hspi1,cmd,1,1000) == HAL_OK)
        {
            //return cmd[0];
        }
        else;
    }
    else;
    return cmd[0];
}

//TMC5160 takes 40 bit data: 8 address and 32 data
void sendData(unsigned long address,long long datagram)
{
    unsigned char i;
    cmd[0]=address;
    cmd[1]=(datagram >> 24) & 0xff;
    cmd[2]=(datagram >> 16) & 0xff;
    cmd[3]=(datagram >> 8) & 0xff;
    cmd[4]=datagram & 0xff;

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); 		//SPI_CS片选拉低
//	SPI_SendByte(address);
//	SPI_SendByte((datagram >> 24) & 0xff);
//	SPI_SendByte((datagram >> 16) & 0xff);
//	SPI_SendByte((datagram >> 8) & 0xff);
//	SPI_SendByte(datagram & 0xff);
    for(i=0;i<5;i++)
    {
        if(HAL_SPI_Transmit(&hspi1,&cmd[i],1,100) == HAL_OK)
        {
        }
        else;
    }
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);  	//SPI_CS片选拉高

}

unsigned long ReadData(long address)
{
    char data[4] = {0, 0, 0, 0};
    unsigned long datagram = 0;

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); 	//SPI_CS片选拉低

    SPI_SendByte(address);
    data[0] = SPI_ReceiveByte();//SPI_ReceiveByte((datagram >> 24) & 0xff);
    data[1] = SPI_ReceiveByte();//SPI_ReceiveByte((datagram >> 16) & 0xff);
    data[2] = SPI_ReceiveByte();//SPI_ReceiveByte((datagram >> 8) & 0xff);
    data[3] = SPI_ReceiveByte();//SPI_ReceiveByte(datagram & 0xff);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); 	//SPI_CS片选拉高

    datagram = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    return datagram;
}