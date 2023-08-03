#include "hX711.h"
#include "main.h"

/* HX711 初始化 */
unsigned long Weight_Maopi = 0;//毛皮重
char Weight_Err = 0;//重量异常标志

void HX711_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 配置数据引脚为输入 */
    GPIO_InitStruct.Pin = HX711_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(HX711_DATA_GPIO_Port, &GPIO_InitStruct);

    /* 配置时钟引脚为输出 */
    GPIO_InitStruct.Pin = HX711_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HX711_CLK_GPIO_Port, &GPIO_InitStruct);
    //__HAL_RCC_GPIOA_CLK_ENABLE();
    Get_Maopi();
    /* 时钟引脚置低 */
    HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_RESET);

}

/* 读取HX711数据 */
int32_t HX711_Read(void)
{
    int32_t data = 0;
    uint8_t i;
    /* 等待数据引脚为低 ,即A/D转换器准备好*/
    while (HAL_GPIO_ReadPin(HX711_DATA_GPIO_Port, HX711_DATA_Pin) == GPIO_PIN_SET)//当数据位是高电平时，一直进入循环，直到数据为变成低电平
    {;}
    /* 读取24位数据 */
    for (i = 0; i < 24; i++)
    {
        /* 时钟引脚置高 */
        HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_SET);
       data <<= 1;
        delay_us(1);
        /* 时钟引脚置低 */
        HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_RESET);
        /* 检查数据引脚的值 */
        if (HAL_GPIO_ReadPin(HX711_DATA_GPIO_Port, HX711_DATA_Pin) == GPIO_PIN_SET)
        {
            data++;
        }
        delay_us(1);
    }
    /* 时钟引脚置高 */
    HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_SET);
    delay_us(1);
    data=data^ 0x800000;
    /* 时钟引脚置低 */
    HAL_GPIO_WritePin(HX711_CLK_GPIO_Port, HX711_CLK_Pin, GPIO_PIN_RESET);
    delay_us(1);    /* 符号位扩展 */
    return data;
}
unsigned long Get_Maopi(void)
{
    unsigned long Weight_Maopi_temp;

    Weight_Maopi_temp = HX711_Read();
    HAL_Delay(500);
    Weight_Maopi = HX711_Read();

    if((Weight_Maopi*10)/HX711_SCALE_FACTOR != (Weight_Maopi_temp*10)/HX711_SCALE_FACTOR)
        return 	 Weight_Maopi;
    else
        return 0;
}
/* 获取力（牛顿） */
float HX711_GetForce(void)
{
    double Weight_Shiwu;
    unsigned int wei;
    Weight_Shiwu = HX711_Read();
    if(Weight_Shiwu>=Weight_Maopi)
    {
        Weight_Shiwu = Weight_Shiwu - Weight_Maopi;		//获取净重
        Weight_Err = 0;
    }
    else
    {
        Weight_Shiwu = Weight_Maopi - Weight_Shiwu;
        Weight_Err = 1;//出现负值了，可能是去皮异常
    }
    wei = ((Weight_Shiwu*10)/HX711_SCALE_FACTOR)+0.5f;
    if(wei==0)Weight_Err = 0;
    return wei;

    int32_t rawData = HX711_Read();
  //  float force = (float)rawData;
	  //float force = (float)rawData / HX711_SCALE_FACTOR;
	//float force = ((0-(float)rawData) / HX711_SCALE_FACTOR)/10+2.538;
	//float force = ((float)rawData )*10/ HX711_SCALE_FACTOR+3.60;

}
