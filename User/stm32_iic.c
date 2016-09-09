#include "stm32_iic.h"

#define  SCL_H         GPIOB->BSRR = GPIO_Pin_6  
#define  SCL_L         GPIOB->BRR  = GPIO_Pin_6  
#define  SDA_H         GPIOB->BSRR = GPIO_Pin_7  
#define  SDA_L         GPIOB->BRR  = GPIO_Pin_7 

#define  SCL_read      GPIOB->IDR  & GPIO_Pin_6  
#define  SDA_read      GPIOB->IDR  & GPIO_Pin_7  

// IIC专用延时函数
static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)

    i--;
}

//IIC接口初始化
void I2CInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}