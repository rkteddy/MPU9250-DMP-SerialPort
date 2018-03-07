#include "stm32_iic.h"

#define  SCL_H         GPIOB->BSRR = GPIO_Pin_6  
#define  SCL_L         GPIOB->BRR  = GPIO_Pin_6  
#define  SDA_H         GPIOB->BSRR = GPIO_Pin_7  
#define  SDA_L         GPIOB->BRR  = GPIO_Pin_7 

#define  SCL_read      GPIOB->IDR  & GPIO_Pin_6  
#define  SDA_read      GPIOB->IDR  & GPIO_Pin_7  

// IICר����ʱ����
static void I2C_delay(void)
{
    volatile int i = 7;
    while (i)

    i--;
}
