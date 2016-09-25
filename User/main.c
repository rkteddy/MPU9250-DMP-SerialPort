#include <stdio.h>
#include "stm32f10x.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"
#include "eeprom.h"
#include "adc.h"
#include "led.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

int main(void)
{  
    u16 count=0;  
	
	  // 串口初始化
    USART_Config(); 
	
		// LED初始化
    LED_Config();
	
    // IIC总线初始化
    I2CInit();

    // ADC初始化
		ADC1_Init();

    // 系统延时
    delay_ms(10);
}

void USART1_IRQHandler()
{
		uint8_t ch;

		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{     
				ch = USART_ReceiveData(USART1);
				printf( "%c", ch ); 
		} 
}

void DMA1_Channel1_IRQHandler()
{
		if(DMA_GetITStatus(DMA1_IT_TC1))
		{
				TIM_Cmd(TIM3,DISABLE);  
				DMA_ClearITPendingBit(DMA1_IT_GL1);
		}
}