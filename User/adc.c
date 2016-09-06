#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

void ADC1_Init()
{	
		GPIO_InitTypeDef GPIO_InitStructure;

		// GPIO≈‰÷√
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
}	