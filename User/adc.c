#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"

void ADC1_Init()
{	
		DMA_InitTypeDef DMA_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;

		// GPIO≈‰÷√
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
		GPIO_Init(GPIOA,&GPIO_InitStructure);

		// DMA≈‰÷√
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_PeripheralBaseAddr=ADC1_DR_Address;
		DMA_InitStructure.DMA_MemoryBaseAddr=(u32)&ADC_ConvertedValue;
		DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize=1;
		DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Disable;
		DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority=DMA_Priority_High;
		DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
		DMA_Init(DMA1_Channel1,&DMA_InitStructure);
		DMA_Cmd(DMA1_Channel1,ENABLE);
}	