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

// 姿态角
float Pitch,Roll,Yaw;

// 四元数
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

struct rx_s 
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s 
{
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};

static struct hal_s hal = {0};

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