/*
 * 指示灯配置
 */
void LED_Config()
{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);    
		
		led_off;
}

/*
 * 指示灯闪烁
 */ 
void LED_Flash()
{	
		int i = 100;
		while(i--)
		{
				led_on;
				delay_ms(100);
				led_off;
				delay_ms(100);
		}
}

/*
 * 置零开关
 */
void SwitchInit()
{
		GPIO_InitTypeDef GPIO_InitStructure;
			
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
}
