#ifndef _LED_H_
#define _LED_H_

// LED״̬
#define  led_on    GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define  led_off   GPIO_SetBits(GPIOB, GPIO_Pin_12)

void LED_Config();

void LED_Flash();

void SwitchInit();

#endif
