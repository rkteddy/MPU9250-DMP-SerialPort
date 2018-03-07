#ifndef __STM32_I2C_H
#define __STM32_I2C_H

#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
#define true 1
#define false 0 
#define bool  uint8_t
#define TRUE  0
#define FALSE -1

// 0��ʾд
#define	I2C_Direction_Transmitter   0
//����ʾ��
#define	I2C_Direction_Receiver      1	 

bool I2CWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool I2CWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool I2CRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void I2CInit(void);

uint16_t i2cGetErrorCounter(void);
static void i2cUnstick(void);

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif


