#include "eeprom.h"

void delay_us(u32 n)
{
		u8 j;
		while(n--)
		for(j=0;j<10;j++);
}

void delay_ms(u32 n)
{
		while(n--)
		delay_us(1000);
}

// ����EEPROM�Ĵ�СΪ1K
u16 STMFLASH_BUFF[STMFLASH_SIZE];
