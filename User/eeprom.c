#include "eeprom.h"

void delay_us(u32 n)
{
		u8 j;
		while(n--)
		for(j = 0; j < 10; j++);
}

void delay_ms(u32 n)
{
		while(n--)
		delay_us(1000);
}

// 设置EEPROM的大小为1K
u16 STMFLASH_BUFF[STMFLASH_SIZE];

void STMFLASH_Write(u16 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
		u16 j;
		// 读出备份区域的数据
		STMFLASH_Read_Backup();
	
		// 解锁
		FLASH_Unlock();
	
		// 擦除第255页
		FLASH_ErasePage(FLASH_PAGE255_ADDR);
	
		// 将数据写到数组中去
		for(j=0;j<NumToWrite;j++)
		{
				STMFLASH_BUFF[j+WriteAddr] = *(pBuffer+j);
		}	
		
		// 将数据写回到第255页中
		STMFLASH_Write_NoErase();
		
		// 上锁	
		FLASH_Lock();
}

/*
 * 读取数据为字节类型
 * 将EEEPROM的1K字节读出来
 */
void STMFLASH_Read_Backup(void)
{
		u16 t;
		for(t = 0; t < STMFLASH_SIZE; t++) 
		{
				*(STMFLASH_BUFF+t) = *(u16*)(FLASH_PAGE255_ADDR+t*2);                
		}
}
