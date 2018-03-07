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

// ����EEPROM�Ĵ�СΪ1K
u16 STMFLASH_BUFF[STMFLASH_SIZE];

void STMFLASH_Write(u16 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
		u16 j;
		// �����������������
		STMFLASH_Read_Backup();
	
		// ����
		FLASH_Unlock();
	
		// ������255ҳ
		FLASH_ErasePage(FLASH_PAGE255_ADDR);
	
		// ������д��������ȥ
		for(j=0;j<NumToWrite;j++)
		{
				STMFLASH_BUFF[j+WriteAddr] = *(pBuffer+j);
		}	
		
		// ������д�ص���255ҳ��
		STMFLASH_Write_NoErase();
		
		// ����	
		FLASH_Lock();
}

/*
 * ��ȡ����Ϊ�ֽ�����
 * ��EEEPROM��1K�ֽڶ�����
 */
void STMFLASH_Read_Backup(void)
{
		u16 t;
		for(t = 0; t < STMFLASH_SIZE; t++) 
		{
				*(STMFLASH_BUFF+t) = *(u16*)(FLASH_PAGE255_ADDR+t*2);                
		}
}
