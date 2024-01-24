#include <flash.h>

uint16_t EEPROM_buffer[512];
uint16_t eeprom_size=512;

void EEPROM_init(uint16_t size)
{
	eeprom_size=size;
	for(int i=0;i<eeprom_size;i++)EEPROM_buffer[i]=Flash_read(i);
}
void Flash_Lock()
{
	HAL_FLASH_Lock();
}

void Flash_Unlock()
{
	HAL_FLASH_Unlock();
}

void Flash_Erase(uint32_t addr)
{
Flash_Unlock();
  while((FLASH->SR&FLASH_SR_BSY));
  FLASH->CR |= FLASH_CR_PER; //Page Erase Set
  FLASH->AR = addr; //Page Address
  FLASH->CR |= FLASH_CR_STRT; //Start Page Erase
  while((FLASH->SR&FLASH_SR_BSY));
	FLASH->CR &= ~FLASH_SR_BSY;
  FLASH->CR &= ~FLASH_CR_PER; //Page Erase Clear
Flash_Lock();
}
void EEP_write(uint16_t add, uint16_t data)
{
	EEPROM_buffer[add]=data;
}

uint16_t EEP_read(uint16_t add)
{
	return EEPROM_buffer[add];
}

uint16_t Flash_read(uint16_t add)
{
	int32_t addr;
	addr = DATA_START_ADDRESS + (add*2);
	uint16_t* val = (uint16_t *)addr;
	return *val;
}
void EEP_commit(void)
{
	uint32_t addr;
	Flash_Erase(DATA_START_ADDRESS);
	Flash_Unlock();
	for(int i=0;i<eeprom_size;i++)
	{
		addr = DATA_START_ADDRESS + (i*2);
		FLASH->CR |= FLASH_CR_PG;				/*!< Programming */
		while((FLASH->SR&FLASH_SR_BSY));
		*(__IO uint16_t*)addr = EEPROM_buffer[i];
		while((FLASH->SR&FLASH_SR_BSY));
		FLASH->CR &= ~FLASH_CR_PG;
	}
	Flash_Lock();
}
