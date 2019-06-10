#include <flash.h>

void deleteBuffer(char* data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		data[i] = 0;
	}
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
  while((FLASH->SR&FLASH_SR_BSY));
  FLASH->CR |= FLASH_CR_PER; //Page Erase Set
  FLASH->AR = addr; //Page Address
  FLASH->CR |= FLASH_CR_STRT; //Start Page Erase
  while((FLASH->SR&FLASH_SR_BSY));
	FLASH->CR &= ~FLASH_SR_BSY;
  FLASH->CR &= ~FLASH_CR_PER; //Page Erase Clear
}

void Flash_Write_Int(uint32_t addr, int data)
{
	Flash_Unlock();
	FLASH->CR |= FLASH_CR_PG;				/*!< Programming */
	while((FLASH->SR&FLASH_SR_BSY));
	*(__IO uint16_t*)addr = data;
	while((FLASH->SR&FLASH_SR_BSY));
	FLASH->CR &= ~FLASH_CR_PG;
	Flash_Lock();
}

uint16_t Flash_Read_Int(uint32_t addr)
{
	uint16_t* val = (uint16_t *)addr;
	return *val;
}

void Flash_Write_Char(char* dataIn, uint8_t len, uint32_t addr)
{
	Flash_Unlock();
	Flash_Erase(addr);
	int i;
  FLASH->CR |= FLASH_CR_PG;
	int var = 0;
  for(i=0; i<len; i+=1)
  {
    while((FLASH->SR&FLASH_SR_BSY));
		var = (int)dataIn[i];
    *(__IO uint16_t*)(addr + i*2) = var;
  }
	while((FLASH->SR&FLASH_SR_BSY)){};
  FLASH->CR &= ~FLASH_CR_PG;
  FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_Read_Char(char* dataOut, uint8_t len, uint32_t addr)
{
	deleteBuffer(dataOut,len);
	for(int i = 0; i < len; i++)
	{
		dataOut[i] = Flash_Read_Int(addr + (uint32_t)(i*2));
	}
}

void Flash_ProgramPage(char* dataIn, uint8_t len, uint32_t addr)
{
	//FLASH_Unlock
	Flash_Unlock();
	//Flash_Erase Page
	Flash_Erase(addr);
	//FLASH_Program HalfWord
	Flash_Write_Char(dataIn,len,addr);
}
