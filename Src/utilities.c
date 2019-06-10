#include "utilities.h"

uint8_t Crc8_Calc(uint8_t *data, uint32_t len)
{
  uint32_t crc = 0;
  int i, j;
  for (j = len; j; j--, data++) {
    crc ^= (*data << 8);
    for(i = 8; i; i--) {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}

void Write_Flash(uint16_t msgId, uint16_t filterId, bool filterIdMode, uint8_t timeout, uint8_t baudrate)
{
	char flashData[FLASH_DATA_LENGTH]= {0};
	flashData[0] = msgId >> 8;
	flashData[1] = msgId & 0x00FF;
	flashData[2] = filterId >> 8;
	flashData[3] = filterId & 0x00FF;
	flashData[4] = filterIdMode;
	flashData[5] = timeout;
	flashData[6] = baudrate;
	Flash_Write_Char(flashData,FLASH_DATA_LENGTH,DATA_START_ADDRESS);
}

void Read_Flash(uint16_t *msgId, uint16_t *filterId, bool *filterIdMode, uint8_t *timeout, uint8_t *baudrate)
{
	char flashData[FLASH_DATA_LENGTH]= {0};
	Flash_Read_Char(flashData,FLASH_DATA_LENGTH,DATA_START_ADDRESS);
	*msgId = ((uint16_t)flashData[0] << 8) | (uint16_t)flashData[1];
	*filterId = ((uint16_t)flashData[2] << 8) | (uint16_t)flashData[3];
	if((uint8_t)flashData[4] == 0xFF)
	{
		*filterIdMode = DEFAULT_FILTER_ID_MODE;
	}
	else
	{
		*filterIdMode = flashData[4];
	}
	
	*timeout = flashData[5];
	
	if((uint8_t)flashData[6] == 0xFF)
	{
		*baudrate = DEFAULT_BAUDRATE;
	}
	else
	{
		*baudrate = flashData[6];
	}
}

