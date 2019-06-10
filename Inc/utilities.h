#ifndef __UTILITIES_H
#define __UTILITIES_H

#include "stm32f0xx_hal.h"
#include "stdbool.h"
#include "flash.h"

#define GET_VERSION							0xF0
#define GET_ID									0xF1
#define SET_ID									0xF2
#define GET_FILTER_ID						0xF3
#define SET_FILTER_ID						0xF4
#define GET_TIMEOUT							0xF5
#define SET_TIMEOUT							0xF6
#define GET_BAUDRATE						0xF7
#define SET_BAUDRATE						0xF8
#define USB_MESSAGE  						true
#define CAN_MESSAGE  						!USB_MESSAGE

#define CAN_BAUDRATE_50  				0x28				//40
#define CAN_BAUDRATE_100  			0x14				//20
#define CAN_BAUDRATE_200  			0x0A				//10
#define CAN_BAUDRATE_400  			0x05				//5

#define ADDR_FLASH_PAGE_31    	((uint32_t)0x08007800) /* Base @ of Page 30, 1 Kbyte */
#define DATA_START_ADDRESS 		 	ADDR_FLASH_PAGE_31	//Page 31
#define FLASH_DATA_LENGTH				7

#define DEFAULT_FILTER_ID_MODE 	false
#define DEFAULT_TIMEOUT 		 		0x0A
#define DEFAULT_BAUDRATE				CAN_BAUDRATE_50

typedef struct _UsbMessage {
    uint8_t data[8];
		uint8_t length;
		bool type;
} UsbMessage;

uint8_t Crc8_Calc(uint8_t *bytes, uint32_t len);
void Write_Flash(uint16_t msgId, uint16_t filterId, bool filterIdMode, uint8_t timeout, uint8_t buadrate);
void Read_Flash(uint16_t *msgId, uint16_t *filterId, bool *filterIdMode, uint8_t *timeout, uint8_t *buadrate);
#endif /* __UTILITIES_H */
