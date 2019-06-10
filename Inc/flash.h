#include "stm32f0xx_hal.h"
#include "stdint.h"
#include "string.h"

void deleteBuffer(char* data, uint8_t len);
void 	Flash_Lock(void);
void 	Flash_Unlock(void);
void 	Flash_Erase(uint32_t addr);
void 	Flash_Write_Int(uint32_t addr, int data);
uint16_t Flash_Read_Int(uint32_t addr);
void 	Flash_Write_Char(char* dataIn, uint8_t len, uint32_t addr);
void 	Flash_Read_Char(char* dataOut, uint8_t len, uint32_t addr);
void 	Flash_ProgramPage(char* dataIn, uint8_t len, uint32_t addr);
