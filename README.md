# USB-CAN
Firmware USB-CAN converter for stm32f0

This project was built by keilC <br/>

CAN receive interrupt located in "main.c" file <br/>
USB receive interrupt located in "usbd_cdc_if.c" file <br/>

The setting parameters are writing to flash after each adjustment <br/>
This parameters located at address 0x08007800 (Page 31) of stm32f0 flash <br/>

