#ifndef UART_H
#define UART_H
#include "mcu.h"

void SerialInit(void);                     //rx ring + tx DMA on the uart in use (uarten), after UARTX_Init()
int SerialGetByte(void);                   //next received byte, -1 if none
void SerialSend(const u8* buf, u16 len);   //DMA, waits for the previous frame; len <= 32

#endif
