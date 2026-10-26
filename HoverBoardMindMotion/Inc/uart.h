#ifndef UART_H
#define UART_H

#include "mcu.h"

void UART_Send_Group(u8* buf, u16 len);
void UART_SendString(char* str);

#endif
