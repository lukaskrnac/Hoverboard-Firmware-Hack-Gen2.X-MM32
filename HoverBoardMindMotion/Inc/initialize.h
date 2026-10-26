#ifndef INITIALIZE_H
#define INITIALIZE_H

#include "mcu.h"

void io_init(void);
void TIM1_init(u16 arr, u16 psc);
void BLDC_init(void);
void NVIC_Configure(u8 ch, u8 pri);
void UARTX_Init(u32 baudrate,uint8_t uart);
void adc_Init(void);
uint8_t HALLTIM_Init(u32 arr, u16 psc);
void HALL_Init(void);
void Iwdg_Init(u16 IWDG_Prescaler, u16 Reload);
uint8_t UART_GPIO_Init(void);

#endif
