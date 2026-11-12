//Serial transport for the RemoteUartBus. Receive: circular DMA into a ring buffer, no interrupt per
//byte; the main loop takes the bytes with SerialGetByte(). Transmit: DMA from a frame buffer.
//(Before: a 1 byte DMA buffer with an interrupt per byte and the frame parser in that interrupt, and
//replies sent byte by byte with busy waiting: fine at 19200, not at 115200+.)
//DMA channels: TX 4, RX 5 for either uart (UART1's requests remapped there, SYSCFG): 2 and 3 are the only
//channels of I2C1 (the IMU, i2c.c).
#include "hal_uart.h"
#include "pinout.h"
#include "uart.h"
#include "main.h"

#define RX_RING 256
static uint8_t rxRing[RX_RING];
static uint16_t rxPos;
static uint8_t txBuf[32];
static DMA_Channel_TypeDef* dmaRx;
static DMA_Channel_TypeDef* dmaTx;
static UART_TypeDef* uartReg;

#define DMA_CCR_EN_   (1u<<0)
#define DMA_CCR_DIR_  (1u<<4)    //memory to peripheral
#define DMA_CCR_CIRC_ (1u<<5)
#define DMA_CCR_MINC_ (1u<<7)

void SerialInit(void){
	UART_TypeDef* u = uarten==1 ? UART1 : UART2;
	uartReg = u;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	if(uarten==1){
		RCC->APB2ENR |= RCC_APB2ENR_EXTI;    //bit 0: SYSCFG (and EXTI)
		SYSCFG->CFGR |= SYSCFG_CFGR_UART1TX_DMA_RMP|SYSCFG_CFGR_UART1RX_DMA_RMP;
	}
	dmaRx = DMA1_Channel5;
	dmaTx = DMA1_Channel4;
	dmaRx->CCR = 0;
	dmaRx->CPAR = (u32)&u->RDR;
	dmaRx->CMAR = (u32)rxRing;
	dmaRx->CNDTR = RX_RING;
	dmaRx->CCR = DMA_CCR_MINC_|DMA_CCR_CIRC_|DMA_CCR_EN_;
	dmaTx->CCR = 0;
	dmaTx->CPAR = (u32)&u->TDR;
	rxPos = 0;
	u->GCR |= UART_GCR_DMA;
}

#define RX_ERRORS (UART_ISR_RXOERR|UART_ISR_RXPERR|UART_ISR_RXFERR|UART_ISR_RXBRK)
int SerialGetByte(void){
	//a receive error (break or frame error while the adapter drops off USB, an overrun) stops the rx DMA
	//requests until it is cleared: the board stayed deaf after a USB hub reset (measured)
	if(uartReg->ISR&RX_ERRORS) uartReg->ICR = RX_ERRORS;
	uint16_t w = (uint16_t)(RX_RING-dmaRx->CNDTR);    //next index the DMA writes
	if(w>=RX_RING) w = 0;
	if(rxPos==w) return -1;
	uint8_t c = rxRing[rxPos];
	rxPos = (uint16_t)((rxPos+1)%RX_RING);
	return c;
}

void SerialSend(const u8* buf, u16 len){
	if(len>sizeof(txBuf)) return;
	uint32_t t = millis;
	while((dmaTx->CCR&DMA_CCR_EN_)&&dmaTx->CNDTR&&millis-t<20);    //previous frame (15 bytes: 8 ms at 19200)
	dmaTx->CCR = 0;
	for(u16 i=0;i<len;i++) txBuf[i] = buf[i];
	dmaTx->CMAR = (u32)txBuf;
	dmaTx->CNDTR = len;
	dmaTx->CCR = DMA_CCR_MINC_|DMA_CCR_DIR_|DMA_CCR_EN_;
}
