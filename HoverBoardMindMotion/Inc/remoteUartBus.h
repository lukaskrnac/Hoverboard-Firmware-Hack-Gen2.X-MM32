#ifndef REMOTEUARTBUS_H
#define REMOTEUARTBUS_H

#include "mcu.h"

// wState bits of the type 0/1 command frames (same as the Gen2.x-GD32 firmware)
#define STATE_LED_GREEN      1
#define STATE_LED_ORANGE     2      //the "blue/orange" LED (LEDB)
#define STATE_LED_RED        4
#define STATE_LED_UP         8
#define STATE_LED_DOWN       16
#define STATE_LED_BATTLEVEL  32     //green/orange/red show the battery level instead of the bits above
#define STATE_DISABLE        64     //gates off, motor coasts (disarmed)
#define STATE_SHUTOFF        128    //not implemented

void RemoteUpdate(void);
uint8_t RemoteTimedOut(void);    //no command frame within SERIAL_TIMEOUT
void serialit(void);    //uart rx dma interrupt, one byte in sRxBuffer[0]

#endif
