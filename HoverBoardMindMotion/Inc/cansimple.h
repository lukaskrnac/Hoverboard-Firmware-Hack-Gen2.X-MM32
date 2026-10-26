#ifndef CANSIMPLE_H
#define CANSIMPLE_H
#include "mcu.h"

//ODrive CANSimple protocol over the RemoteUartBus (cansimple.c)
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 0
#define FW_VERSION_REVISION 0

uint8_t CanSimpleFeed(uint8_t c);    //one received byte; 0: not inside a CANSimple frame (for the v1 parser)
uint8_t SdoAccess(const uint8_t* d, uint8_t len, uint8_t* out);    //RxSdo payload -> TxSdo payload length
int32_t StepsPerTurn(void);
float TorqueConstant(void);

#endif
