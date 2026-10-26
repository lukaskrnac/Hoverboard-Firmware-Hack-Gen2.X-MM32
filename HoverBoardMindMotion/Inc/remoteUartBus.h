#ifndef REMOTEUARTBUS_H
#define REMOTEUARTBUS_H

#include "mcu.h"

void AnswerMaster(void);
void RemoteUpdate(void);
void serialit(void);    //uart rx dma interrupt, one byte in sRxBuffer[0]

#endif
