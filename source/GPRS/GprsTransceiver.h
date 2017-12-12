#ifndef __GPRSTRANSCEIVER_H__
#define __GPRSTRANSCEIVER_H__

#include "GPRS.h"

bool GprsCreateTransceiverQueue(void);
bool GprsSendData(void *pData, uint16_t dataLen, uint8_t sesion, bool isNeedAck);
bool GprsReceiver(void *pData, uint16_t dataLen, uint8_t sesion);
void GprsReceiveChecker(void);

#endif //__GPRSTRANSCEIVER_H__