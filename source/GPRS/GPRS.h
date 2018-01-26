#ifndef __GPRS_H__
#define __GPRS_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "Task.h"
#include "event_groups.h"	 
#include "queue.h"

typedef void (*ipSendCb)(bool isSendOk, uint8_t sesionNum);
typedef void (*ipRxDataHanle)(uint8_t *pData, uint16_t len, uint8_t sesinoNum);
typedef void (*ipConnectCb)(bool state, uint8_t sesinoNum);
/*net sesion's state */
typedef enum {
//	ipSesionInactive,
	ipSesionClosing,
	ipSesionClose,
	ipSesionOpening,
	ipSesionOpened,
//	ipSesionIdle,
//	ipSesionTransmit,
} netSesionState_t;

TaskHandle_t IOT_Gethandle(void);
void IOT_Task(void *pParm);
bool IOT_Init(void);
bool IOT_PowerDown(void);
bool IOT_PowerUp(void);
bool IOT_SelfTeste(void);
bool IOT_IpNetSend(uint8_t channel, uint8_t *pData, uint16_t len, ipSendCb cb);
bool IOT_IpOpen(uint8_t channel, uint32_t waitTime);
bool IOT_IpOpenUnblock(uint8_t channel);
bool IOT_IpClose(uint8_t channel);
bool IOT_Restart(void);
bool IOT_isModuleReady(void);
netSesionState_t IOT_GetSessionState(uint8_t num);
bool IOT_GetImei(uint8_t *pBuffer);
bool IOT_GetImsi(uint8_t *pBuffer);
bool IOT_GetIccid(uint8_t *pBuffer);
uint8_t IOT_GetCsq(void);
uint8_t IOT_IsIpTransOk(void);
uint8_t IOT_IsSimReady(void);
void IOT_receiveDatahanlerRegester(ipRxDataHanle handle);
void IOT_ipConnectStateChangeNotifierRegester(ipConnectCb notifier);
#endif
