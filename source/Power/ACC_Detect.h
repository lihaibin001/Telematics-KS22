/* $Header:   ACC_Detect.h $*/
/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2017
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2017, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/
#ifndef __ACC_DETECT_H__
#define __ACC_DETECT_H__

#include "rtc.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#define ACC_PORT        PORTC
#define ACC_GPIO        GPIOC
#define ACC_PORT_CLK    kCLOCK_PortC
#define ACC_GPIO_PIN    6U
#define ACC_PORT_IRQ    PORTC_IRQn     

void ACC_InitializeMonitor(void);
void ACC_ACCMonitorProcess(void);
void ACC_SetACCStatus(bool status);
bool ACC_GetStatus(void);
void ACC_OnSignal(void);
void ACC_OffSignal(void);
bool ACC_GetCurrentSignalLevel(void);
#endif //__ACC_DETECT_H__