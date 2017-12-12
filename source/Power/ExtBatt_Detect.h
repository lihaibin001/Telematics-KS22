/* $Header:   EB_Detect.h $*/
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
#ifndef __EB_DETECT_H__
#define __EB_DETECT_H__

#include "rtc.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#define EB_PORT        PORTC
#define EB_GPIO        GPIOC
#define EB_PORT_CLK    kCLOCK_PortC
#define EB_GPIO_PIN    1U
#define EB_PORT_IRQ    PORTC_IRQn     

void EB_InitializeMonitor(void);
void EB_EBMonitorProcess(void);
void EB_SetEBStatus(bool status);
bool EB_GetStatus(void);
void EB_OnSignal(void);
void EB_OffSignal(void);
bool EB_GetCurrentSignalLevel(void);
#endif //__EB_DETECT_H__