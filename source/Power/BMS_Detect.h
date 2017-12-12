/* $Header:   BMS_Detect.h $*/
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
#ifndef __BMS_DETECT_H__
#define __BMS_DETECT_H__

/*******************************************************************************
 * include
 ******************************************************************************/
#include "rtc.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BMS_PORT        PORTC
#define BMS_GPIO        GPIOC
#define BMS_PORT_CLK    kCLOCK_PortC
#define BMS_GPIO_PIN    5U
#define BMS_PORT_IRQ    PORTC_IRQn 

/*******************************************************************************
 * Declaration
 ******************************************************************************/
void BMS_InitializeMonitor(void);
void BMS_BMSMonitorProcess(void);
void BMS_SetBMSStatus(bool status);
bool BMS_GetStatus(void);
void BMS_OnSignal(void);
void BMS_OffSignal(void);
bool BMS_GetCurrentSignalLevel(void);
#endif //__BMS_DETECT_H__