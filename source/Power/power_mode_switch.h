/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _POWER_MODE_SWITCH_H_
#define _POWER_MODE_SWITCH_H_

#include "fsl_common.h"
#include "fsl_smc.h"
#include "fsl_llwu.h"
#include "fsl_rcm.h"
#include "fsl_lptmr.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pmc.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BAT_WAKEUP_GPIO         GPIOC
#define BAT_WAKEUP_PORT         PORTC
#define BAT_WAKEUP_PORT_CLK     kCLOCK_PortC
#define BAT_WAKEUP_GPIO_PIN     1U
#define BAT_WAKEUP_IRQ          PORTC_IRQn
#define BAT_WAKEUP_IRQ_HANDLER  PORTC_IRQHandler
#define BAT_WAKEUP_NAME         "battery wakeup"
#define BAT_WAKEUP_IRQ_TYPE     kPORT_InterruptEitherEdge
#define BAT_LLWU_WAKEUP_PIN_IDX 6U
#define BAT_LLWU_WAKEUP_PIN_TYPE kLLWU_ExternalPinAnyEdge 

#define ACC_WAKEUP_GPIO         ACC_GPIO
#define ACC_WAKEUP_PORT         ACC_PORT
#define ACC_WAKEUP_PORT_CLK     ACC_PORT_CLK
#define ACC_WAKEUP_GPIO_PIN     ACC_GPIO_PIN
#define ACC_WAKEUP_IRQ          ACC_PORT_IRQ
#define ACC_WAKEUP_IRQ_HANDLER  PORTC_IRQHandler
#define ACC_WAKEUP_NAME         "acc wakeup"
#define ACC_WAKEUP_IRQ_TYPE     kPORT_InterruptFallingEdge
#define ACC_LLWU_WAKEUP_PIN_IDX 10U                         
#define ACC_LLWU_WAKEUP_PIN_TYPE kLLWU_ExternalPinAnyEdge 

#define BMS_WAKEUP_GPIO         BMS_GPIO
#define BMS_WAKEUP_PORT         BMS_PORT
#define BMS_WAKEUP_PORT_CLK     BMS_PORT_CLK
#define BMS_WAKEUP_GPIO_PIN     BMS_GPIO_PIN
#define BMS_WAKEUP_IRQ          BMS_PORT_IRQ
#define BMS_WAKEUP_IRQ_HANDLER  PORTC_IRQHandler
#define BMS_WAKEUP_NAME         "bms wakeup"
#define BMS_WAKEUP_IRQ_TYPE     kPORT_InterruptFallingEdge
#define BMS_LLWU_WAKEUP_PIN_IDX 9U                          
#define BMS_LLWU_WAKEUP_PIN_TYPE kLLWU_ExternalPinAnyEdge 


/* Power mode definition used in application. */
typedef enum _app_power_mode
{
    kAPP_PowerModeMin = 'A' - 1,
    kAPP_PowerModeRun,   /* Normal RUN mode */
    kAPP_PowerModeWait,  /* WAIT mode. */
    kAPP_PowerModeStop,  /* STOP mode. */
    kAPP_PowerModeVlpr,  /* VLPR mode. */
    kAPP_PowerModeVlpw,  /* VLPW mode. */
    kAPP_PowerModeVlps,  /* VLPS mode. */
    kAPP_PowerModeLls,   /* LLS mode. */
    kAPP_PowerModeVlls0, /* VLLS0 mode. */
    kAPP_PowerModeVlls1, /* VLLS1 mode. */
    kAPP_PowerModeVlls2, /* VLLS2 mode. */
    kAPP_PowerModeVlls3, /* VLLS3 mode. */
    kAPP_PowerModeHsrun, /* HighSpeed RUN mode */
    kAPP_PowerModeMax
} app_power_mode_t;

typedef enum _app_wakeup_source
{
    kAPP_WakeupSourceLptmr, /*!< Wakeup by LPTMR.        */
    kAPP_WakeupSourcePin    /*!< Wakeup by external pin. */
} app_wakeup_source_t;

void PMS_SetWakeupConfig(app_power_mode_t targetMode);
bool PMS_CheckPowerMode(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode);
void PMS_PowerModeSwitch(smc_power_state_t curPowerState, app_power_mode_t targetPowerMode);

#endif /* _POWER_MODE_SWITCH_H_ */
