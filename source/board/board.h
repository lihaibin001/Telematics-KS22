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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_rtc.h"
#include "standard.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The board name */
#define BOARD_NAME "TBOX_HY"

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_UART
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) UART0
#define BOARD_DEBUG_UART_CLKSRC kCLOCK_CoreSysClk
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetCoreSysClkFreq()
#define BOARD_UART_IRQ UART0_RX_TX_IRQn
#define BOARD_UART_IRQ_HANDLER UART0_RX_TX_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 9600
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn /*!< Tickless timer IRQ number. */
     
#define IO_4V_CTRL_OUT(value)		        GPIO_WritePinOutput(GPIOB, 16U, value)
#define IO_GSM_PWR_ON_OUT(value)			GPIO_WritePinOutput(GPIOD, 7U, value)
#define IO_GSM_STAUS()                      GPIO_ReadPinInput(GPIOC, GPIO_Pin_7)
#define IO_GPS_RESET_OUT(value)	    		GPIO_WritePinOutput(GPIOC, GPIO_Pin_8, value)
#define IO_3V3_GPS_EN_OUT(value)           	GPIO_WritePinOutput(GPIOB, GPIO_Pin_2, value)
#define IO_FLASH_WP_OUT(value)		        GPIO_WritePinOutput(GPIOD, GPIO_Pin_1, value)
#define IO_FLASH_HOLD_OUT(value)		    GPIO_WritePinOutput(GPIOD, GPIO_Pin_0, value)
#define IO_CHARGE_CTL(value)		        GPIO_WritePinOutput(GPIOB, GPIO_Pin_3, value)
#define IO_CHRG_IS_IDLE()		            GPIO_ReadPinInput(GPIOA, GPIO_Pin_5)
#define IO_LED1_CTL_OUT(value)		        GPIO_WritePinOutput(GPIOC, GPIO_Pin_11, value)
#define IO_LED2_CTL_OUT(value)		        GPIO_WritePinOutput(GPIOC, GPIO_Pin_10, value)
#define IO_CAN_STANBY_OUT(value)            GPIO_WritePinOutput(GPIOB, GPIO_Pin_17, value)
#define IO_CAN_ENABLE_OUT(value)            GPIO_WritePinOutput(GPIOC, GPIO_Pin_0, value)
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
}BitAction;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
void BOADR_IO_Init(void);
void BOADR_Init(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
