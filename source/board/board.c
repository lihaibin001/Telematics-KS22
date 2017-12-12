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

#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "CAN_HW.h"
#include "rtc.h"
#include "ACC_Detect.h"
#include "BMS_Detect.h"
#include "ExtBatt_Detect.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 0u, kPORT_MuxAlt7);
    PORT_SetPinMux(PORTB, 1u, kPORT_MuxAlt7);
    
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOADR_IO_Init(void)
{
    /* power control port */
    const gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0,};
    const gpio_pin_config_t gpio_input_config = {kGPIO_DigitalInput, 0,};
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    /* 4V control pin */
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 16U, &gpio_config);
    /* gsm powerkey pin */
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 7U, &gpio_config);
    /* 4V PG pin */
    PORT_SetPinMux(PORTA, GPIO_Pin_2, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, GPIO_Pin_2, &gpio_input_config);
    /* wakeup pin */
    PORT_SetPinMux(PORTC, GPIO_Pin_1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, GPIO_Pin_1, &gpio_input_config);
    /* Err pin */
    PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 19U, &gpio_input_config);
    /* charge pin */
    PORT_SetPinMux(PORTB, 3U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 3U, &gpio_config);    
    /* INH pin */
    //PORT_SetPinMux(PORTC, GPIO_Pin_6, kPORT_MuxAsGpio);
    //GPIO_PinInit(GPIOC, GPIO_Pin_6, &gpio_input_config);
    /* GSM power status pin */
    PORT_SetPinMux(PORTC, GPIO_Pin_7, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, GPIO_Pin_7, &gpio_input_config);
    /* BMS pin */
    PORT_SetPinMux(PORTC, GPIO_Pin_5, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, GPIO_Pin_5, &gpio_input_config);
    /* Acc pin */
    PORT_SetPinMux(PORTC, GPIO_Pin_6, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, GPIO_Pin_6, &gpio_input_config);
    /* RING pin */
    PORT_SetPinMux(PORTD, GPIO_Pin_4, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, GPIO_Pin_4, &gpio_input_config);

    /* gps enable pin */
    PORT_SetPinMux(PORTB, GPIO_Pin_2, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, GPIO_Pin_2, &gpio_config);
    /* gps reset pin */
    PORT_SetPinMux(PORTC, GPIO_Pin_8, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, GPIO_Pin_8, &gpio_config);
    
    /* can0 controller  pin */
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 0U, &gpio_config); 
    PORT_SetPinMux(PORTB, 17U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 17U, &gpio_config); 
    /*can inh*/
    PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 9U, &gpio_input_config);     
    /* flash hold pin */
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 0U, &gpio_config);     
    /* flash pw pin   */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 1U, &gpio_config);   
    /* GPIO3 control */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 1U, &gpio_config);   
}

void BOADR_Init(void)
{
    Uart_Initialize(0);  //Use for showing the run log, must do first! 
    VHCL_CAN_Init();
    ADC_Initialize();
    RTC_Initialize();
    BMS_InitializeMonitor();  
    ACC_InitializeMonitor(); 
    EB_InitializeMonitor();
    spi_flash_init();  
}
