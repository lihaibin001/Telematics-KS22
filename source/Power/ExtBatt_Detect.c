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
#include "ExtBatt_Detect.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum
{
    EB_off,
    debouncing1,
    debouncing2,
    EB_Init,
    debouncing3,
    debouncing4,
    EB_on,
    EB_Status_Num,
}EB_Staus;
typedef struct
{
    uint32_t time;  //express the EB on time
    EB_Staus status;
}EB_StatusType;
/*******************************************************************************
 * Variables
 ******************************************************************************/
EB_StatusType EB_Status;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
*    Function: EB_InitializeMonitor
*
*  Parameters: none
*     Returns: none
* Description: initialize the Battery Management System monitor
*******************************************************************************/
void EB_InitializeMonitor(void)
{
    port_pin_config_t portPinConfig = {0};
    portPinConfig.pullSelect = kPORT_PullDisable;
    portPinConfig.mux = kPORT_MuxAsGpio;
    
    CLOCK_EnableClock(EB_PORT_CLK);
    PORT_SetPinConfig(EB_PORT, EB_GPIO_PIN, &portPinConfig);
    PORT_SetPinInterruptConfig(EB_PORT, EB_GPIO_PIN, kPORT_InterruptEitherEdge); 
    EB_Status.status = EB_Init;
    EnableIRQ(EB_PORT_IRQ);
}
/*******************************************************************************
*    Function: EB_EBMonitor
*
*  Parameters: none
*     Returns: none
* Description: monitor the Battery Management System
*******************************************************************************/
void EB_EBMonitorProcess(void)
{
    if(EB_Status.status == EB_off || EB_Status.status == EB_on)
    {
        return;
    }
    else
    {
        uint8_t signalLevel = !GPIO_ReadPinInput(EB_GPIO, EB_GPIO_PIN);
        if(signalLevel)
        {
            EB_Status.status++;
            if(EB_Status.status == EB_on)
            {
                EB_Status.time = RTC_GetCounter();
            }
            return;
        }
        if(EB_Status.status-- == EB_off)
        {
            //report external battery off warning
        }
        
    }
}
/*******************************************************************************
*    Function: EB_SetStatus
*
*  Parameters: status express the EB status
*     Returns: true express obc is charging
* Description: Get the EB status 
*******************************************************************************/
void EB_SetEBStatus(bool status)
{
    
}
/*******************************************************************************
*    Function: EB_GetStatus
*
*  Parameters: none
*     Returns: true express obc is charging
* Description: Get the EB status 
*******************************************************************************/
bool EB_GetStatus(void)
{
    if(EB_Status.status == EB_on)
    {
        return true;
    }
    return false;
}
/*******************************************************************************
*    Function: EB_OnSignal
*
*  Parameters: none
*     Returns: none
* Description: set the EB status to  EB_Pre_on_1;
*******************************************************************************/
void EB_OnSignal(void)
{
    EB_Status.status++;
}
/*******************************************************************************
*    Function: EB_OffSignal
*
*  Parameters: none
*     Returns: none
* Description: set the EB status to  EB_Pre_off_1;
*******************************************************************************/
void EB_OffSignal(void)
{
    EB_Status.status--;
}
/*******************************************************************************
*    Function: EB_GetCurrentSignalLevel
*
*  Parameters: none
*     Returns: true express EB on
* Description: Get the current EB signal level;
*******************************************************************************/
bool EB_GetCurrentSignalLevel(void)
{
    return !GPIO_ReadPinInput(EB_GPIO, EB_GPIO_PIN);

}

