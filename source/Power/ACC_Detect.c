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
#include "ACC_Detect.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum
{
    ACC_off,
    debouncing1,
    debouncing2,
    ACC_Init,
    debouncing3,
    debouncing4,
    ACC_on,
    ACC_Status_Num,
}ACC_Staus;
typedef struct
{
    uint32_t time;  //express the acc on time
    ACC_Staus status;
}ACC_StatusType;
/*******************************************************************************
 * Variables
 ******************************************************************************/
ACC_StatusType ACC_Status;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
*    Function: ACC_InitializeMonitor
*
*  Parameters: none
*     Returns: none
* Description: initialize the Battery Management System monitor
*******************************************************************************/
void ACC_InitializeMonitor(void)
{
    port_pin_config_t portPinConfig = {0};
    portPinConfig.pullSelect = kPORT_PullDisable;
    portPinConfig.mux = kPORT_MuxAsGpio;
    
    CLOCK_EnableClock(ACC_PORT_CLK);
    PORT_SetPinConfig(ACC_PORT, ACC_GPIO_PIN, &portPinConfig);
    PORT_SetPinInterruptConfig(ACC_PORT, ACC_GPIO_PIN, kPORT_InterruptEitherEdge); 
    ACC_Status.status = ACC_Init;
    EnableIRQ(ACC_PORT_IRQ);
}
/*******************************************************************************
*    Function: ACC_ACCMonitor
*
*  Parameters: none
*     Returns: none
* Description: monitor the Battery Management System
*******************************************************************************/
void ACC_ACCMonitorProcess(void)
{
    uint8_t signalLevel = !GPIO_ReadPinInput(ACC_GPIO, ACC_GPIO_PIN);

    if(signalLevel)
    {
        if(ACC_Status.status == ACC_on)
        {
            return;
        }
        ACC_Status.status++;
    }
    else
    {
        if(ACC_Status.status == ACC_off)
        {
            return;
        }
        ACC_Status.status--;
    }
}
/*******************************************************************************
*    Function: ACC_SetStatus
*
*  Parameters: status express the ACC status
*     Returns: true express obc is charging
* Description: Get the ACC status 
*******************************************************************************/
void ACC_SetACCStatus(bool status)
{
    
}
/*******************************************************************************
*    Function: ACC_GetStatus
*
*  Parameters: none
*     Returns: true express obc is charging
* Description: Get the ACC status 
*******************************************************************************/
bool ACC_GetStatus(void)
{
    if(ACC_Status.status == ACC_on)
    {
        return true;
    }
    return false;
}
/*******************************************************************************
*    Function: ACC_OnSignal
*
*  Parameters: none
*     Returns: none
* Description: set the ACC status to  ACC_Pre_on_1;
*******************************************************************************/
void ACC_OnSignal(void)
{
    ACC_Status.status++;
}
/*******************************************************************************
*    Function: ACC_OffSignal
*
*  Parameters: none
*     Returns: none
* Description: set the ACC status to  ACC_Pre_off_1;
*******************************************************************************/
void ACC_OffSignal(void)
{
    ACC_Status.status--;
}
/*******************************************************************************
*    Function: ACC_GetCurrentSignalLevel
*
*  Parameters: none
*     Returns: true express ACC on
* Description: Get the current ACC signal level;
*******************************************************************************/
bool ACC_GetCurrentSignalLevel(void)
{
    return !GPIO_ReadPinInput(ACC_GPIO, ACC_GPIO_PIN);

}

