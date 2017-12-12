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
#include "BMS_Detect.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef enum
{
    BMS_off,
    debouncing1,
    debouncing2,
    BMS_Init,
    debouncing3,
    debouncing4,
    BMS_on,
    BMS_Status_Num,
}BMS_Staus;
typedef struct
{
    uint32_t time;  //express the BMS on time
    BMS_Staus status;
}BMS_StatusType;
/*******************************************************************************
 * Variables
 ******************************************************************************/
BMS_StatusType BMS_Status;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
*    Function: BMS_InitializeMonitor
*
*  Parameters: none
*     Returns: none
* Description: initialize the Battery Management System monitor
*******************************************************************************/
void BMS_InitializeMonitor(void)
{
    port_pin_config_t portPinConfig = {0};
    portPinConfig.pullSelect = kPORT_PullDisable;
    portPinConfig.mux = kPORT_MuxAsGpio;
    
    CLOCK_EnableClock(BMS_PORT_CLK);
    PORT_SetPinConfig(BMS_PORT, BMS_GPIO_PIN, &portPinConfig);
    PORT_SetPinInterruptConfig(BMS_PORT, BMS_GPIO_PIN, kPORT_InterruptEitherEdge); 
    BMS_Status.status = BMS_Init;
    EnableIRQ(BMS_PORT_IRQ);
}
/*******************************************************************************
*    Function: BMS_BMSMonitor
*
*  Parameters: none
*     Returns: none
* Description: monitor the Battery Management System
*******************************************************************************/
void BMS_BMSMonitorProcess(void)
{
    if(BMS_Status.status == BMS_off || BMS_Status.status == BMS_on)
    {
        return;
    }
    else
    {
        uint8_t signalLevel = !GPIO_ReadPinInput(BMS_GPIO, BMS_GPIO_PIN);
        if(signalLevel)
        {
            BMS_Status.status++;
            if(BMS_Status.status == BMS_on)
            {
                BMS_Status.time = RTC_GetCounter();
            }
            return;
        }
        BMS_Status.status--;
    }
}
/*******************************************************************************
*    Function: BMS_SetStatus
*
*  Parameters: status express the BMS status
*     Returns: true express obc is charging
* Description: Get the BMS status 
*******************************************************************************/
void BMS_SetBMSStatus(bool status)
{
    
}
/*******************************************************************************
*    Function: BMS_GetStatus
*
*  Parameters: none
*     Returns: true express obc is charging
* Description: Get the BMS status 
*******************************************************************************/
bool BMS_GetStatus(void)
{
    if(BMS_Status.status == BMS_on)
    {
        return true;
    }
    return false;
}
/*******************************************************************************
*    Function: BMS_OnSignal
*
*  Parameters: none
*     Returns: none
* Description: set the BMS status to  BMS_Pre_on_1;
*******************************************************************************/
void BMS_OnSignal(void)
{
    BMS_Status.status++;
}
/*******************************************************************************
*    Function: BMS_OffSignal
*
*  Parameters: none
*     Returns: none
* Description: set the BMS status to  BMS_Pre_off_1;
*******************************************************************************/
void BMS_OffSignal(void)
{
    BMS_Status.status--;
}
/*******************************************************************************
*    Function: BMS_GetCurrentSignalLevel
*
*  Parameters: none
*     Returns: true express BMS on
* Description: Get the current BMS signal level;
*******************************************************************************/
bool BMS_GetCurrentSignalLevel(void)
{
    return !GPIO_ReadPinInput(BMS_GPIO, BMS_GPIO_PIN);

}

