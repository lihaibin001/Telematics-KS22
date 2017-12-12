/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "wdog.h"
#include "debug.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define WDOG_WCT_INSTRUCITON_COUNT (256U)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
*    Function: WaitWctClose
*
*  Parameters: 'base' point to the watchdog.
*     Returns: None.
* Description: 
*******************************************************************************/
static void WaitWctClose(WDOG_Type *base)
{
    /* Accessing register by bus clock */
    for (uint32_t i = 0; i < WDOG_WCT_INSTRUCITON_COUNT; i++)
    {
        (void)base->RSTCNT;
    }
}
/*******************************************************************************
*    Function: WDOG_Initialize
*
*  Parameters: None.
*     Returns: None.
* Description: Initialize the watchdog controller.
*******************************************************************************/
void WDOG_Initialize(void)
{
    uint16_t wdog_reset_count = 0;
    wdog_test_config_t test_config;
    wdog_config_t config;
    WDOG_Type *wdog_base = WDOG;
    RCM_Type *rcm_base = RCM;
    if (!(RCM_GetPreviousResetSources(rcm_base) & kRCM_SourceWdog))
    {
        /*If not wdog reset*/
        WDOG_ClearResetCount(wdog_base);
    }
    wdog_reset_count = WDOG_GetResetCount(wdog_base);
    if (wdog_reset_count == 0)
    {
        /*quick test*/
        test_config.testMode = kWDOG_QuickTest;
        test_config.timeoutValue = 0xfffffu;
        test_config.testedByte = kWDOG_TestByte0;
        DEBUG(DEBUG_LOW,"[WDOG] Quick test\r\n");
        WDOG_SetTestModeConfig(wdog_base, &test_config);
        WaitWctClose(wdog_base);

        /*wait for timeout reset*/
        while (1)
        {
        }
    }
    else if (wdog_reset_count == 1)
    {
        DEBUG(DEBUG_LOW, "[WDOG] Quick test done\r\n");
        /*
         * config.enableWdog = true;
         * config.clockSource = kWDOG_LpoClockSource;
         * config.prescaler = kWDOG_ClockPrescalerDivide1;
         * config.enableUpdate = true;
         * config.enableInterrupt = false;
         * config.enableWindowMode = false;
         * config.windowValue = 0U;
         * config.timeoutValue = 0xFFFFU;
         */
        WDOG_GetDefaultConfig(&config);
        config.timeoutValue = 0x7ffU;
        /* wdog refresh test in none-window mode */
        DEBUG(DEBUG_LOW, "[WDOG] None-window mode refresh test start\r\n");
        WDOG_Init(wdog_base, &config);
        WaitWctClose(wdog_base);
        for (uint32_t i = 0; i < 10; i++)
        {
            WDOG_Refresh(wdog_base);
            while (GetTimerOutputValue(wdog_base) < (config.timeoutValue >> 3U))
            {
            }
        }
        /* wait for wdog timeout reset */
        while (1)
        {
        }
    }
    else if(wdog_reset_count == 2)
    {
        DEBUG(DEBUG_LOW, "[WDOG] None-Window mode test done\r\n");
    } 
    else
    {
        DEBUG(DEBUG_HIGH, "[WDOG] Warning System was reseted by WDOG\r\n");
    }
}
