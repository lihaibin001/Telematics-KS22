/* $Header:   lowpower.c   $*/
/**********************************************************************
 *             Title:   lowpower.C
 *
 *       Description:   This file contains micro specific code to place 
 *                      it into a low power state 
 *
 *            Author:  
 *
 *********************************************************************/

/**********************************************************************
 * Installation Instructions (periodic tasks, etc.)
 *
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
/* Dependent "optimize.cmd"                                          */
/*********************************************************************/
#include    "lowpower.h"
#include "debug.h"
/*********************************************************************
 * File level pragmas
 *********************************************************************/
 
/*---------------------------------------------------------------------
 * pragma statements to keep all "boot" at the beginning of memory 
 *-------------------------------------------------------------------*/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define PSMR_IDLE (0x02)  /* select idle 2 power save mode */
#define PSMR_STOP (0x01)  /* select stop power save mode */
/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
//static uint8_t wkup_enable=0;
/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/


/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/


/**********************************************************************
 * Add User defined functions
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
 
#ifdef STOP_OSCILLATOR
#define PSMR_REG PSMR_STOP
#else
#define PSMR_REG PSMR_IDLE
#endif

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/**********************************************************************
 * Description: Places the processor into STOP or IDLE2 mode
 *    If not stop oscillator  
 *       Main oscillator running 
 *       Watch timer running  (RTI / TOD)
 *
 *    PLL Stopped 
 *    CPU Clock stopped  
 *    Peripheral Clock stopped
 * 
 *    After wake-up, the PLL will still be disabled.
 *
 *  The user must configure wakeup interrupts before calling this function
 *  
 *  Parameters: None
 *     Returns: None
 *********************************************************************/
void Micro_Go_Low_Power( void)
{
    smc_power_state_t curPowerState;
    PMS_SetWakeupConfig(kAPP_PowerModeVlls0);
    
    /* switch to run mode first */
    curPowerState = SMC_GetPowerModeState(SMC);
    if(curPowerState != kSMC_PowerStateRun)
    {
        if(!PMS_CheckPowerMode(curPowerState, kAPP_PowerModeRun))
        {
            return;
        }
        PMS_PowerModeSwitch(curPowerState, kAPP_PowerModeRun);
        curPowerState = SMC_GetPowerModeState(SMC);
    }
    if(curPowerState == kSMC_PowerStateRun)
    {
        PMS_PowerModeSwitch(curPowerState, kAPP_PowerModeVlls0);    
    }
    else
    {
        //DEBUG(DEBUG_HIGH, "[LP] Can not swicht\r\n");
    }
}




/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
