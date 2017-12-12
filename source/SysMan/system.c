/* $Header:   system.c  $*/
/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2016
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2016, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/

/*===========================================================================*\
   Title                    : SYSTEM.C

   Module Description       : This is the standard code file for SYSTEM
                              for 78K0R core.

   Author                   : 

\*===========================================================================*/

/*===========================================================================*\
 * Installation Instructions (periodic tasks, etc.)
 *
 *
\*===========================================================================*/


/*===========================================================================*\
 * Include header files
\*===========================================================================*/
/*===========================================================================*/
#include "stdint.h"
#include "stdbool.h"
#include "standard.h"

#include "fsl_wdog.h"
/*===========================================================================*\
 * File level pragmas
\*===========================================================================*/

/*===========================================================================*\
 * Constant and Macro Definitions using #define
\*===========================================================================*/
#define SY_HW_ID       {'V','1','.','0'} 
#define SY_SW_ID       {'D','R','0','0','3','-','-','1','7','D','0','1','A','0','0','1'}
#define SY_SW_DATE     {'2','0','1','7','0','1','2','0','R'}
#define SY_PROTOCOL_VER  {0,1}

#define LED_MAX_NUM (2)

/*===========================================================================*\
 * Enumerations and Structures and Typedefs
\*===========================================================================*/

/*===========================================================================*\
 * Global and Const Variable Defining Definitions / Initializations
\*===========================================================================*/
const int8_t  Sys_HWID[] = SY_HW_ID;  // HW ID,FOR test,need get the ID from EEPROM:SCB
const int8_t  Sys_SWID[] = SY_SW_ID;
const int8_t  Sys_SW_Date[] = SY_SW_DATE;
const int8_t  Sys_Protocol_Ver[] = SY_PROTOCOL_VER;

/*===========================================================================*\
 * Static Variables and Const Variables With File Level Scope
\*===========================================================================*/
__no_init wake_up_src_flags_t sys_wakeup_src_flags;/* clear after each warm start!!! */
__no_init  bool sys_sleep_request_flag;/*    */
__no_init  uint8_t sys_2g_request_flag;/*    */

//static bool watchdog_reset_flag = false;
static LED_Toggle_Period_T Led_Period[LED_MAX_NUM];
//static uint8_t Led_ID = 0;
/*===========================================================================*\
 * ROM Const Variables With File Level Scope
\*===========================================================================*/


/*===========================================================================*\
 * Function Prototypes for Private Functions with File Level Scope
\*===========================================================================*/

/*===========================================================================*\
 * Add User defined functions
\*===========================================================================*/

#ifndef SY_POWER_OFF_DELAY
#   define SY_POWER_OFF_DELAY (MSec_To_Ticks(200))
#endif /* SY_POWER_OFF_DELAY */

/*===========================================================================*\
 * Function Definitions
\*===========================================================================*/
/**********************************************************************
 *    Function: SY_Swid()
 *
 *  Parameters: None.
 *
 *     Returns: sy_swid
 *
 * Description: Return the current S/W ID revision.
 *
 **********************************************************************/
int8_t *SY_Swid(void)
{
   return((int8_t *) &Sys_SWID[0]);
}
/**********************************************************************
 *    Function: SY_Swid()
 *
 *  Parameters: None.
 *
 *     Returns: sy_swid
 *
 * Description: Return the current S/W ID revision.
 *
 **********************************************************************/
int8_t *SY_Hwid(void)
{
   return((int8_t *)&Sys_HWID[0]);
}
/**********************************************************************
 *    Function: SY_Sw_Date()
 *
 *  Parameters: None.
 *
 *     Returns: sy_swid
 *
 * Description: Return the current S/W Date revision.
 *
 **********************************************************************/
int8_t *SY_Sw_Date(void)
{
   return((int8_t *) &Sys_SW_Date[0]);
}

/**********************************************************************
 *    Function: SY_Sw_Version()
 *
 *  Parameters: None.
 *
 *     Returns: sy_swid
 *
 * Description: Return the current S/W Date revision.
 *
 **********************************************************************/
int8_t *SY_Sw_Version(void)
{
   return((int8_t *) &Sys_SWID[0]);
}

/**********************************************************************
 *    Function: SY_Protocol_Version()
 *
 *  Parameters: None.
 *
 *     Returns: sy_protocol_ver
 *
 * Description: Return the current S/W Protocol revision.
 *
 **********************************************************************/
int8_t *SY_Protocol_Version(void)
{
   return((int8_t *) &Sys_Protocol_Ver[0]);
}

static void Force_WatchDog_Reset(void)
{
    wdog_config_t config;
    WDOG_GetDefaultConfig(&config);
    config.timeoutValue=0x0010U;
    WDOG_Init(WDOG,&config);
}

/*===========================================================================*\
 *    Function: Restart
 *
 *  Parameters: type = WARM or COLD
 *
 *     Returns: None
 *
 * Description: Forces obd into warm start after EE Queue is cleaned up.
 *
\*===========================================================================*/
void Restart(bool warm)
{
   static bool do_warm;

   do_warm = warm;

   if (do_warm)                 /* true if warm start is desired */
   {
      Set_warm_Start();
#if 1
      //NVIC_SystemReset();
      Force_WatchDog_Reset();
#endif
   }
   else
   {
#if 1
      //NVIC_SystemReset();
      Force_WatchDog_Reset();   /* force reset (cold start) via watchdog */
#endif
   }
}
/**********************************************************************
*    Function: Sys_Set_CAN_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_CAN_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= CAN_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Clear_CAN_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_CAN_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  &= ~CAN_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Is_CAN_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_CAN_Wakeup (void)
{
    return(CAN_WAKE_UP & sys_wakeup_src_flags.all) ; 
}

/**********************************************************************
*    Function: Sys_Is_2G_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_2G_Wakeup (void)
{
    return false;
}

/**********************************************************************
*    Function: Sys_Set_2G_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_2G_Wakeup_Flag (void)
{
}

/**********************************************************************
*    Function: Sys_Clear_2G_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_2G_Wakeup_Flag (void)
{
}


/**********************************************************************
*    Function: Sys_Set_Low_Batt_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_Low_Batt_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= LOW_BAT_WAKE_UP; 
}
/**********************************************************************
*    Function: Sys_Is_Low_Batt_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_Low_Batt_Wakeup (void)
{
    return(LOW_BAT_WAKE_UP & sys_wakeup_src_flags.all) ; 
}

/**********************************************************************
*    Function: Sys_Clear_Low_Batt_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_Low_Batt_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  &= ~LOW_BAT_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Set_Gsensor_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_Gsensor_Wakeup_Flag(void)
{
}

/**********************************************************************
*    Function: Sys_Is_Gsensor_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_Gsensor_Wakeup(void)
{
    return false;
}

/**********************************************************************
*    Function: Sys_Clear_Gsensor_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_Gsensor_Wakeup_Flag (void)
{
}

/**********************************************************************
*    Function: Sys_Set_RTC_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_RTC_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= RTC_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Is_RTC_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_RTC_Wakeup (void)
{
    return(RTC_WAKE_UP & sys_wakeup_src_flags.all) ; 
}

/**********************************************************************
*    Function: Sys_Clear_RTC_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_RTC_Wakeup (void)
{
    sys_wakeup_src_flags.all  &= ~RTC_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Set_RTC_Deep_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_RTC_Deep_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= RTC_DEEP_STDBY_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Is_RTC_Deep_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_RTC_Deep_Wakeup (void)
{
    return(RTC_DEEP_STDBY_WAKE_UP & sys_wakeup_src_flags.all) ; 
}

/**********************************************************************
*    Function: Sys_Clear_RTC_Deep_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_RTC_Deep_Wakeup (void)
{
    sys_wakeup_src_flags.all  &= ~RTC_DEEP_STDBY_WAKE_UP; 
}

/* 2017.7.7 lihaibin modify */
/**********************************************************************
*    Function: Sys_Set_ACC_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_ACC_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= ACC_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Clear_ACC_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_ACC_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  &= ~ACC_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Is_ACC_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_ACC_Wakeup (void)
{
    return(ACC_WAKE_UP & sys_wakeup_src_flags.all) ; 
}

/**********************************************************************
*    Function: Sys_Set_BMS_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Set_BMS_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  |= BMS_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Clear_BMS_Wakeup_Flag
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
void Sys_Clear_BMS_Wakeup_Flag (void)
{
    sys_wakeup_src_flags.all  &= ~BMS_WAKE_UP; 
}

/**********************************************************************
*    Function: Sys_Is_BMS_Wakeup
*  Parameters: none
*     Returns: none
* Description: 
**********************************************************************/
bool Sys_Is_BMS_Wakeup (void)
{
    return(BMS_WAKE_UP & sys_wakeup_src_flags.all) ; 
}
/* end 2017.7.7 lihaibin modify */

/**********************************************************************
 *       Name: Sys_Get_Wakeup_Src_Flags(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
uint8_t Sys_Get_Wakeup_Src_Flags(void)
{
    return (sys_wakeup_src_flags.all);
}
/**********************************************************************
 *       Name: Sys_Clear_Wakeup_Src_Flags(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
void Sys_Clear_Wakeup_Src_Flags(void)
{
    sys_wakeup_src_flags.all = 0x00;
}

/**********************************************************************
 *       Name: Sys_Clear_Standby_Req_Flag(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
void Sys_Clear_Standby_Req_Flag(void)
{
    sys_sleep_request_flag = false;
}
/**********************************************************************
 *       Name: Sys_Set_Standby_Req_Flag(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
void Sys_Set_Standby_Req_Flag(void)
{
    sys_sleep_request_flag = 0x5A; 
}
/**********************************************************************
 *       Name: Sys_Get_Standby_Req_Flag(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
bool Sys_Get_Standby_Req_Flag(void)
{
    return (sys_sleep_request_flag);
}

/**********************************************************************
 *       Name: Sys_Req_Enter_Deep_Standby(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
void  Sys_Req_Enter_Deep_Standby(void)
{
     Sys_Set_Standby_Req_Flag();
     SY_Warm_Start(); 	
}

/**********************************************************************
 *       Name: LED_Set_Toggle_Period(LED_Toggle_Period_T period )
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/

void LED_Set_Toggle_Period(uint8_t Led_id, LED_Toggle_Period_T period )
{
	Led_Period[Led_id] = period;
}
/**********************************************************************
 *       Name: LED_Get_Toggle_Period(void)
 *    Purpose: 
 * Parameters: none
 *    Returns: none
 *********************************************************************/
LED_Toggle_Period_T LED_Get_Toggle_Period(uint8_t Led_id)
{
	return Led_Period[Led_id];
}

/*===========================================================================*\
 * File Revision History
 *===========================================================================
 *
\*===========================================================================*/

