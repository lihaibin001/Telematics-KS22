/* $Header:   timer.h    $*/
/*************************************************************************
   Title                    : TIMER.H

   Module Description       : This file has all of the standard global defines
                              for module TIMER

   Author                   : 

   Created                  : 

   Configuration ID         : 

*************************************************************************/

/*----------------------------------------------------------------------
*   Instructions for using this module if any:
*
*---------------------------------------------------------------------*/
#ifndef  TIMER_H
#define  TIMER_H

/*********************************************************************/
/* Include files                                                     */
/*********************************************************************/
#include   "standard.h"        /* include standard includes */

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/

/**********************************************************************
*    Function: TMR_Check_Timer
*  Parameters: timer id
*     Returns: true  if timer is active AND ran down (no more running)?
*              false otherwise
* Description: returns if a certain timer ran down or not
**********************************************************************/
#define TMR_Check_Timer(x)    (TMR_Is_Timer_Active(x) && !TMR_Is_Timer_Running(x))
#define TMR_Is_Timer_Stop(x)    (!(TMR_Is_Timer_Active(x) && TMR_Is_Timer_Running(x)))
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef enum
{
    PS_MIN_AWAKE_TIMER,
    PS_STDBY_TIMER,
    PS_STDBY_TIMEOUT_TIMER, //timer for force sleep
    PS_MONITOR_TIMER,//Monitor ACC ON/OFF
    TELM_TCP_TIMER,	//timer waiting for tcp link time out	
    TELM_UPLOAD_TIMER,//timer waiting for telmatics server ACK timeout
    TELM_COMMAND_TIMER,//timer waiting for execute telmatics command timeout
    TELM_BATT_CHECK_DELAY_TIMER,//for delay sometime to check battery
    TELM_BATT_CHECK_TIMER,//for  check battery
    PS_CRANK_TIMER,//crank, start communicate
    PS_CHARGE_TIMER,
    
    GPRS_START_TIMER,
    PS_DEV_ON_TIMER,
    DEV_TRACE_TIMER, // for device trace.
    //DEV_WAKE_TIMER, // wait 1 minute, then sleep
    //GPS_UPLOAD_TIMER, // wait for 45s to upload GPS
    
    //RECORD_SEND_TIMER,
    
    //LONGIN_RETRY_TIMER,
    
    //RECORD_RECORD_TIEMR,
    //RECORD_CHECK_TIMER,
    //////////////////
    NUMBER_OF_TIMERS
} TMR_ID_T;

/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/
extern void TMR_Initialize(void);
extern void TMR_Check_Timers(void);
extern void TMR_Start_Timer(TMR_ID_T timer_id, Tick_Type period, void_fptr func_ptr);
extern void TMR_Stop_Timer(TMR_ID_T timer_id);
extern bool TMR_Is_Timer_Active(TMR_ID_T timer_id);
extern bool TMR_Is_Timer_Running(TMR_ID_T timer_id);
extern Tick_Type TMR_Get_Pending_Time(TMR_ID_T timer_id);

extern void null_action (void);

#endif

/**********************************************************************
*                                                                     *
* REVISION RECORDS                                                    *
*                                                                     *
**********************************************************************/
/*********************************************************************/
/* 
 *
 *********************************************************************/

