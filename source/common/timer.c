/* $Header:   timer.c   $*/
/**********************************************************************
   Title                    : TIMER.C

   Module Description       : This is the standard code file for the
                              Timer module

   Author                   : 

   Created                  : 

   Configuration ID         : 

**********************************************************************/

/*********************************************************************
* Installation Instructions (periodic tasks, etc.)
*
*
**********************************************************************/


/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/* Dependent "compile.cmd"                                           */
/*********************************************************************/
#include    "standard.h"

/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/
/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/

// faster as subroutine (timer_id must be valid)
#define Timer_Is_Timer_Active(timer_id)  (tmr_array[(timer_id)].active_status)
#define Timer_Is_Timer_Running(timer_id) (OS_Time() < tmr_array[timer_id].time_stamp)
#define rl_write_timer(x, y)      (rl_timer[x] = y)
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef struct TMR_Data_Tag
{
    bool        active_status;
    Tick_Type   time_stamp;
    void_fptr   fptr;
} TMR_Data_T;

/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static TMR_Data_T tmr_array[NUMBER_OF_TIMERS];

/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/

/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/

/*********************************************************************/
/* Function Definitions                                              */
/*********************************************************************/

/**********************************************************************
*
*    Function: TMR_Initialize
*
*  Parameters: none
*
*     Returns: none
*
* Description: initialization of the timer module
*
*
**********************************************************************/
void TMR_Initialize (void)
{
    TMR_ID_T i;

    for (i=(TMR_ID_T)0; i < NUMBER_OF_TIMERS; i++)
    {
        tmr_array[i].active_status = false;
        tmr_array[i].time_stamp    = 0;
        tmr_array[i].fptr          = null_action;
    }
}

/**********************************************************************
*
*    Function: TMR_Check_Timers
*
*  Parameters: none
*
*     Returns: none
*
* Description: checks automatically all timers (called in Periodic_Task)
*
*
**********************************************************************/
void TMR_Check_Timers (void)
{
    TMR_ID_T i;

    for (i = (TMR_ID_T)0; i < NUMBER_OF_TIMERS; i++)
    {
        // is timer active AND ran down (no more running)?
        if (Timer_Is_Timer_Active(i) && !Timer_Is_Timer_Running(i))
        {
            // in case of null_action don't stop the timer because 
            // the task who called this service wants to check the
            // timer itself
            if(tmr_array[i].fptr != null_action)
            {
                TMR_Stop_Timer(i);
            }
            (*tmr_array[i].fptr)();     //execute the desired function
        }
    }
}

/**********************************************************************
*
*    Function: TMR_Start_Timer
*
*  Parameters: timer_id - id name of the timer
*              period   - amount of ticks the timer should run down
*
*     Returns: none
*
* Description: starts/restarts a timer
*
*
**********************************************************************/
void TMR_Start_Timer (TMR_ID_T timer_id, Tick_Type period, void_fptr func_ptr)
{
    if (NUMBER_OF_TIMERS > timer_id)
    {
        tmr_array[timer_id].active_status = true;
        tmr_array[timer_id].time_stamp    = OS_Time() + period;
        tmr_array[timer_id].fptr          = ((NULL == func_ptr) ? null_action : func_ptr);
    }
}

/**********************************************************************
*
*    Function: TMR_Stop_Timer
*
*  Parameters: timer_id - id name of the timer
*
*     Returns: none
*
* Description: stops a timer
*
*
**********************************************************************/
void TMR_Stop_Timer (TMR_ID_T timer_id)
{
    if (NUMBER_OF_TIMERS > timer_id)
    {
        tmr_array[timer_id].active_status = false;
        tmr_array[timer_id].time_stamp    = 0;
        // tmr_array[timer_id].fptr          = null_action; 
        //not reset the pointer, because the sequence is changed in TMR_Check_Timers()
    }
}

/**********************************************************************
*
*    Function: TMR_Is_Timer_Active
*
*  Parameters: timer_id - id name of the timer
*
*     Returns: true  - timer is in use, that is active
*              false - timer is not in use
*
* Description: returns if a certain timer is active or not
*
*
**********************************************************************/
bool TMR_Is_Timer_Active (TMR_ID_T timer_id)
{
    if (NUMBER_OF_TIMERS > timer_id)
    {
        return(Timer_Is_Timer_Active(timer_id));
    }

    return(false);
}

/**********************************************************************
*
*    Function: TMR_Is_Timer_Running
*
*  Parameters: timer_id - id name of the timer
*
*     Returns: true  - timer is still running
*              false - timer is expired
*
* Description: returns if a certain timer is running or not
*
*
**********************************************************************/
bool TMR_Is_Timer_Running (TMR_ID_T timer_id)
{
    if (NUMBER_OF_TIMERS > timer_id)
    {
        return(Timer_Is_Timer_Running(timer_id));
    }

    return(false);
}

/**********************************************************************
*
*    Function: TMR_Get_Pending_Time
*
*  Parameters: timer_id - id name of the timer
*
*     Returns: Tick_Type pending_time
*
* Description: returns the pending time of a timer
*
*
**********************************************************************/
Tick_Type TMR_Get_Pending_Time (TMR_ID_T timer_id)
{
    Tick_Type pending_time = 0;

    if (NUMBER_OF_TIMERS > timer_id)
    {
        Tick_Type os_time = OS_Time();

        if (os_time < tmr_array[timer_id].time_stamp)
        {
            pending_time= tmr_array[timer_id].time_stamp - os_time;
        }
    }

    return(pending_time);
}

/**********************************************************************
*
*    Function: null_action
*
*  Parameters: None
*
*     Returns: None
*
* Description: This function is needed as parameter for TMR_Start_Timer()
*              only if no other function is going to be called!
*
*
**********************************************************************/
void null_action (void)
{
   // nothing to do
}
#if 0
void rl_delay_without_schedule(Tick_Type ms)
{
    TickType_t currentTickCount = OS_Time();
    TickType_t delayTaicks = pdMS_TO_TICKS(ms);
    TickType_t TickCout = 0;
    while (delayTaicks + currentTickCount >= TickCout)
    {
        TickCout = OS_Time();
        Feed_Dog();                               // make sure we have full watchdog timeout
    }
}
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
