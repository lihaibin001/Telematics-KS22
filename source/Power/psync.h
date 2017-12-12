/* $Header:   Psync.h $*/

#ifndef  PSYNC_H
#define  PSYNC_H
/**********************************************************************
 *  Title:   Psync.h
 *
 *  Description:  Interface information needed for PSYNC module
 *
 *  Author:  
 *
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "stdint.h"
#include "stdbool.h"
/**********************************************************************
 * Global Constant and Macro Definitions using #define                        
 *********************************************************************/

#define  EVT_NOP        0x00
#define  EVT_NOT_FOUND  -1
#define  ALL_MODES      0xFFFF

#define PS_Power_On()                       0
#define PS_Is_Play_Requested()              PS_Play_On()


#define BATT_MEAS_COUNT                     10 


#define PS_DEV_ON_TIME                      (PS_Get_Dev_On_Minutes()*60000)// cold start without CAN can awake 5min
#define PS_EVENT_ON_TIME                    (PS_Get_Event_On_Minutes()*60000)// Event Triggered can awake 2min
#define PS_ACC_ON_TIME                      (PS_Get_ACC_On_Minutes()*60000)// ACC on timer-mode can awake 30min

#define PS_STDBY_TIMEOUT_TIME               (60000)/*60s timeout, force sleep */
#define PS_STDBY_RETRY_TIME                 (3000)/*system will awake for  3s to check CAN network again */
#define PS_STDBY_TIME3                      (30000)/*system will awake for 30s before all data are uploaded*/
//#define PS_STDBY_TIME_FROM_ENG_ON                  (120000)/*system will awake for 120s after engine off */
#define PS_BATT_CHECK_DELAY_TIME            (60000)/*system will delay for  60s for checking batt volt*/
//#define PS_BUS_OFF_RETRY_TIME                  	(1000)/*system will STOP bus active for 1s, dont send any bus message*/

#define PS_DEEP_SLEEP_POLLING_TIME          (50)/*RTC will awake once per 50*6 = 300s to polling vehicle battery*/
#define PS_RTC_TICK                         (5)
//#define PS_RTC_DEEP_STANDBY_TICK               (300)
#define PS_RTC_DEEP_STANDBY_TICK            (86400)
#define PS_RTC_NET_FAIL_RETRY_STANDBY_TICK  (14400)

#define PS_TASK_REQUEUE_TIME                30
#define PS_MIN_AWAKE_TIME                   30000
#define PS_IDLE_TIME                        1500

#define PS_ACC_MONITOR_TIME                 3000//acc is off if no PID response received,diagnostic session is timeout also
//#define PS_ENG_ON_DEBOUNCE                   2

#define ps_put_user_on(x)                   PS_Set_Flag(USER_ON, x)
#define ps_put_awake_requested(x)           PS_Set_Flag(AWAKE_REQUESTED, x)
#define ps_put_idle_requested(x)            PS_Set_Flag(IDLE_REQUESTED, x)
#define ps_put_permanent_sleep(x)           PS_Set_Flag(PERMANENT_SLEEP, x)
#define ps_put_ap_sleep_requested(x)        PS_Set_Flag(AP_SLEEP_REQUESTED, x)
#define ps_put_ap_ctrl_pwr_req(x)           PS_Set_Flag(AP_CTRL_PWR_MODE, x)

#define ps_user_on()                        PS_Get_Flag(USER_ON)
#define ps_awake_requested()                PS_Get_Flag(AWAKE_REQUESTED)
#define ps_idle_requested()                 PS_Get_Flag(IDLE_REQUESTED)
#define ps_permanent_sleep()                PS_Get_Flag(PERMANENT_SLEEP)
#define ps_ap_sleep_requested()             PS_Get_Flag(AP_SLEEP_REQUESTED)
#define ps_ap_ctrl_pwr_requested()          PS_Get_Flag(AP_CTRL_PWR_MODE)
/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/
/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/
typedef uint8_t  PS_Current_State_T;

typedef uint8_t PS_Flags_T;

typedef enum PS_Flags_Enum_Tag
{
   USER_ON,
   AWAKE_REQUESTED,
   IDLE_REQUESTED,
   PERMANENT_SLEEP,
   AP_SLEEP_REQUESTED,
   AP_CTRL_PWR_MODE,  // AP get power mode control  
} PS_Flags_Enum_T;

/**
* PSYNC dynamic persistent storage data
*/
typedef struct PS_Data_Type_Tag
{
    PS_Flags_T            flags;
    PS_Current_State_T    current_state;
    PS_Current_State_T    last_state;
    bool                  logical_ignition;
    bool                  logical_engine;
    bool                  dev_on;
} ps_data_type;

/**
* PSYNC calibration persistent storage data
*/
typedef struct PS_Cal_Type_Tag
{
    uint16_t   dev_on_minutes;
    uint16_t   acc_on_minutes;
    uint16_t   event_on_minutes;
} PS_Cal_T;

/*===========================================================================*\
 * Custom Type Declarations
\*===========================================================================*/
typedef enum psync_internal_msg_type_tag
{
    PS_BEGIN_EVENTS = 0,
    PS_EVT_UPDATE,
    PS_EVT_GO_IDLE,
    PS_EVT_IDLE,                                             
    PS_EVT_AWAKE,
    PS_EVT_DEV_ON,
    PS_EVT_DEV_OFF,
    PS_EVT_IGN_ON,
    PS_EVT_IGN_OFF,
    PS_EVT_ENG_ON,
    PS_EVT_ENG_OFF,
    PS_EVT_DEV_ON_TMR_EXP,
    PS_NUM_EVENTS
} psync_internal_msg_type;

/* the precompiler produces an enum of the states */
#include "fsm_stat.h"
#include "ps_stt.h"

/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/
/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/


/*********************************************************************/
/**
 * Returns if the system shall be switched off.
 *
 * @return
 *    true  - conditions that the system
 *            should stay off are valid
 *    false - otherwise
 *
 * Reports combination of informations from CAN bus.
 */
/*********************************************************************/
bool PS_Is_Dev_Off_Requested(void);

void PS_Initialize(void);
bool PS_Is_Idle_Requested(void);
bool PS_Is_RTC_Requested(void);
void PS_Go_Idle(bool rtc_config);
void PS_Go_Awake(Tick_Type awake_time_in_ms);

bool PS_Idle(void);
bool PS_Awake(void);
bool PS_User_On(void);
bool PS_Play_On(void);
bool PS_Eng_On(void);
bool PS_Running(void);
bool PS_Full_System(void);
bool PS_Permanent_Sleep(void);
bool PS_Pending_Awake(void);

Tick_Type PS_Get_Idle_Time(void);

void PS_Set_Ignition(bool status);
void PS_Set_Dev_On(bool status);
void PS_Set_Engine_On(bool status);

void PS_Set_ON_Minutes(uint16_t minutes);
void PS_Restart_ON_Timer(uint16_t minutes);
uint16_t PS_Get_ON_Minutes(void);

void PS_Set_Idle(void);
void PS_Set_LSM_Awake(void);

bool PS_Logical_Ignition_On(void);
bool PS_Logical_Engine_On(void);

uint16_t PS_Get_Event_On_Minutes(void);
uint16_t PS_Get_ACC_On_Minutes(void);
uint16_t PS_Get_Dev_On_Minutes(void);
void PS_Set_Dev_On_Minutes(uint16_t minutes);
void PS_Set_ACC_On_Minutes(uint16_t minutes);
void PS_Set_Event_On_Minutes(uint16_t minutes);

PS_Current_State_T PS_Get_Current_State(void);
PS_Current_State_T PS_Get_Last_State(void);
void PS_Set_Current_State(PS_Current_State_T state);
void PS_Set_Last_State(PS_Current_State_T state);
void PS_Set_Flag(PS_Flags_Enum_T flag, bool status);
bool PS_Get_Flag(PS_Flags_Enum_T flag);
uint8_t ps_set_acc_out_off (void);

void PS_Start_Batt_Check_Timer (void);//workaround only
void PS_Check_Batt_Before_Sleep (void);
void PS_Send_Crash_Msg_On_Timer (void);
void PS_force_sleep_set(bool enable);
/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
 
#endif


