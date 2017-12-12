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
/* $Header:   relays.c  $*/
/**********************************************************************
   Title                    : relays.c

   Module Description       : This is the standard code file for relays.

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/* Dependent "compile.cmd"                                           */
/*********************************************************************/
#include    "relays.h"
//#include  "lowpower.h"
//#include  "delay.h"
#include    "system.h"
#include	"standard.h"
#include    "GPRS.h"
#include	"TelmProtocol.h"
//#include    "crc_ccitt.h"
#include    "timer.h"
//#define USE_DEBUG
#include    "Debug.h"
#include "ACC_Detect.h"
#include "BMS_Detect.h"
#include "ATProtocol.h"
#include "spi_flash_freertos.h"
#include "adc.h"
/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/
/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
//Fixme
//#define Pwr_Fail_Is_Reset_Condition() false

#define rl_ignition_to_awake rl_play_to_awake

#define rl_write_timer(x, y)      (rl_timer[x] = y)
#define rl_read_timer(x)          (rl_timer[x])
#define rl_timer_running(x)       (rl_timer[x] > OS_Time())

#define rl_set_current_state(state)    (rl_data = state)
#define rl_get_current_state()         (rl_data)

#define rl_remain_idle(x)                 ((x > 0) ? (OS_Time() <= x) : 1)

#define rl_wait_finish_awake_sequence_hook()             
#define rl_regulator_outputs_ok()	     true
#define rl_awake_to_idle_hook()
#define rl_idle_to_awake_hook()
#define rl_awake_to_play_hook()                                       
#define rl_wait_to_go_idle_hook()                                      
#define rl_other_task_cleanup_complete() true

#define partial_terminating_tasks         all_terminating_tasks
#define ALL_TERMINATING_TASKS             (Num_Elems(all_terminating_tasks))    

#define RL_GPRS_START_DELAY (2000)

#define MIN_TO_TICKS    (60000)
#define MIN_TO_SEC    (60)
#define SEC_TO_TICKS    (1000)

// time in seconds
#define ENV_TEST_SLEEP_TIME (1200)

// time in ms
#define ENV_TEST_WAKE_TIME (60000)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

typedef enum rl_current_state_tag
{
   RL_CS_IDLE,
   RL_CS_AWAKE,
   RL_CS_PLAY,
   MAX_RL_CURRENT_STATES
} rl_current_state_t;

enum
{
   AWAKE_DELAY_TIMER,
   REG_OFF_DELAY_TIMER,
   WAIT_TO_STOP_TIMER,
   PF_DELAY_TIMER,
   NUM_RL_TIMERS
};

/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/
//user define
static bool rl_wake_up(void);
static void rl_check (void);
static void rl_emergency_shutdown(void);
//building block
static void rl_idle_to_awake(void);
static void rl_play_to_awake(void);
static void rl_cs_awake(void);
static void rl_cs_play(void);
static void rl_awake_to_idle(void);
static void rl_cs_idle(void);
static void rl_wait_for_tasks_to_suspend(void);
static void rl_check_wake_up(void);
static void rl_low_voltage (void);
static uint8_t rl_get_low_batt_sleep(void);
/*===========================================================================*\
 * Global and Const Variable Defining Definitions / Initializations
\*===========================================================================*/

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static rl_current_state_t rl_data ;
static bool rl_wake_up_requested;
static Tick_Type rl_timer[NUM_RL_TIMERS];
static bool rl_is_amp_muted;
static const void_fptr rl_current_state[MAX_RL_CURRENT_STATES] = 
{
   rl_cs_idle,
   rl_cs_awake,
   rl_cs_play
};
__no_init uint32_t rtc_timeout;
__no_init bool can_interrupt_disable;
__no_init uint8_t low_batt_sleep;
/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/
/*********************************************************************/
/* Global Function Definitions                                       */
/*********************************************************************/
/**********************************************************************
*    Function: rl_not_awake_IO
*
*  Parameters: none
*
*     Returns: none
*
* Description: This routine handles any specific action in 
*              preparation for I/O idle configuration.
*
**********************************************************************/
static void rl_not_awake_IO(void)
{
    /* set all unsed pin as input */
    const gpio_pin_config_t gpio_input_config = {kGPIO_DigitalInput, 0,};
    PORT_SetPinMux(PORTA, 0, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 0, &gpio_input_config);   
    PORT_SetPinMux(PORTA, 1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 1, &gpio_input_config); 
    PORT_SetPinMux(PORTA, 2, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 2, &gpio_input_config);
    //PORT_SetPinMux(PORTA, 3, kPORT_MuxAsGpio);
    //GPIO_PinInit(GPIOA, 3, &gpio_input_config);
    PORT_SetPinMux(PORTA, 4, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 4, &gpio_input_config);   
    PORT_SetPinMux(PORTA, 12, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 12, &gpio_input_config);
    PORT_SetPinMux(PORTA, 13, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOA, 13, &gpio_input_config);
    
    PORT_SetPinMux(PORTB, 0, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 0, &gpio_input_config);   
    PORT_SetPinMux(PORTB, 1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 1, &gpio_input_config); 
    PORT_SetPinMux(PORTB, 17, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 17, &gpio_input_config);
    PORT_SetPinMux(PORTB, 18, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 18, &gpio_input_config);
    PORT_SetPinMux(PORTB, 19, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOB, 19, &gpio_input_config);    
 
    PORT_SetPinMux(PORTC, 1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 1, &gpio_input_config);   
    PORT_SetPinMux(PORTC, 2, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 2, &gpio_input_config);
    PORT_SetPinMux(PORTC, 3, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 3, &gpio_input_config);   
    PORT_SetPinMux(PORTC, 4, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 4, &gpio_input_config); 
    PORT_SetPinMux(PORTC, 5, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 5, &gpio_input_config);
    PORT_SetPinMux(PORTC, 7, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 7, &gpio_input_config);
    PORT_SetPinMux(PORTC, 10, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 10, &gpio_input_config);
    PORT_SetPinMux(PORTC, 11, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOC, 11, &gpio_input_config);  
    
    PORT_SetPinMux(PORTD, 0, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 0, &gpio_input_config);   
    PORT_SetPinMux(PORTD, 1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 1, &gpio_input_config); 
    PORT_SetPinMux(PORTD, 2, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 2, &gpio_input_config);
    PORT_SetPinMux(PORTC, 3, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 3, &gpio_input_config);
    PORT_SetPinMux(PORTD, 4, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 4, &gpio_input_config);  
    PORT_SetPinMux(PORTD, 5, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 5, &gpio_input_config);   
    PORT_SetPinMux(PORTD, 6, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOD, 6, &gpio_input_config); 
    
    
    PORT_SetPinMux(PORTE, 0, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOE, 0, &gpio_input_config);
    PORT_SetPinMux(PORTE, 1, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOE, 1, &gpio_input_config);
}

/**********************************************************************
 *    Function: Relays_Task
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Entry point for the relays task.
 *
 *********************************************************************/
void Relays_Task( void *pvParameters )
{
    rl_current_state_t old_current_state;
    Config_t default_config;
    rl_idle_to_awake();
    Load_Param();
    Get_config_data(&default_config);
    for(;;)
    {
        OS_Wait_Resource(RES_RELAYS, MAX_RL_WAIT_TIME);  /* wait for semaphore or timeout */

        /* do all other checks first */
        rl_check();

        do
        {
            if (MAX_RL_CURRENT_STATES > rl_get_current_state())
            {
                old_current_state = rl_get_current_state();
                (*rl_current_state[rl_get_current_state()])();
            }
            else
            {
                /* current state index is currupted, fix it! */
                old_current_state = MAX_RL_CURRENT_STATES;/* force state machine to run again */
            }
        } while (old_current_state != rl_get_current_state());
    }
}
/**********************************************************************
*
*    Function: RL_Set_Pwr_Fail_Detected
*
*  Parameters: status true/false
*
*     Returns: none
*
* Description: informs Relays that powerfail  voltage range is
*              entered or exited
*
**********************************************************************/
void RL_Set_Pwr_Fail_Detected(bool status)
{
    UNUSED_PARAM(status);

    /* trigger RELAYS task when powerfail status is updated */
    OS_Release_Resource(RES_RELAYS);
}

/**********************************************************************
 *    Function: rl_wake_up
 *  Parameters: none
 *     Returns: none
 * Description: Intended to contain any obd specific information which
 *              would cause the obd to wake up (such as the SOS/CAN Bus
 *              waking up).
 **********************************************************************/

static bool rl_wake_up(void)
{
    //PMS_SetWakeupConfig(kAPP_PowerModeVlls0);
    return true;
}

static void RL_Start_GPRS(void)
{
    //rl_delay_without_schedule(100);
    {
        OS_Activate_Task(OS_IOT_TASK); 	
    }
}


static void RL_Start_GPS(void)
{
    /**********GPS start sequence*************/
    /*Enable PB.00 MCU_3V3-GPS_EN*/
    //IO_3V3_GPS_EN_OUT(Bit_SET);
    /*delay 25ms for power stable*/
    //rl_delay_without_schedule(25);
    {
        OS_Activate_Task(OS_GPS_TASK); 
    }
}
/**********************************************************************
 *    Function: RL_Begin_Awake_Sequence
 *  Parameters: none
 *     Returns: none
 * Description: Performs initial steps to move tbox to awake state - 
 *              namely, turns the regulator on
 *********************************************************************/

void RL_Begin_Awake_Sequence(void)
{
    //wait battery to stable
    rl_delay_without_schedule(100);
    if(!Pwr_Fail_Is_Reset_Condition() \
        || Pwr_Fail_Get_Int_Voltage() > INTER_BAT_CHARGE_THRESHOLD)
	{
       	RL_Start_GPS();
       	{
            //DEBUG(DEBUG_MEDIUM,"[RELAYS]Start GPRS 2\n\r");
            //Sys_Clear_2G_Wakeup_Flag();
            TMR_Start_Timer(GPRS_START_TIMER, RL_GPRS_START_DELAY, RL_Start_GPRS);	
       	}
        OS_Activate_Task(OS_DIAG_TASK); 
    }
    else
    {
        DEBUG(DEBUG_HIGH,"[RELAYS] Power low, going to sleep!\n\r");
        rl_awake_to_idle();
    }
}

/**********************************************************************
 *    Function: RL_Finish_Awake_Sequence
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Performs any further steps to move tbox to awake state -
 *              namely, wait for the regulator to power up
 *
 *********************************************************************/

void RL_Finish_Awake_Sequence(void)
{
    /* the AWAKE_DELAY_TIMER was loaded in RL_Begin_Awake_Sequence; */
    /* wait until it expires before proceeding but do not give up   */
    rl_write_timer(AWAKE_DELAY_TIMER, OS_Time() + REG_ON_TIME);
    while (rl_timer_running(AWAKE_DELAY_TIMER))
    {    
        /* do powerfail monitoring */
        if (Pwr_Fail_Is_Reset_Condition())
        {
           /* shut voltage off and go to idle */
            rl_emergency_shutdown();
        }
        else if(Pwr_Fail_AD_get_Voltage() < 650)
        {
            if(1 == rl_get_low_batt_sleep())//recover from low battery force sleep
            {
                if(false == Sys_Get_Standby_Req_Flag())
                {
                    RL_Force_Sleep();//Not yet, so force sleep again right now
                }
            }
        }
        else
        {
            rl_set_low_batt_sleep(0);//battery recovery, go ahead
        }
        Enable_Interrupts();                      // in case WTNCS is corrupt
        Feed_Dog();                               // make sure we have full watchdog timeout
    }
}

/*********************************************************************/
/* Local Function Definitions                                        */
/*********************************************************************/
/**********************************************************************
*    Function: rl_check
*  Parameters: none
*     Returns: void
* Description: this is a hook up function from the RL_Task,
*              call here all other functions that need to be executed every
*              task run, if not needed it can be removed by using a empty macro
**********************************************************************/
static void rl_check (void)
{
    /* check powerfail-reset: voltage range that allows reset */
    /* check restart request flag that allows reset           */
#if 0
    if (Pwr_Fail_Is_Reset_Condition())
    {
        /* shut voltage off and go to idle */
        rl_emergency_shutdown();        /* record the state before obd off when  power  became bad */
    }
    else if(Periodic_Get_Low_Volt_Cnt() > 20)//low volt for 20s in any state
    {
        DEBUG(DEBUG_HIGH,"[SYSTEM]:Battery low, Force Sleep!!!\n\r");

        OS_Sleep(15000);//15 seconds for sending battery low warning

        rl_set_low_batt_sleep(1);
        RL_Force_Sleep();
    }
    else if(PS_Play_On())
    {
        /* check for good battery voltage */
        if (rl_is_amp_muted)
        {
            /* check if battery voltage is good */
            if (Pwr_Fail_Is_Voltage_Good())
            {
                rl_is_amp_muted = false;
            }
        }
        else
        {
            /* check if battery voltage is not good */
            if (!Pwr_Fail_Is_Voltage_Good())
            {
                rl_is_amp_muted = true;
               //should inform that voltage is not good,tbd
            }
        }
    }
#endif
}

/**********************************************************************
 *    Function: rl_idle_to_awake
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Performs action to move obd from idle to awake
 *
 *********************************************************************/

static void rl_idle_to_awake(void)
{
    /* 2017.7.4 lihaibin modify  */
   //rl_finish_awake_sequence_hook();
    /*end  2017.7.4 lihaibin modify  */
   RL_Begin_Awake_Sequence();
   rl_set_current_state(RL_CS_AWAKE);
}

/**********************************************************************
 *    Function: rl_play_to_awake
 *  Parameters: none
 *     Returns: none
 * Description: Performs action to move obd from play to awake
 *********************************************************************/

static void rl_play_to_awake(void)
{
  rl_set_current_state(RL_CS_AWAKE);
}

/**********************************************************************
 *    Function: rl_cs_awake
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator and bridge support for awake state.
 *********************************************************************/
static void rl_cs_awake(void)
{
    if (!PS_Awake())
    {
        if(Pwr_Fail_Is_Mute_Condition())
        {
            if(!Sys_Get_Standby_Req_Flag())
            {
                Sys_Clear_Wakeup_Src_Flags();
                //Sys_Req_Enter_Deep_Standby();
            }
            else
            {
                Sys_Clear_Standby_Req_Flag();
            }
        }
        rl_awake_to_idle();
    }
    else /* refresh rl_cs_awake */
    {
    }
}


/**********************************************************************
 *    Function: rl_cs_play
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator and bridge support for play state.
 **********************************************************************/

static void rl_cs_play(void)
{
    if (!PS_Play_On())
    {
        rl_play_to_awake();
    }
    else /* refresh rl_cs_play */
    {
    }	   
}

/**********************************************************************
 *    Function: rl_awake_to_idle
 *  Parameters: none
 *     Returns: none
 * Description: Performs action to move obd from wake to idle
 *********************************************************************/
static void rl_awake_to_idle(void)
{
    Config_t config_data;
    uint8_t activate_status=Get_Activation_Status();

    Get_config_data(&config_data);
    Set_Config(config_data);
    DEBUG(DEBUG_HIGH,"[SYSTEM]:Sleeping...\n\r"); 
    {
        // set 2G module to sleep mode
        //GPRS_Module_GoSleep();

        OS_Sleep(1000);
    }
    // Save data to flash
    Save_Param();
    OS_Sleep(STBY_OFF_WAIT_TICKS);

    Sys_Clear_Wakeup_Src_Flags();
    Sys_Clear_Standby_Req_Flag();

    rl_awake_to_idle_hook();

    rl_wait_for_tasks_to_suspend();

    OS_Sleep(STBY_OFF_WAIT_TICKS);

    // Set sleep wakeup time
    //RTC_SetAlarmCount(RTC_GetCounter() + (uint32_t)config_data.structData.sleep_time * 60);
//    RTC_SetAlarmCount(RTC_GetCounter() + 60);
    RTC_EnableInterupt();
    
    OS_Clr_Start_Flag(); 
    Sys_Clear_Wakeup_Src_Flags();
    /* 2017.7.11 lihaibin modify */
    spi_flash_deepPwrDown();
//    vATProt_Power_Off();
    IO_3V3_GPS_EN_OUT(Bit_RESET);
    IO_CHARGE_CTL(Bit_RESET);
    IO_LED1_CTL_OUT(Bit_SET);
    IO_LED2_CTL_OUT(Bit_SET);
    IO_CAN_STANBY_OUT(Bit_RESET);
    IO_CAN_ENABLE_OUT(Bit_RESET);
    //ADC_Deinitialize();
    /* end 2017.7.11 lihaibin modify */    
    rl_not_awake_IO();
    rl_set_current_state(RL_CS_IDLE);
    
    if(BMS_GetCurrentSignalLevel() || ACC_GetCurrentSignalLevel())
    {
        NVIC_SystemReset();
        return ;
    }
    Micro_Go_Low_Power();
}

/**********************************************************************
 *    Function: rl_cs_idle
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator support for idle state.
 *********************************************************************/
static void rl_cs_idle(void)
{
    Tick_Type idle_time = 0;
    while(rl_remain_idle(idle_time))
    {
#if 0
        /*executes all configurations needed for idle mode*/
        rl_check_wake_up();
        rl_configure_idle_hook();
        Feed_Dog();
        Enable_Interrupts();
        rl_configure_micro_for_idle();
#endif
    }
}

/**********************************************************************
*
*    Function: rl_delay
*
*  Parameters: none
*
*     Returns: none
*
* Description: indicate that power supply for awake AP is ready
*
*********************************************************************/
void rl_delay_without_schedule(Tick_Type ms)
{
    rl_write_timer(AWAKE_DELAY_TIMER, OS_Time() + ms);

    while (rl_timer_running(AWAKE_DELAY_TIMER))
    {
        Enable_Interrupts();                      // in case WTNCS is corrupt
        Feed_Dog();                               // make sure we have full watchdog timeout
    }
}

/**********************************************************************
*    Function: RL_In_Idle
*  Parameters: none
*     Returns: whether or not relays is in the idle state
* Description: Accessor to determine if relays is in the idle state
*
*********************************************************************/
bool RL_In_Idle (void)
{
    return(RL_CS_IDLE == rl_get_current_state());
}

/**********************************************************************
*    Function: RL_In_Play
*  Parameters: none
*     Returns: whether or not relays is in the play state
* Description: Accessor to determine if relays is in the play state
*
*********************************************************************/
bool RL_In_Play (void)
{
    return(RL_CS_PLAY == rl_get_current_state());
}

/**********************************************************************
 *    Function: rl_wait_for_tasks_to_suspend
 *  Parameters: none
 *     Returns: none
 * Description: Sends out GO_IDLE message and waits for appropriate tasks
 *              to receive message and suspend.  Has a timeout value and
 *              will return E_OK or E_TIMEOUT depending on whether all
 *              tasks suspended in the appropriate time or not.
 *
 **********************************************************************/
static void rl_wait_for_tasks_to_suspend(void)
{
    int     awake_to_idle_increments = 0;
    Status_Type suspend_error;

    do
    {
        rl_check_wake_up();

        /* We need to make sure all tasks which are supposed to be suspended are indeed   */
        /* suspended before proceeding to the idle loop.  We also need to make sure we do */
        /* not hold up going to idle for tasks which do not suspend (e.g., OS_IDLE_TASK,  */
        /* OS_RELAYS).  Only those tasks with a while (PS_Running()) loop will suspend.   */

        suspend_error = OS_E_OK;  /* assume we are cleaned up until proven otherwise in below test */
        if (!rl_other_task_cleanup_complete())
        {
            suspend_error = OS_E_TIMEOUT;
        }
        if (OS_E_TIMEOUT == suspend_error)
        {
            if (!(awake_to_idle_increments % NUM_INCREMENTS_TO_RESEND_IDLE_MSG))
            {

            }
            OS_Sleep(AWAKE_TO_IDLE_INCREMENT_TICKS);
        }//50ms * 10 = 500ms timer for all_terminating_tasks(current is 3g task) to finish current cycle(OTA packet decoding? dataflash writing? ).
    
    } while ((OS_E_TIMEOUT == suspend_error) && (++awake_to_idle_increments < NUM_INCREMENTS_TO_FORCE_IDLE));
    if (!awake_to_idle_increments)
    {
        /* every task which should have suspended already was, send out one GO_IDLE     */
        /* for tasks which do not suspend so that they are aware that we are idle       */
        /* (otherwise would have been notified non-suspending tasks in above for loop). */
    }
}

/**********************************************************************
 *    Function: rl_check_wake_up
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: This function determines whether or not the device 
 *              should wake up from an idle state.  If the obd needs
 *              to wake up, this function will call Restart() and NOT
 *              return to the caller.
 *
 *********************************************************************/

static void rl_check_wake_up(void)
{
    if (rl_wake_up())
    {
        //SY_Warm_Start();
    }
}
 

/**********************************************************************
*
*    Function: rl_low_voltage
*
*  Parameters: none
*
*     Returns: none
*
* Description: Performs action for staying in low voltage mode.
*              As long as 3V3 Main is not stable, we stay in while loop.
*              After 1 sec, system is immediatelly shut down.
*              When leaving power-fail state, Cold Start will be
*              performed.
*
*********************************************************************/
static void rl_low_voltage (void)
{

}
/**********************************************************************
*
*    Function: rl_full_awake_on
*
*  Parameters: none
*
*     Returns: none
*
* Description: indicate that power supply for awake AP is ready
*
*********************************************************************/
bool rl_full_awake_on(void)
{
    return(rl_data > RL_CS_IDLE);
}
/**********************************************************************
*
*    Function: RL_Enter_Powerfail
*
*  Parameters: none
*
*     Returns: none
*
* Description: Set the relays to the powerfail state
*
*
*********************************************************************/
extern void RL_Enter_Powerfail(bool low_voltage)
{
    if(low_voltage)
    {
        rl_low_voltage();
    }
    else
    {

    }
}

/**********************************************************************
*
*    Function: rl_emergency_shutdown
*
*  Parameters: none
*
*     Returns: none
*
* Description: Performs action for staying in powerfail reset mode.
*             As long as the battery sense line is low (Vbatt < 6.0V)
*             we stay in a while loop. 
*             If the battery sense line goes up again (Vbatt > 6.5V)
*             a warm start is done.
*
*********************************************************************/
static void rl_emergency_shutdown(void)
{
    if (PS_IDLE==PS_Get_Current_State())
        return;
    PS_Set_Idle();
//    rl_awake_to_idle();

    rl_write_timer(PF_DELAY_TIMER, OS_Time() + (MSec_To_Ticks(180000)));
    /*wait 5mins for recovery, then give up and go idle*/
    while (rl_timer_running(PF_DELAY_TIMER))
    {
        Feed_Dog();                               // make sure we have full watchdog timeout
        Enable_Interrupts();                      // in case WTNCS is corrupt
        Halt();
    }
}
/**********************************************************************
*
*    Function: RL_Force_Sleep
*
*  Parameters: none
*
*     Returns: none
*
* Description:Force sleep in case CAN bus is always active,disable CAN wake up interrupt
*
*********************************************************************/
void RL_Force_Sleep(void)
{
    Sys_Clear_Wakeup_Src_Flags();
    //Sys_Req_Enter_Deep_Standby();   
    //Micro_Go_Low_Power();	
    rl_awake_to_idle();
}



void rl_rtc_disable(void)
{
  
}

#if 0
/**
  * @brief  Configures EXTI Lines.
  * @param  None
  * @retval None
  */
static void rl_exti_config(void)
{
}
#endif 
/**
  * @brief  Get rtc_timeout value.
  * @param  None
  * @retval None
  */
uint32_t rl_get_rtc_timeout(void)
{
    return rtc_timeout;
}

/**
  * @brief  Set rtc_timeout value.
  * @param  None
  * @retval None
  */
void rl_set_rtc_timeout(uint32_t value)
{
    rtc_timeout = value;
}

/**
  * @brief  Get low_batt_sleep value.
  * @param  None
  * @retval None
  */
static uint8_t rl_get_low_batt_sleep(void)
{
    return low_batt_sleep;
}

/**
  * @brief  Set low_batt_sleep value.
  * @param  None
  * @retval None
  */
void rl_set_low_batt_sleep(uint8_t value)
{
    low_batt_sleep = value;
}

void rl_reset()
{
    Save_Param();
    SY_Cold_Start();
}

#if 0
static void rl_go_sleep(void)
{
    DEBUG(DEBUG_MEDIUM,"[RELAYS] Go to Sleep\n\r");
    rl_emergency_shutdown();
}
#endif
/**********************************************************************
 *                                                                     
 * REVISION RECORDS                                                    
 *                                                                     
 *********************************************************************/
/*********************************************************************/
/*
 *
 *********************************************************************/
