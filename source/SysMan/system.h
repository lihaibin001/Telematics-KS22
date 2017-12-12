/* $Header:  system.h   $*/
/***************************************************************************
   Title                    : SYSTEM.H

   Module Description       : This file contains application specific header
                              information for SYSTEM. 

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *************************************************************************/


/*----------------------------------------------------------------------
 *   Instructions for using this module if any:
 *
 *---------------------------------------------------------------------*/
#ifndef  SYSTEM_H
#define  SYSTEM_H

/**********************************************************************
 * Include files
 *********************************************************************/


/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
#define DISABLE_POWER_KEY_TIME     1000 //500ms
/*the following definition strongly depends on MAKE macro, should be careful
   remember to change this once MAKE macro is changed!!!!!!*/
/**********************************************************************/
#define  Sys_Error()
/*****************************************************************************
 * Personalization
 ****************************************************************************/

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef enum wake_up_src_f
{
   NO_FLAGS                 = 0x00,
   CAN_WAKE_UP              = 0x01,   /*CAN Bus wake up*/
   GSENSOR_WAKE_UP          = 0x02,   /*Gsensor wake up */
   RTC_WAKE_UP              = 0x04,    /*rtc wake up */
   LOW_BAT_WAKE_UP          = 0x08,    /*low batter wake up */
   RTC_DEEP_STDBY_WAKE_UP   = 0x10,    /*rtc deep standby wake up */
   ACC_WAKE_UP              = 0x20, 
   BMS_WAKE_UP              = 0x40,
} wake_up_src_f_t;

typedef union wake_up_src_flags
{
   uint8_t all;
   struct
   {
      uint8_t CAN_WakeUp_f			: 1;  // bit 0
      uint8_t Gsensor_WakeUp_f		: 1;  // bit 1
      uint8_t RTC_WakeUp_f			: 1;  // bit 2
      uint8_t Low_Bat_WakeUp_f		: 1;  // bit 3
      uint8_t RTC_DEEP_WakeUp_f		: 1;  // bit 4
      uint8_t ACC_WakeUp_f			: 1;
      uint8_t BMS_WakeUp_f          : 1;
      uint8_t reserve               : 1;
   }field;
} wake_up_src_flags_t;


typedef enum 
{
   LED_OFF				        = 0x00,
   LED_TOGGLE_FAST_BLINK_1		= 0x01,   /*toggle @ 160ms*/
   LED_TOGGLE_FAST_BLINK_2	    = 0x04,   /*toggle @ 160*3ms*/
   LED_TOGGLE_SLOW_BLINK_1		= 0x08,    
   LED_TOGGLE_SLOW_BLINK_2		= 0x20,    /*toggle @ 160*32ms*/
   LED_ON		= 0x11,    
} LED_Toggle_Period_T;

/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/
extern wake_up_src_flags_t sys_wakeup_src_flags;/* clear after before sleep !!! */
extern uint8_t ucResetFlag;

/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/
extern int8_t *SY_Swid (void);
extern int8_t *SY_Hwid (void);//TEST BY SCB
extern int8_t *SY_Sw_Date (void);
extern int8_t *SY_Sw_Version(void);
extern int8_t *SY_Protocol_Version(void);
extern void Restart(bool warm);

extern void __program_restart(void);

extern uint8_t Sys_Get_Wakeup_Src_Flags(void);
extern void Sys_Clear_Wakeup_Src_Flags(void);
extern void Sys_Clear_Standby_Req_Flag(void);
extern void Sys_Set_Standby_Req_Flag(void);
extern bool Sys_Get_Standby_Req_Flag(void);
extern void  Sys_Req_Enter_Deep_Standby(void);

extern void Check_Watchdog_Reset_Flag(void);

extern void LED_Set_Toggle_Period(uint8_t Led_id, LED_Toggle_Period_T period);
extern LED_Toggle_Period_T LED_Get_Toggle_Period(uint8_t Led_id);

extern void Sys_Set_CAN_Wakeup_Flag (void);
extern bool Sys_Is_CAN_Wakeup (void);
extern void Sys_Clear_CAN_Wakeup_Flag (void);

extern void Sys_Set_Gsensor_Wakeup_Flag(void);
extern bool Sys_Is_Gsensor_Wakeup(void);
extern void Sys_Clear_Gsensor_Wakeup_Flag (void);

extern void Sys_Set_2G_Wakeup_Flag (void);
extern bool Sys_Is_2G_Wakeup(void);
extern void Sys_Clear_2G_Wakeup_Flag (void);

extern void Sys_Set_RTC_Wakeup_Flag(void);
extern bool Sys_Is_RTC_Wakeup (void);
extern void Sys_Clear_RTC_Wakeup (void);

extern void Sys_Set_RTC_Deep_Wakeup_Flag(void);
extern bool Sys_Is_RTC_Deep_Wakeup (void);
extern void Sys_Clear_RTC_Deep_Wakeup (void);

/* 2017.7.7 lihaibin modify */
void Sys_Set_ACC_Wakeup_Flag (void);
void Sys_Clear_ACC_Wakeup_Flag (void);
bool Sys_Is_ACC_Wakeup (void);
void Sys_Set_BMS_Wakeup_Flag (void);
void Sys_Clear_BMS_Wakeup_Flag (void);
bool Sys_Is_BMS_Wakeup (void);
/* end 2017.7.7 lihaibin modify */

extern void Sys_Set_Low_Batt_Wakeup_Flag (void);
extern bool Sys_Is_Low_Batt_Wakeup (void);
extern void Sys_Clear_Low_Batt_Wakeup_Flag (void);

#endif
/*===========================================================================*\
 * File Revision History
 *===========================================================================

 **********************************************************************/

