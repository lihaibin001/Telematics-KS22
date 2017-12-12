/* $Header:   rtos_ext.h  $*/
/**********************************************************************
 *   Title                      : rtos_ext.h
 *
 *   Module Description         : provide os-related external function
 *
 *   Author                     : 
 *
 *   Created                    : 
 *
 *********************************************************************/
/*---------------------------------------------------------------------
 *   Instructions for using this module if any:
 *
 *-------------------------------------------------------------------*/

#ifndef  RTOS_EXT_H
#define  RTOS_EXT_H 1

/*********************************************************************
 * Include User Header file
 *********************************************************************/
#include "rtos.h"
#include "definiti.h"
/*********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/

/*********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/
/*--------------------------------------------------------------------
 *     Alarm and Timer Defintions
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *     Task Control Block Defintion
 *-------------------------------------------------------------------*/

/*********************************************************************
 * Global Variable extern Declarations
 *********************************************************************/

/*********************************************************************
 * Function Prototypes
 *********************************************************************/
extern void OS_Init(void);
extern void OS_Start(void);
extern void OS_Suspend_Task(Task_Type pTasknum);
extern Status_Type OS_Activate_Task(Task_Type task_id);
extern Status_Type OS_Send_Message(Task_Type task_id, Message_Type msg);
extern Status_Type OS_Send_MessageISR(Task_Type task_id, Message_Type msg);
extern Status_Type OS_Receive_Message(Task_Type task_id, Message_Type *msg);
extern Status_Type OS_Wait_Message(Task_Type task_id,Message_Type * msg, Tick_Type timeout);
extern Status_Type   OS_Get_Resource(Resource_Type resid);
extern Status_Type   OS_Release_Resource(Resource_Type resid);
extern Status_Type   OS_Release_Resource_From_ISR(Resource_Type resid, bool from_tick_isr);
extern Status_Type   OS_Wait_Resource(Resource_Type resid, Tick_Type timeout);
extern Task_State_Type OS_Task_State(Task_Type task_id);
extern Message_Type Build_Message(uint16_t id, int16_t data);
/*--------------------------------------------------------------------
 *    prototype all task routines (to avoid including all headers)
 *-------------------------------------------------------------------*/
extern void CAN_Task( void *pvParameters );
extern void Psync_Task( void *pvParameters );
extern void Relays_Task( void *pvParameters );
extern void Periodic_Task(void *pvParameters);
extern void Audio_Task(void *pvParameters);
extern void v3G_Task(void *pvParameters);
extern void GPS_Task(void *pvParameters);

/*--------------------------------------------------------------------
      Task management
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *       Resource Management (Semaphores)
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *       Alarms/Time delays
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *       Messages
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *     Events - Non-RTOS - Events are owned at system level and are not separate by
 *     task;
 *-------------------------------------------------------------------*/

#endif/*RTOS_EXT_H*/

/*===========================================================================*\
 * File Revision History (top to bottom: first revision to last revision)
 *===========================================================================
 *
 *********************************************************************/
