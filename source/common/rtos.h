/* $Header:   rtos.h  $*/
/**********************************************************************
 *   Title                      : rtos.h
 *
 *   Module Description         : provide OS system macro
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

#ifndef  RTOS_H
#define  RTOS_H 1

/*********************************************************************
 * Include User Header file
 *********************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h" /*OS specific header file*/
#include "FreeRTOSConfig.h" /*OS specific header file*/
#include "queue.h" /*OS specific header file*/
#include "Semphr.h" /*OS specific header file*/
#include "stdbool.h"
#include "timers.h"
/*********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define OS_TICKS_SEC            configTICK_RATE_HZ
#define OS_IDLE_TICKS_SEC       16/*not used,just for compile*/

#define RTI_TICK_MS             1
#define RTI_IDLE_TICK_MS        64

#define Enable_Interrupts()     portENABLE_INTERRUPTS()
#define Disable_Interrupts()    portDISABLE_INTERRUPTS()
//#define Enable_Interrupts()    __asm("CPSIE i");
//#define Disable_Interrupts()   __asm("CPSID i");

#define OS_Terminate_Task()     vTaskSuspend(NULL)/*delete call task itself*/

#define OS_Tick_Duration()      (1000000000/OS_TICKS_SEC)
                                             // Duration of system counter tick in nanoseconds
#define MSec_To_Ticks(msec)     (msec)//((((msec) * OS_TICKS_SEC)+999) / 1000U)
                                             // converts millseconds to ticks
#define Cold_Start()            Get_Cold_Start()

#define ONE_MINUTE_IN_TICKS     (MSec_To_Ticks(60000))
#define RESOURCE_TIMEOUT        ( MSec_To_Ticks(250) )   

#define  SY_Warm_Start()        Restart(true)    //trigger warm start
#define  SY_Cold_Start()        Restart(false)  //trigger cold start

//MISC
#define UNUSED_PARAM(param)               (void)(param)

/*********************************************************************
 * Allocate all stacks (in 2 bytes words)
 *********************************************************************/
#define TASK_RECORD_STACK_SIZE      700
#define TASK_PSYNC_STACK_SIZE       200
#define TASK_RELAYS_STACK_SIZE      200
#define TASK_PERIODIC_STACK_SIZE    200
#define TASK_DIAG_STACK_SIZE        200
#define TASK_IOT_STACK_SIZE         200
#define TASK_GPS_STACK_SIZE         200
#define TASK_SENSOR_STACK_SIZE      200

/*********************************************************************
 * Allocate all task message queue size
 *********************************************************************/
#define TASK_RECORD_QUEUE_SIZE 2
#define TASK_PSYNC_QUEUE_SIZE  4
#define TASK_RELAYS_QUEUE_SIZE  2
#define TASK_PERIODIC_QUEUE_SIZE  2
#define TASK_DIAG_QUEUE_SIZE 8
#define TASK_IOT_QUEUE_SIZE 8
#define TASK_GPS_QUEUE_SIZE 2
#define TASK_SENSOR_QUEUE_SIZE 2
/*********************************************************************
 * Allocate all task message size(all task msg are defined as 4Bytes message type) 
 *********************************************************************/
#define TASK_MSG_SIZE           sizeof(Message_Type)
#define TASK_RECORD_MSG_SIZE       TASK_MSG_SIZE
#define TASK_PSYNC_MSG_SIZE     TASK_MSG_SIZE
#define TASK_RELAYS_MSG_SIZE    TASK_MSG_SIZE
#define TASK_PERIODIC_MSG_SIZE  TASK_MSG_SIZE
#define TASK_DIAG_MSG_SIZE  TASK_MSG_SIZE
#define TASK_IOT_MSG_SIZE     TASK_MSG_SIZE
#define TASK_GPS_MSG_SIZE     TASK_MSG_SIZE
#define TASK_SENSOR_MSG_SIZE     TASK_MSG_SIZE

/*********************************************************************
 * Allocate all task priority(0->Lowest)
 *********************************************************************/

//The priority should be less than configMAX_PRIORITIES
#define RTOS_LOW_RIORITY 1
#define RTOS_NORMAL_RIORITY 2
#define RTOS_HIGH_RIORITY 3
#define RTOS_CRITICAL_RIORITY 4

#define TASK_IDLE_PRIORITY 0

#define TASK_RECORD_PRIORITY          RTOS_NORMAL_RIORITY
#define TASK_PSYNC_PRIORITY      RTOS_LOW_RIORITY/*should be lower priority*/
#define TASK_RELAYS_PRIORITY     RTOS_LOW_RIORITY/*should be lower priority*/
#define TASK_PERIODIC_PRIORITY   RTOS_HIGH_RIORITY
#define TASK_DIAG_PRIORITY   RTOS_NORMAL_RIORITY
#define TASK_IOT_PRIORITY      RTOS_HIGH_RIORITY
#define TASK_GPS_PRIORITY      RTOS_NORMAL_RIORITY
#define TASK_SENSOR_PRIORITY      RTOS_NORMAL_RIORITY

/*********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/
typedef enum Task_Type_Tag
{
   OS_PSYNC_TASK=0,
   OS_RELAY_TASK,
   OS_PERIODIC_TASK,
   OS_RECORD_TASK,
   OS_GPS_TASK,
   OS_IOT_TASK,
   OS_DIAG_TASK,
#if 0
   OS_SENSOR_TASK,
#endif
   OS_NUM_TASKS
} Task_Type;

typedef uint32_t Message_Type;                  // Generic message

typedef struct Data_Message_Parts_Tag           // Generic message with data 
{
    uint16_t   msg;
    int16_t    data; 
} Data_Message_Parts_T;

typedef union Data_Message_Tag
{
    Message_Type          all;
    Data_Message_Parts_T  parts;
} Data_Message_T; 

typedef portSTACK_TYPE OS_Stack_Type;                      // type of individual stack element

typedef struct OS_TCB_Init_Tag                  // task intializaiton block
{
    void (*Entry) (void *pvParameters);                       // function pointer for task entry
    uint16_t     Stack_Size;                   // size of stack
    uint8_t       Queue_Size;                     // number of mail slots
    uint8_t       Msg_Size;                    //size of message
    uint16_t     Priority;                   // priority of task
    const char * name;     //task name
} OS_TCB_Init_Type;

typedef enum  Event_Mask_Type_Tag           // Use bit enumerations x01,0x02,0x04,...
{
    OS_EVT_MUTE       = 0x00000001,
    OS_EVT_DMUTE      = 0x00000002,
    OS_EVT_RUNNING    = 0x00000004,
    OS_EVT_HI_SIP_RX = 0x00000080,       // HISIP data received and awaiting processing
    OS_EVT_ALL_VALID  = 0x000000FF        // mask for all valid event flags 
} Event_Mask_Type;

typedef enum Task_State_Tag                     // Task State definitions
{
    SUSPENDED      = 0,
    READY,
    RUNNING,
    OS_WAITING,
    INVALID_STATE
} Task_State_Type;

typedef enum  Resource_Type_Tag              // Index of resources
{
    RES_IOT = 0,   
    RES_RELAYS = 1,
    RES_PERIODIC,
    RES_CAN,
    RES_DMA_CHANNELS,
    RES_IIC_0,
    RES_SPIFLASH,//SPI flash
    OS_NUM_RESOURCES,
} Resource_Type;


typedef struct Semaphore_Init_Tag       /* Define type to initialize a semaphore */
{
    uint8_t init_count;          /* Initial Semaphore count */
    uint8_t max;                 /* maximum allowed value for Semaphore count */
} Semaphore_Init;

/*--------------------------------------------------------------------
 *     Alarm and Timer Defintions
 *-------------------------------------------------------------------*/
typedef portTickType Tick_Type;                     // System Timer Tick

/*--------------------------------------------------------------------
 *     Task Control Block Defintion
 *-------------------------------------------------------------------*/

/*********************************************************************
 * Global Variable extern Declarations
 *********************************************************************/
extern uint32_t OS_Cold_Start ;                      // false if last reset was coldstart
extern uint8_t  OS_ResF;       /* Cause of hardware reset */
extern uint8_t  OS_RamF;       /* RAM retention voltage detection */
/*********************************************************************
 * Function Prototypes
 *********************************************************************/
extern void Set_Cold_Start(void);
extern void Clear_Cold_Start(void);
extern void Set_warm_Start(void);
extern bool Get_Cold_Start(void);
extern void Init_Cold_Start_Flag(void);

extern void  vPortClearHeap( void );
extern void OS_Sleep(Tick_Type time);
extern void OS_Check_Cold_Start(void);
extern bool OS_Is_Started(void);
extern void OS_Clr_Start_Flag(void);
extern void SY_Reset(void);
//extern void vApplicationStackOverflowHook( void );
extern void vApplicationIdleHook( void );
extern void vApplicationTickHook(void);
extern unsigned char vApplicationGetTickBaseHook(void);
extern void vApplicationStartHook(void);
//extern void vApplicationDaemonTaskStartupHook( void );
/*--------------------------------------------------------------------
 *    Intialization routines
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
      Task management
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *       Resource Management (Semaphores)
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *       Alarms/Time delays
 *-------------------------------------------------------------------*/
/* Current system timer value 
 * An actual OS_Time function is used for micros that don't support atomic 32 bit integers
 * Otherwise, just directly access the global variable
 */ 
#ifdef USE_OS_TIME_FUNC
extern Tick_Type OS_Time(void);
#else // USE_OS_TIME_FUNC
#define OS_Time()             (xTaskGetTickCountFromISR())
#endif // USE_OS_TIME_FUNC

/*--------------------------------------------------------------------
 *       Messages
 *-------------------------------------------------------------------*/

/*--------------------------------------------------------------------
 *     Events - Non-RTOS - Events are owned at system level and are not separate by
 *     task;
 *-------------------------------------------------------------------*/

#endif/*RTOS_H*/

/*===========================================================================*\
 * File Revision History (top to bottom: first revision to last revision)
 *===========================================================================
 *
 *********************************************************************/
