/* $Header:   rtos.c    $*/
/*===========================================================================*\
 * FILE: rtos.c
 *===========================================================================
 * 
 * DESCRIPTION:
 *    Operating System Kernel Abstract C file
 *
 * AUTHOR:
 *    
 *
\*===========================================================================*/

/*===========================================================================*\
 * Include Header files 
\*===========================================================================*/

#include "standard.h"
#include "debug.h"
#include "wdog.h"
/*===========================================================================*\
 * Constant and Macro Definitions using #define
\*===========================================================================*/

static QueueHandle_t pxTaskQueue[OS_NUM_TASKS];//message queue handle
static TaskHandle_t  pxTaskHandle[OS_NUM_TASKS];//task handle
static SemaphoreHandle_t  pxResources[OS_NUM_RESOURCES];//Counting Semaphore for system resource

static OS_TCB_Init_Type const os_TCB_init[OS_NUM_TASKS] =                                            
{
    /*Entry          stack size                       msg queue size                  message size        priority       task name, */
   {Psync_Task,     TASK_PSYNC_STACK_SIZE,      TASK_PSYNC_QUEUE_SIZE,      TASK_PSYNC_MSG_SIZE,        TASK_PSYNC_PRIORITY,    "psync task"},
   {Relays_Task,    TASK_RELAYS_STACK_SIZE,     TASK_RELAYS_QUEUE_SIZE,     TASK_RELAYS_MSG_SIZE,       TASK_RELAYS_PRIORITY,   "relays task"},
   {Periodic_Task,  TASK_PERIODIC_STACK_SIZE,   TASK_PERIODIC_QUEUE_SIZE,   TASK_PERIODIC_MSG_SIZE,     TASK_PERIODIC_PRIORITY, "peroidic task"},
   {Record_Task,    TASK_RECORD_STACK_SIZE,     TASK_RECORD_QUEUE_SIZE,     TASK_RECORD_MSG_SIZE,       TASK_RECORD_PRIORITY,   "record task"},
   {GPS_Task,       TASK_GPS_STACK_SIZE,        TASK_GPS_QUEUE_SIZE,        TASK_GPS_MSG_SIZE,          TASK_GPS_PRIORITY,      "GPS task"},
   {IOT_Task,       TASK_IOT_STACK_SIZE,        TASK_IOT_QUEUE_SIZE,        TASK_IOT_MSG_SIZE,          TASK_IOT_PRIORITY,      "IoT task"},
   {Diag_Task,      TASK_DIAG_STACK_SIZE,       TASK_DIAG_QUEUE_SIZE,       TASK_DIAG_MSG_SIZE,         TASK_DIAG_PRIORITY,     "diag task"},
#if 0
   {Sensor_Task,   TASK_SENSOR_STACK_SIZE,   TASK_SENSOR_QUEUE_SIZE, TASK_SENSOR_MSG_SIZE,  TASK_SENSOR_PRIORITY, "Sensor task"},
#endif
};
//---------------------------------------------------------------------
//  Set up Resources initialization data 
//---------------------------------------------------------------------
static Semaphore_Init const resource_init[OS_NUM_RESOURCES] =
{
   {1,1},                                       // RES_IOT  (one available, and maximum of one)
   {1,1},                                       // RES_RELAYS  (one available, and maximum of one)
   {0,10},                                    //RES_PERIODIC(zero available, and maximum of ten)
   {0,1},                                       //RES_CAN  (zero available, and maximum of one)
   {4,4},                                       // RES_DMA_CHANNELS  (four available, and maximum of four)
   {1,1},                                       //RES_IIC_0  (one available, and maximum of one)   {1,1},   
   {1,1},                           // RES_SPIFLASH  (one available, and maximum of one)
};

/*===========================================================================*\
 * Enumerations and Structures and Typedefs
\*===========================================================================*/
typedef struct os_tcb_Tag
{
   OS_Stack_Type *stack_ptr;    /* Pointer to current top of stack */
   Event_Mask_Type event_mask;  /* Event mask */
   Tick_Type alarm_count;       /* Alarm for timeouts */
   uint16_t alarm_period;       /* alarm delta */
   int8_t state;                /* (Task_State_Type) Task status (SUSPENDED,WAITING,READY,RUNNING,WAITING_MSG) */
} OS_TCB;
/*--------------------------------------------------------------------
 *    Resource Type
 *------------------------------------------------------------------*/


/*===========================================================================*\
 * Global and Const Variable Defining Definitions / Initializations
\*===========================================================================*/
__no_init  uint32_t OS_Cold_Start; //                    // false if last reset was coldstart
uint8_t  OS_ResF;       /* Cause of hardware reset */
uint8_t  OS_RamF;       /* RAM retention voltage detection */
//PRIVILEGED_DATA bool          os_started;                  /* Set true when OS task switching has been enabled */
bool          os_started;                  /* Set true when OS task switching has been enabled */
/*===========================================================================*\
 * Static Variables and Const Variables With File Level Scope
\*===========================================================================*/

/*===========================================================================*\
 * Function Prototypes for Private Functions with File Level Scope
\*===========================================================================*/
static void OS_Create_Task(const OS_TCB_Init_Type *task_info, xQueueHandle *pQuehandle,xTaskHandle *pTaskhandle);
static Status_Type os_get_resource(Resource_Type resid);
static void Idle_Task (void );
/*===========================================================================*\
 * Add User defined functions
\*===========================================================================*/
void SY_Reset(void)
{
     OS_Sleep(100);//100ms delay?
}

extern void Init_Cold_Start_Flag(void)
{
    if(OS_Cold_Start != 0x55)
        OS_Cold_Start = true;
    else
        OS_Cold_Start = false;//otherwise it must be warm start
}

extern void Set_Cold_Start(void)
{
    OS_Cold_Start = true;
}

extern void Clear_Cold_Start(void)
{
    OS_Cold_Start = false;
}
extern void Set_warm_Start(void)
{
    OS_Cold_Start = 0x55;
}

extern bool Get_Cold_Start(void)
{
    return OS_Cold_Start ;
}

/**********************************************************************
 *    Function: OS_Check_Cold_Start
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 *    Modifies: Cold_Start
 *
 * Description: Set Cold_Start flag if any previous stack was corrupted
 *
 *********************************************************************/
void OS_Check_Cold_Start(void)
{
}


/*===========================================================================*\
 * Function Definitions
\*===========================================================================*/
/*===========================================================================*\
 *    Function: OS_Init
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 *
 * Description: Initializes all RTOS data structures at system startup
 *
\*===========================================================================*/
void OS_Init(void)
{
    Resource_Type res_id;

    for (res_id = (Resource_Type) 0; res_id < OS_NUM_RESOURCES; res_id++)
    {                            /* initialize all resources */
        pxResources[res_id] = xSemaphoreCreateCounting(resource_init[res_id].max,resource_init[res_id].init_count);
    }
}
/*===========================================================================*\
 *    Function: OS_Task_State
 *
 *  Parameters: 'task_id' task to return information on
 *
 *     Returns: task state
 *
 *    Modifies: none
 *
 * Description: Return 'task_id' state (SUSPENDED, READY, RUNNING,
 *                 OS_WAIT_ALARM, OS_WAIT_MSG, OS_WAIT_EVENT, OS_WAIT_RES)
 *
\*===========================================================================*/
Task_State_Type OS_Task_State(Task_Type task_id)
{
    eTaskState state = eTaskGetState(pxTaskHandle[task_id]);
    switch(state)
    {
        case eRunning:	/* A task is querying the state of itself, so must be running. */
            return RUNNING;
        case eReady:			/* The task being queried is in a read or pending ready list. */
            return READY;
        case eBlocked:		/* The task being queried is in the Blocked state. */
            break;
        case eSuspended:		/* The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out. */
            return(SUSPENDED);
        case eDeleted:		/* The task being queried has been deleted, but its TCB has not yet been freed. */
            break;
        case eInvalid:			/* Used as an 'invalid state' value. */ 
            return(INVALID_STATE);
        default:
            break;
    }
    return INVALID_STATE;
}/* GetTaskState */

/*===========================================================================*\
 *    Function: OS_Activate_Task
 *
 *  Parameters: 'task_id' id of task to activate.
 *
 *    Returns: E_OK if sucessful
 *             else E_OS_ID if invalid ID
 *
 *    Modifies: none
 *
 * Description: Starts the requested task from its initial entry position
 *              Note, the task MUST release all resources before calling this routine.
 *
\*===========================================================================*/
Status_Type OS_Activate_Task(Task_Type task_id)
{
    if (OS_NUM_TASKS > task_id)
    {
        {
            OS_Create_Task(&os_TCB_init[task_id], &pxTaskQueue[task_id],&pxTaskHandle[task_id]);
            /* Initialize task entry point and stack */
        }
        return (OS_E_OK);
    }
    else
    {
        return (OS_E_OS_ID);         /* Invalid task number */
    }
}
/*===========================================================================*\
 *    Function: OS_Create_Task
 * Description: Initialize new task
 *  Parameters: task_info is point to RTOS task control block
 *              pQuehandle is pointer to task message queue
 *              pTaskhandle is pointer to task handle
 *     Returns: None
\*===========================================================================*/
static void OS_Create_Task(const OS_TCB_Init_Type * task_info, xQueueHandle *pQuehandle,xTaskHandle *pTaskhandle)
{
    xQueueHandle xQueue;

    xQueue = xQueueCreate( task_info->Queue_Size, task_info->Msg_Size );/*create message queues for each task*/
    if((errQUEUE_FULL == xQueue)&&(task_info->Queue_Size > 0))
    {
        while(1);/*error, queue create failed*/
    }

    *pQuehandle = xQueue;

    /*create task with static prameters*/
    xTaskCreate( (pdTASK_CODE)(task_info->Entry) \
   	            , (char const *)task_info->name \
   	            , task_info->Stack_Size \
   	            , ( void * ) (*pQuehandle) \
   	            , task_info->Priority \
   	            , pTaskhandle );

    if(NULL == pTaskhandle)
    {
        while(1);/*error, task create failed,just for debug*/
    }
}
/*===========================================================================*\
 *    Function: OS_Suspend_Task
 *  Parameters: 
 *     Returns: None
\*===========================================================================*/
void OS_Suspend_Task(Task_Type pTasknum)
{
    if(pxTaskHandle[pTasknum] !=NULL)  //if the task is active
    {
        vTaskSuspend(pxTaskHandle[pTasknum]);
    }
}
/*===========================================================================*\
 *    Function: Idle_Task
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 *    Modifies: none
 *
 * Description: This function is executed when no other tasks are ready to run.
 *              It is an external user defined routine called by OS idle task
 *
\*===========================================================================*/
static void Idle_Task (void )
{
    Feed_Dog();          /* update watchdog */
    /* if idle task starved for more than 10 seconds, will reset */
    Enable_Interrupts();      /* enable interrupts */
    Halt();                   /* pause CPU until next interrupt */
    /*an system tick interrupt should be executed*/ 
}
/*===========================================================================*\
 *    Function: OS_Start
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 *    Modifies: os_cur_task_id
 *              os_cur_task_mask
 *              os_cur_task
 *
 * Description: Start multitasking. This routine should never return to caller.
 *
\*===========================================================================*/
void OS_Start(void)
{
    /* Start the scheduler. */
    vTaskStartScheduler();
    /* branches to start of task */
}
/*===========================================================================*\
 *    Function: OS_Send_Message
 *
 *  Parameters: 'task_id' is task to send message to
 *              'msg'   pointer value to write to mailbox
 *
 *     Returns: None
 *
 *    Modifies: task's mailbox
 *              OS_Error_Ctr
 *
 *  Description: if mailbox empty then place message in it.
 *            if task pending on mailbox, queue it.
 *
\*===========================================================================*/
Status_Type OS_Send_Message(Task_Type task_id, Message_Type msg)
{
    Message_Type xmsg = msg;

    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxTaskQueue[task_id]== NULL)
    {
        return (OS_E_ERROR);
    }
    if(errQUEUE_FULL == xQueueSend(pxTaskQueue[task_id], ( void * ) &xmsg, 0))
    {
        return (OS_E_ERROR);       /* Queue full */
    }
    else
    {
        return (OS_E_OK);     /* else return there was no message */
    }
}

/*===========================================================================*\
 *    Function: OS_Send_MessageISR
 *
 *  Parameters: 'task_id' is task to send message to
 *              'msg'   pointer value to write to mailbox
 *
 *     Returns: None
 *
 *    Modifies: task's mailbox
 *              OS_Error_Ctr
 *
 *  Description: if mailbox empty then place message in it.
 *            if task pending on mailbox, queue it.
 *
\*===========================================================================*/
Status_Type OS_Send_MessageISR(Task_Type task_id, Message_Type msg)
{
    Message_Type xmsg = msg;

    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxTaskQueue[task_id]== NULL)
    {
        return (OS_E_ERROR);
    }
    if(errQUEUE_FULL == xQueueSendFromISR(pxTaskQueue[task_id], ( void * ) &xmsg, 0))
    {
        return (OS_E_ERROR);       /* Queue full */
    }
    else
    {
        return (OS_E_OK);     /* else return there was no message */
    }
}

/*===========================================================================*\
 *    Function: OS_Receive_Message
 *
 *  Parameters: 'msg'   pointer to write the message to from mailbox
 *
 *     Returns: Status_Type - E_OK if message received
 *                     E_COM_NOMSG if no message present
 *
 *    Modifies: task's mailbox
 *
 * Description: Returns the current message in mailbox if there is one.
 *              Unlike OS_Wait_Message(), OS_Receive_Message() does not suspend
 *              the calling task if a message is not available.
 *
\*===========================================================================*/
Status_Type OS_Receive_Message(Task_Type task_id,Message_Type * msg)
{
    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxTaskQueue[task_id]== NULL)
    {
        return (OS_E_ERROR);
    }
    if(pdFALSE == xQueueReceive(pxTaskQueue[task_id],(void *)msg,0))
    {
        return (OS_E_ERROR);     /* else return there was no message */
    }
    else
    {
        return (OS_E_OK);     /*OK */
    }
}

/*===========================================================================*\
 *    Function: OS_Wait_Message
 *
 *  Parameters: 'msg'   pointer to write the message to from mailbox
 *              'timeout'   maximum number of clock ticks to wait for resource
 *                    (0 will wait 'forever', unless previous alarm condition is set)
 *
 *     Returns: Status_Type - E_OK if message received
 *                     E_COM_NOMSG if timeout
 *
 *    Modifies: none
 *
 *  Description: if mailbox empty then waits for it with optional timeout
 *            else returns with message from the mailbox, emptying the mailbox
 *
\*===========================================================================*/
Status_Type OS_Wait_Message(Task_Type task_id,Message_Type * msg, Tick_Type timeout)
{
    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxTaskQueue[task_id]== NULL)
    {
        return (OS_E_ERROR);
    }
    if(pdFALSE == xQueueReceive(pxTaskQueue[task_id],(void *)msg,timeout))
    {
        return (OS_E_ERROR);     /* else return there was no message */
    }
    else
    {
        return (OS_E_OK);     /*OK */
    }
}

/*===========================================================================*\
 * Description: Builds a Message Type message from the id and data
 *  Parameters: 16 bit message id and 16 bit data
 *     Returns: 32 bit packed data
\*===========================================================================*/
Message_Type Build_Message(uint16_t id, int16_t data)
{
    Data_Message_T temp_msg = {0};
    temp_msg.parts.msg = id;
    temp_msg.parts.data = data;
    return (temp_msg.all);
}
/*===========================================================================*\
 *   Function: os_get resource
 *
 * Parameters:  'resid'    is the index of desired resource(Semaphore)
 *
 *    Returns:  E_OK (0)   if the resource is available
 *                     the semaphore is decremented so the next time
 *                     os_get_resource() is called, the resource may no
 *                     longer be available.
 *              E_OS_RESOURCE (!=0) if the resource is not available
 *
 *    Modifies: resources
 *
 * Description: This internal function actually checks the semaphore to see if
 *             the resource is available or if an event occurred.
\*===========================================================================*/
static Status_Type os_get_resource(Resource_Type resid)
{
    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxResources[resid] == NULL)
    {
        return (OS_E_OS_RESOURCE);
    }
    if(pdTRUE == xSemaphoreTake(pxResources[resid], 0))
    {
        return (OS_E_OK);
    }
    else
    {
        return (OS_E_OS_RESOURCE);/* default is resource not available */
    }
}

/*===========================================================================*\
 *   Function: OS_Get Resource
 *
 * Parameters:  'resid'    is the index of desired resource
 *
 *    Returns:  E_OK (0)   if the resource is available
 *                     the semaphore is decremented so the next time
 *                     OS_Get_Resource() is called, the resource may no
 *                     longer be available.
 *              E_OS_RESOURCE (!=0) if the resource is not available
 *
 *    Modifies: resources
 *
 * Description: This global function runs the scheduler before checking
 *              the semaphore to see if the resource
 *              is available or if an event occurred.  Unlike Wait_Resource,
 *              Get_Resource does not suspend the calling task if the
 *              resource is not available
\*===========================================================================*/
Status_Type OS_Get_Resource(Resource_Type resid)
{
    return (os_get_resource(resid));     /* attempt actual get of resource */
}

/*===========================================================================*\
 *    Function: OS_Wait_Resource
 *
 *  Parameters: 'resid'    is the index of desired resource
 *              'timeout'   maximum number of clock ticks to wait for resource
 *                    (0 will wait 'forever', unless previous alarm condition is set)
 *
 *     Returns: E_OK (0)       if the resource was allocated within timeout.
 *              E_OS_RESOURCE  if the resource was not available within timeout
 *
 *    Modifies: resources
 *
 * Description: This function checks the semaphore to see if the resource is available.
 *              Wait Resource will block the calling task if the resource is not available
 *
\*===========================================================================*/
Status_Type OS_Wait_Resource(Resource_Type resid, Tick_Type timeout)
{
    //Need check the pointer valid or not, freeRTOS doesn't do this.
    if (pxResources[resid] == NULL)
    {
        return (OS_E_OS_RESOURCE);
    }
    if(pdTRUE == xSemaphoreTake(pxResources[resid], timeout))
    {
       return (OS_E_OK);
    }
    else
    {
       return (OS_E_TIMEOUT);/* semaphore wait timer out */
    }
}

/*===========================================================================*\
 *    Function: OS_Release Resource
 *
 *  Parameters: 'resid'    is the index of desired resource
 *
 *     Returns: E_OK
 *
 *    Modifies: resources
 *
 * Description: This function releases a resource by either assigning it
 *              to a waiting task or incrementing its semaphore.
 *              Note, task may block if higher priority task was waiting.
\*===========================================================================*/
Status_Type OS_Release_Resource(Resource_Type resid)
{
    if (NULL  != pxResources[resid])
    {
        if(pdTRUE == xSemaphoreGive(pxResources[resid]))
        {
            return (OS_E_OK);
        }
        else
        {
            return (OS_E_ERROR);/*error*/
        }
    }
    else
    {
        return (OS_E_ERROR);/*error*/
    }
}

/*===========================================================================*\
 *    Function: OS_Release_Resource_FromISR
 *
 *  Parameters: 'resid'    is the index of desired resource
 *
 *     Returns: E_OK
 *
 *    Modifies: resources
 *
 * Description: This function releases a resource by either assigning it
 *              to a waiting task or incrementing its semaphore.
 *              Note, No schedule will happen.
\*===========================================================================*/
Status_Type OS_Release_Resource_From_ISR(Resource_Type resid, bool from_tick_isr)
{
    BaseType_t  HigherPriorityTaskWoken = false; //fixme.flag indicates if higher priority task woken.

    if (NULL  != pxResources[resid])
    {
        if(pdTRUE == xSemaphoreGiveFromISR(pxResources[resid], &HigherPriorityTaskWoken))
        {
            if(!from_tick_isr)
            {
                portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
            }
            return (OS_E_OK);                                                                                             //would switch in rtc interrupt.
        }
        else
        {
            return (OS_E_ERROR);/*error*/
        }
    }
    else
    {
        return (OS_E_ERROR);/*error*/
    }
}
/***********************************************************************
 *    Function: OS_Sleep
 *
 *  Parameters: 'time' in msec to sleep for
 *
 *     Returns: None
 *
 *    Modifies: none
 *
 * Description: Delays the current task for 'n' msec.
 *              Note: If scheduler is locked (which the current task must have done)
 *                    this function will return immediately.
 *
 **********************************************************************/
void OS_Sleep(Tick_Type time)
{
    if ( os_started )                                  /* task switch only if OS has been started */
    {
        vTaskDelay(time);
    }
    else
    {
        UNUSED_PARAM(time);
        Enable_Interrupts();                               /* ensure interrupts are enabled */
    }
}
/***********************************************************************
 *    Function: OS_Is_Started
 *
 *  Parameters: 
 *
 *     Returns: None
 *
 *    Modifies: none
 *
 **********************************************************************/
bool OS_Is_Started(void)
{
    return (os_started);
}
/***********************************************************************
 *    Function: OS_Is_Started
 *
 *  Parameters: 
 *
 *     Returns: None
 *
 *    Modifies: none
 *
 **********************************************************************/
void OS_Clr_Start_Flag(void)
{
    os_started = false;
}
/*===========================================================================*\
 *    Function: vApplicationStackOverflowHook
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 * Description: just for link error fixed
 *
\*===========================================================================*/
#if 1
void vApplicationStackOverflowHook( TaskHandle_t *pxTask,signed char *pcTaskName )
{
	/* This will be called if a task overflows its stack.  pxCurrentTCB
	can be inspected to see which is the offending task. */
	for( ;; )
    {
        DEBUG(DEBUG_HIGH, "Statc overflow:%s\r\n", pcTaskName);
    }
}
/*===========================================================================*\
 *    Function: vApplicationIdleHook
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 * Description: Application hook in IDLE Task
 *
\*===========================================================================*/
extern void vApplicationIdleHook( void )
{
      Idle_Task(); 
}
/*===========================================================================*\
 *    Function: vApplicationTickHook
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 * Description: Application hook in Tick interrupt ISR
 *
\*===========================================================================*/
extern void vApplicationTickHook(void)
{
    if(OS_Is_Started())
    {
        Periodic_Tick();
    }
}

extern void vApplicationMallocFailedHook(void)
{

}
#endif
/*===========================================================================*\
 *    Function: vApplicationGetTickBaseHook
 *
 *  Parameters: None
 *
 *     Returns: None
 *
 * Description: 
 *
\*===========================================================================*/
extern unsigned char vApplicationGetTickBaseHook(void)
{
    return(0);
}

void vApplicationDaemonTaskStartupHook(void)
{ 
     /* Do initialization after power on. */
    Task_Type   task_id;
    BOADR_IO_Init();
    BOADR_Init();
    PS_Initialize();
    if (Cold_Start())
    {
        Sys_Clear_Wakeup_Src_Flags();
        Sys_Clear_Standby_Req_Flag();
    }
    OS_Init();
    for (task_id = (Task_Type)0; task_id < (OS_NUM_TASKS-3); task_id++)
    {
        OS_Activate_Task(task_id);
    }
    os_started = true;                   /* flag that task switching has been enabled */  
    //WDOG_Initialize();
}

/*===========================================================================*\

 * File Revision History (bottom to top: first revision to last revision)
 *===========================================================================
 *
 * Date           
 * ----------- --------
 *
\*===========================================================================*/
