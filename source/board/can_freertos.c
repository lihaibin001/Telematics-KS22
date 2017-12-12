#include "can_freertos.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CAN_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void CAN0_RTOS_SendTask(void *argument);
static void CAN0_RTOS_Parse_Task(void *argument);
static uint8_t CAN_RTOS_StoreFrame(uint8_t chan, flexcan_frame_t *frame);
static can_pareser_t can_pareser[CAN_NUM_CHANNELS];
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern flexcan_handle_t flexcanHandle[CAN_NUM_CHANNELS];
CAN_rtos_handle_t can_rtos_handle[CAN_NUM_CHANNELS];
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief FlexCAN Call Back function
 */
static void CAN0_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        case kStatus_FLEXCAN_RxFifoWarning:
            break;
        case kStatus_FLEXCAN_RxIdle:
            break;
        case kStatus_FLEXCAN_TxIdle:      
            xEventGroupSetBitsFromISR(can_rtos_handle[0].can_event, CAN0_TX_OK_EVENT, 0);
            break;
        case kStatus_FLEXCAN_TxSwitchToRx:
            break;
        case kStatus_FLEXCAN_RxFifoIdle:
            CAN_RTOS_StoreFrame(0, handle->rxFifoFrameBuf);
            break;
        case kStatus_FLEXCAN_RxFifoOverflow:
            break;
        case kStatus_FLEXCAN_ErrorStatus:
            break;
        case kStatus_FLEXCAN_UnHandled:
            break;
        default:
            break;
    }
}
/*******************************************************************************
*    Function: CAN_RTOS_Init
*
*  Parameters: chan, spcecify the can channel. xfer point to the data buffer
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: send data without blocking
*******************************************************************************/
uint8_t CAN_RTOS_Init(uint8_t chan, uint32_t bandrate)
{
    flexcan_transfer_callback_t cb;
    int8_t *send_task_name = NULL, *parse_task_name = NULL;
    switch(chan)
    {
        case CAN_CHANNEL0:
            cb = CAN0_callback;
            can_rtos_handle[chan].send_task = CAN0_RTOS_SendTask;
            can_rtos_handle[chan].parse_task = CAN0_RTOS_Parse_Task;
            send_task_name = "CAN0_send_task";
            parse_task_name = "CAN0_parse_task";
            break;
        default:
            return 1;
    }
    can_rtos_handle[chan].can_event = xEventGroupCreate();
    if(can_rtos_handle[chan].can_event == NULL)
    {
        return 2;
    }
    can_rtos_handle[chan].rx_queue = xQueueCreate(5, sizeof(flexcan_frame_t));
    if(can_rtos_handle[chan].rx_queue == 0)
    {
        return 2;
    }
    can_rtos_handle[chan].tx_queue = xQueueCreate(5,sizeof(flexcan_frame_t));
    if(can_rtos_handle[chan].tx_queue == 0)
    {
        return 2;
    }
    
    xTaskCreate(can_rtos_handle[chan].send_task, 
                (const char *)(send_task_name), 
                configMINIMAL_STACK_SIZE, 
                NULL, CAN_TASK_PRIORITY, 
                NULL);
    xTaskCreate(can_rtos_handle[chan].parse_task, 
                 (const char *)(parse_task_name), 
                 configMINIMAL_STACK_SIZE, 
                 NULL, CAN_TASK_PRIORITY, 
                 NULL);
    CAN_Initialize(chan, cb, bandrate);
    return 0;
}
/*******************************************************************************
*    Function: CAN_RTOS_Deinit
*
*  Parameters: chan express the channel of CAN
*     Returns: 0 sucess, 1 fail
* Description: Deinitialize the specified CAN channel
*******************************************************************************/
uint8_t CAN_RTOS_Deinit(uint8_t chan)
{
    return 0;
}

/*******************************************************************************
*    Function: CAN_RTOS_Send
*
*  Parameters: chan express the channel of CAN
*               frame point to the frame
*     Returns: 0 sucess, 1 fail
* Description: send frame to the specifed CAN channel
*******************************************************************************/
uint8_t CAN_RTOS_Send(uint8_t chan, flexcan_frame_t *frame)
{
    if(chan >= CAN_NUM_CHANNELS || frame == NULL)
    {
        return 1;
    }
    if(errQUEUE_FULL == xQueueSend(can_rtos_handle[chan].tx_queue, frame, 0))
    {
        return 2;
    }
    return 0;
}

/*******************************************************************************
*    Function: CAN0_RTOS_SendTask
*
*  Parameters: argument point the argument sent to the task
*     Returns: none
* Description: can channel 0 send task
*******************************************************************************/
static void CAN0_RTOS_SendTask(void *argument)
{
    flexcan_frame_t frame;
    EventBits_t event_bits = 0;
    for(;;)
    {
        xQueueReceive(can_rtos_handle[0].tx_queue, &frame, portMAX_DELAY);
        CAN_SendNoBlocking(0, &frame);
        event_bits = xEventGroupWaitBits(can_rtos_handle[0].can_event,    /* The event group handle. */
                                         CAN0_TX_OK_EVENT | CAN0_TX_ERR_EVENT,        /* The bit pattern the event group is waiting for. */
                                         pdTRUE,         /* BIT_0 and BIT_4 will be cleared automatically. */
                                         pdFALSE,        /* Don't wait for both bits, either bit unblock task. */
                                         portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */
        switch(event_bits)
        {
            case CAN0_TX_OK_EVENT:
                break;
            case CAN0_TX_ERR_EVENT:
                break;
            default:
                break;
        }
    }
}


/*******************************************************************************
*    Function: CAN_PraserRegister
*
*  Parameters: channel express the CAN channel
*               pareser express the pareser for the can frame
* Description: regist the pareser
*******************************************************************************/
void CAN_PraserRegister(uint8_t channel, can_pareser_t pareser)
{
    if(channel >= CAN_NUM_CHANNELS || pareser == NULL)
    {
        return ;
    }
    can_pareser[channel] = pareser;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CAN_RTOS_SendTask
 * Description   : Initializes the CAN instance for application
 *
 *END**************************************************************************/
static void CAN0_RTOS_Parse_Task(void *argument)
{
    flexcan_frame_t frame;
    for(;;)
    {
        xQueueReceive(can_rtos_handle[0].rx_queue, &frame, portMAX_DELAY);
        can_pareser[0]( &frame);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CAN_RTOS_StoreFrame
 * Description   : Receives chars for the application
 *
 *END**************************************************************************/
static uint8_t CAN_RTOS_StoreFrame(uint8_t chan, flexcan_frame_t *frame)
{
    if(chan >= CAN_NUM_CHANNELS || frame == NULL)
    {
        return 1;
    }
    xQueueSendFromISR(can_rtos_handle[chan].rx_queue, frame, 0);
    return 0;
}
