#ifndef __CAN_FREERTOS_H__
#define __CAN_FREERTOS_H__

#include "can_ks22.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
/*!
 * @addtogroup CAN_freertos_driver
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CAN0_TX_OK_EVENT    (1 << 0)
#define CAN0_TX_ERR_EVENT   (1 << 1)
typedef void (*RTOSTask)(void *argument);
typedef void (*can_pareser_t)(flexcan_frame_t *frame);
/*! @brief CAN configuration structure */
struct CAN_rtos_config
{
    CAN_Type *base;                 /*!< CAN base address */
    uint32_t srcclk;                /*!< CAN source clock in Hz*/
    uint32_t baudrate;              /*!< Desired communication speed */
    uint8_t *buffer;                /*!< Buffer for background reception */
    uint32_t buffer_size;           /*!< Size of buffer for background reception */
};

/*! @brief CAN FreeRTOS handle */
typedef struct _CAN_rtos_handle
{
    EventGroupHandle_t can_event;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    RTOSTask send_task; 
    RTOSTask parse_task;
//    CAN_rtos_handle_t *can_handle;
} CAN_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name CAN RTOS Operation
 * @{
 */

uint8_t CAN_RTOS_Init(uint8_t chan, uint32_t bandrate);
uint8_t CAN_RTOS_Deinit(uint8_t chan);
uint8_t CAN_RTOS_Send(uint8_t chan, flexcan_frame_t *frame);
uint8_t CAN_RTOS_Receive(uint8_t chan, flexcan_frame_t *frame);
void CAN_PraserRegister(uint8_t channel, can_pareser_t pareser);
/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _CAN_RTOS_H__ */
