#ifndef __CAN_HW_H__
#define __CAN_HW_H__
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "board.h"

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "clock_config.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

#define CAN_FrameFormatStandard     kFLEXCAN_FrameFormatStandard
#define CAN_FrameFormatExtend       kFLEXCAN_FrameFormatExtend

#define CAN_FrameTypeData           kFLEXCAN_FrameTypeData
#define CAN_FrameTypeRemote         kFLEXCAN_FrameTypeRemote
   
typedef enum CAN_CHANNEL_Tag
{
    CAN_CHANNEL0 = 0,
//    CAN_CHANNEL1,
    CAN_NUM_CHANNELS
}CAN_CHANNEL_T;

extern QueueHandle_t can_queue[CAN_NUM_CHANNELS];
extern EventGroupHandle_t can_Event;

void CAN_Initialize(CAN_CHANNEL_T chan, flexcan_transfer_callback_t cb, uint32_t bandrate);
void CAN_RcieiveFIFOConfig(CAN_CHANNEL_T chan);
status_t CAN_EnableRecive(CAN_CHANNEL_T chan);
void* CAN_GetFrame(CAN_CHANNEL_T chan, uint32_t id);
uint8_t CAN_SendNoBlocking(CAN_CHANNEL_T chan, flexcan_frame_t *frame);
uint8_t CAN_SendBlocking(CAN_CHANNEL_T chan, flexcan_frame_t *frame);
void CAN_ReceiveFIFOConfig(CAN_CHANNEL_T chan, uint32_t filter[], uint8_t filterCnt);
void CAN_ReceiveConfig(CAN_CHANNEL_T chan);
#if defined(__cplusplus)
}
#endif /* __cplusplus*/
#endif //__CAN_HW_H__
