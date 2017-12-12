#include "standard.h"
#include "vehicle.h"
#include "diag_task.h"
#include "rtos.h"
#include "ACC_Detect.h"
#include "ring_buf.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "debug.h"
/*******************************************************************************
 * define
 ******************************************************************************/
#define UDS_DATA_COUNT      (uint8_t)2
#define VHCL_CAN_BAUD       250000
/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct
{
    flexcan_frame_t udsData[UDS_DATA_COUNT];
    Ring_Buf_Type dataIdx;
}udsDataTbl_t;
typedef struct
{
    flexcan_frame_t canData[id_FrameCnt];
    SemaphoreHandle_t xSemaphore;
}canDataTbl_t;
/*******************************************************************************
 * Variables
 ******************************************************************************/
static canDataTbl_t canDataTbl;
static udsDataTbl_t udsDataTbl;
static uint8_t alarmLevel;
static uint8_t busoffCount = 0;
/*******************************************************************************
 * declaration
 ******************************************************************************/
//static void VHCL_ParseUDSVIN(flexcan_frame_t const *data);
/*******************************************************************************
*    Function: VHCL_AlarmDetect
*
*  Parameters: None.
*     Returns: The alarm level.
* Description: Detect the alrm level
*******************************************************************************/
static uint8_t VHCL_AlarmDetect(flexcan_frame_t *pFrame)
{
    uint8_t alarmLevel_tmp = pFrame->dataByte4 >> 5;
    Data_Message_T msg;
    if(alarmLevel_tmp == ALARM_LEVEL3)
    {
        if(alarmLevel_tmp == alarmLevel)
        {
            //Detect the level 3 alarm first time,
            //send the alrm event to record_task
            
            msg.parts.msg = RECORD_EVT_ALARM;
            msg.parts.data = LEVEL3_ALARM_ASSERT;
            OS_Send_Message(OS_IOT_TASK, msg.all);
        }
    }
    else
    {
        if(alarmLevel == ALARM_LEVEL3)
        {
            //Level 3 alarm deassert first time
            msg.parts.msg = RECORD_EVT_ALARM;
            msg.parts.data = LEVEL3_ALARM_ASSERT;
            OS_Send_Message(OS_IOT_TASK, msg.all);
        }
    }
    return alarmLevel_tmp;
}

/*******************************************************************************
*    Function: VHCL_FlushVehicleInfo
*
*  Parameters: 'pData' point to the data to be stored
*     Returns: 0 sucess, 1 parameter error, 2 buffer is locked
* Description: Store the received data
*******************************************************************************/
static uint8_t VHCL_CAN_StoreData(flexcan_frame_t *pData)
{
    CanFrameID_t id_index = id_FrameCnt;
    uint32_t rx_id = 0;
    if(xSemaphoreTakeFromISR(canDataTbl.xSemaphore, NULL) == false)
    {
        return 2;
    }
    if(pData == NULL)
    {
        xSemaphoreGiveFromISR(canDataTbl.xSemaphore, NULL);
        return 1;
    }
    rx_id = pData->format == kFLEXCAN_FrameFormatStandard ? 
            pData->id >> CAN_ID_STD_SHIFT : pData->id >> CAN_ID_EXT_SHIFT;
    switch(rx_id)
    {
        case 0x1801FFF4:
            id_index = id_0x1801FFF4;
            break;
        case 0x1802FFF4:
            id_index = id_0x1802FFF4;
            break;
        case 0x1803FFF4:   
            id_index = id_0x1803FFF4;
            break;
        case 0x1804FFF4:
            VHCL_AlarmDetect(pData);
            id_index = id_0x1804FFF4;
            break;
        case 0x1805FFF4:
            id_index = id_0x1805FFF4;
            break;
        case 0x1806FFF4:
            id_index = id_0x1806FFF4;
            break;
        case 0x1801D2F4:
            id_index = id_0x1801D2F4;
            break;
        case 0x1802D2F4:
            id_index = id_0x1802D2F4;
            break;
        case 0x1803D2F4:
            id_index = id_0x1803D2F4;
            break;
        case 0x1804D2F4:
            id_index = id_0x1804D2F4;
            break;
        case 0x1805D2F4:
            id_index = id_0x1805D2F4;
            break;
        case 0x1806D2F4:
            id_index = id_0x1806D2F4;
            break;
        case 0x1807D2F4:
            id_index = id_0x1807D2F4;
            break;
        case 0x1808D2F4:
            id_index = id_0x1808D2F4;
            break;
        case 0x1809D2F4:
            id_index = id_0x1809D2F4;
            break;
        case 0x180AD2F4:
            id_index = id_0x180AD2F4;
            break;
        case  0x1850D2F4:
            id_index = id_0x1850D2F4;
            break;
        case  0x1851D2F4:
            id_index = id_0x1851D2F4;
            break;
        case 0xc0401D0:
            id_index = id_0xC0401D0;
            break;
        case 0xc0501D0:
            id_index = id_0xC0501D0;
            break;
        case 0xc0601D0:
            id_index = id_0xC0601D0;
            break;
        case 0xc08A7F0:
            id_index = id_0xC08A7F0;
            break;
        case  0xc09A7F0:
            id_index = id_0xC09A7F0;
            break;
        case 0xc0AA7F0:
            id_index = id_0xC0AA7F0;
            break;
        case 0xc0BA7F0:
            id_index = id_0xC0BA7F0;
            break;
        case 0x1806E5F4:
            id_index = id_0x1806E5F4;
            break;
        case 0x18FF50E5:
            id_index = id_0x18FF50E5;
            break;
        case 0x7E0:
            {
                Data_Message_T msg;
                UDS_SetData(pData);
                msg.parts.msg = DIAG_EVT_UDS_EVT;
                OS_Send_MessageISR(OS_DIAG_TASK, msg.all);
                id_index = id_0x7E0;
            }
            xSemaphoreGiveFromISR(canDataTbl.xSemaphore, NULL);
            return 0;
        default:
            xSemaphoreGiveFromISR(canDataTbl.xSemaphore, NULL);
            return 1;        
    }
    memcpy(&canDataTbl.canData[id_index], pData, sizeof(flexcan_frame_t));
    xSemaphoreGiveFromISR(canDataTbl.xSemaphore, NULL);
    return 0;
}

/*******************************************************************************
*    Function: VHCL_CAN_ReceiveDataHander
*
*  Parameters: Reference "flexcan_transfer_callback_t".
*     Returns: None.
* Description: Handle the received data
*******************************************************************************/
static void VHCL_CAN_ReceiveDataHander(CAN_Type *pBase, 
                                    flexcan_handle_t *pHandle, 
                                    status_t status, 
                                    uint32_t result, 
                                    void *userData)
{
    switch (status)
    {
        case kStatus_FLEXCAN_RxFifoWarning:
            break;
        case kStatus_FLEXCAN_RxIdle:
            break;
        case kStatus_FLEXCAN_TxIdle:
            break;
        case kStatus_FLEXCAN_TxSwitchToRx:
            break;
        case kStatus_FLEXCAN_RxFifoIdle:
            VHCL_CAN_StoreData(pHandle->rxFifoFrameBuf);
            break;
        case kStatus_FLEXCAN_RxFifoOverflow:
            break;
        case kStatus_FLEXCAN_ErrorStatus:
            if(result == kFLEXCAN_BusOffIntFlag)
            {
                busoffCount++;
            }
            break;
        case kStatus_FLEXCAN_UnHandled:
            break;
        default:
            break;
    }  
}
/*******************************************************************************
*    Function: VHCL_CAN_GetDataList
*
*  Parameters: None
*     Returns: the CAN data list
* Description: 
*******************************************************************************/
const flexcan_frame_t* VHCL_CAN_GetDataList(void)
{
    const TickType_t xMaxBlock = pdMS_TO_TICKS( 100 );
    if(!xSemaphoreTake(canDataTbl.xSemaphore, xMaxBlock ))
    {
        return NULL;
    }
    return (const flexcan_frame_t*)canDataTbl.canData;  
}
/*******************************************************************************
*    Function: VHCL_CAN_GiveDataList
*
*  Parameters: None.
*     Returns: None.
* Description: must be called after VHCL_CAN_GetDataList return 0
*******************************************************************************/
void VHCL_CAN_GiveDataList(void)
{
    xSemaphoreGive(canDataTbl.xSemaphore);
}
/*******************************************************************************
*    Function: VHCL_GetCanDataById
*
*  Parameters: 'pData' point to a buffer that store the data
*               'cnt' express how many frame to be be read
*     Returns: 0 sucess, 1 parameter error, 2 buffer is locked
* Description: 
*******************************************************************************/
uint8_t VHCL_GetCanDataById(CanFrameID_t id, flexcan_frame_t *pData)
{
    if(!xSemaphoreTake(canDataTbl.xSemaphore, pdMS_TO_TICKS( 20 ) ))
    {
        return 2;
    }
    if(id >= id_FrameCnt || pData == NULL)
    {
        xSemaphoreGive(canDataTbl.xSemaphore);
        return 1;
    }
    memcpy(pData, &canDataTbl.canData[id], sizeof(flexcan_frame_t));
    xSemaphoreGive(canDataTbl.xSemaphore);
    return 0;
}
/*******************************************************************************
*    Function: VHCL_CAN_Init
*
*  Parameters: None.
*     Returns: true/false
* Description: Initialize the CAN controller that corresponded to the vehicle
*******************************************************************************/
bool VHCL_CAN_Init(void)
{
    
    uint32_t fileterTbl[id_FrameCnt/2 + id_FrameCnt%2 + 1] = {
        kFLEXCAN_RxFifoFilterTypeB,
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1801FFF4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1802FFF4,0,1) ,
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1803FFF4,0,1) | 
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1804FFF4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1805FFF4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1806FFF4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1801D2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1802D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1803D2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1804D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1805D2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1806D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1807D2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1808D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1809D2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x180AD2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x180BD2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x180CD2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x180DD2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x180ED2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x180FD2F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1810D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1850D2F4,0,1) | 
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x1851D2F4,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0x1806E5F4,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0x18FF50E,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0xC08A7F0,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0xC09A7F0,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0xC0AA7F0,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0xC0BA7F0,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0xC0401D0,0,1) |
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_LOW(0xC0501D0,0,1),
        FLEXCAN_RX_FIFO_EXT_MASK_TYPE_B_HIGH(0xC0601D0,0,1) |
        (FLEXCAN_RX_FIFO_STD_MASK_TYPE_A(0x7E0,0,0) >> 16),
    }; 
    CAN_Initialize(CAN_CHANNEL0, VHCL_CAN_ReceiveDataHander, VHCL_CAN_BAUD);
    CAN_ReceiveFIFOConfig(CAN_CHANNEL0, fileterTbl, id_FrameCnt/2 + id_FrameCnt%2) ;
    IO_CAN_ENABLE_OUT(Bit_SET); //enable can transceiver
    IO_CAN_STANBY_OUT(Bit_SET); //quit standby mode
    canDataTbl.xSemaphore = xSemaphoreCreateMutex();
    if(NULL == canDataTbl.xSemaphore)
    {
        DEBUG(DEBUG_HIGH, "[VHCL] Create mutex error at line %d\r\n",__LINE__);
        return false;
    }
    Ring_Buf_Reset(&udsDataTbl.dataIdx, UDS_DATA_COUNT);
    return true;
}

/*******************************************************************************
*    Function: VHCL_UDS_GetData
*
*  Parameters: 'pData' point to a buffer to stored the got data .
*     Returns: None.
* Description: Get a data from the UDS data table. 
*******************************************************************************/
void UDS_GetData(flexcan_frame_t *pData)
{
    if(pData)
    {
        memcpy(pData,
               &udsDataTbl.udsData[udsDataTbl.dataIdx.out],
               sizeof(flexcan_frame_t));
        Ring_Buf_Remove(&udsDataTbl.dataIdx);
    }
}
/*******************************************************************************
*    Function: UDS_SetData
*
*  Parameters: 'pData' point to received data.
*     Returns: None.
* Description: Store the received data to the UDS data table
*******************************************************************************/
void UDS_SetData(flexcan_frame_t *pData)
{
    if(pData)
    {
        memcpy(&udsDataTbl.udsData[udsDataTbl.dataIdx.in], 
               pData, 
               sizeof(flexcan_frame_t));
        Ring_Buf_Add(&udsDataTbl.dataIdx);
    }
}

/*******************************************************************************
*    Function: VHCL_getAlarmLevel
*
*  Parameters: 'pData' point to received data.
*     Returns: The alarm level.
* Description: Get the alarm level
*******************************************************************************/
uint8_t VHCL_getAlarmLevel(void)
{
    return (const uint8_t)alarmLevel;
}
