#include "standard.h"
#include "can_hw.h"
//#include "string.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

#define CAN_TX_MB_NUM               (15)
/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t                flexcanHandle[CAN_NUM_CHANNELS];

static flexcan_frame_t          CAN_RxframeBuffer[CAN_NUM_CHANNELS];  
static flexcan_frame_t          CAN_TxFrameBuffer[CAN_NUM_CHANNELS]; 
static flexcan_mb_transfer_t    txFrameMB[CAN_NUM_CHANNELS] = 
{
    {
        .mbIdx = CAN_TX_MB_NUM,
        .frame = &CAN_TxFrameBuffer[CAN_CHANNEL0],
    },
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void CAN_InitIO(CAN_CHANNEL_T chan);
//static uint8_t CAN_MB_config(uint8_t chan);
//static uint8_t CAN_StoreFrame(uint8_t chan, flexcan_frame_t *xfer);

/*******************************************************************************
 * Code
 ******************************************************************************/
static void Can_selftestCb(CAN_Type *pBase, 
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
#if 0
bool CAN_sleftest(CAN_CHANNEL_T chan)
{
    CAN_Type *tmpCAN = NULL;
    flexcan_config_t flexcanConfig;
    CAN_InitIO(chan);
    switch(chan)
    {
        case CAN_CHANNEL0:
            tmpCAN = CAN0;
            break;
//            case CAN_CHANNEL1:
//                flexcanConfig.baudRate = CAN1_BANDRATE;
//                tmpCAN = CAN1;
            break;
        default:
            return;
    }
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
    flexcanConfig.baudRate = bandrate;
    /* set Priority*/
    NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 6);
    /* initialize the specify can channel */
    FLEXCAN_Init(tmpCAN, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));
    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(tmpCAN, &flexcanHandle[chan], 
                                 Can_selftestCb, NULL);
    /* Sets up the transmit message buffer. */
    FLEXCAN_SetTxMbConfig(tmpCAN, CAN_TX_MB_NUM, true);    
    return 0;
}
#endif
/*******************************************************************************
*    Function: CAN_Initialze
*
*  Parameters: chan, spcecify the can channel
*     Returns: none
* Description: Initialize the spcecified can channel
*******************************************************************************/
void CAN_Initialize(CAN_CHANNEL_T chan, flexcan_transfer_callback_t cb, uint32_t bandrate)
{
    if(chan < CAN_NUM_CHANNELS)
    {
        CAN_Type *tmpCAN = NULL;
        flexcan_config_t flexcanConfig;
        CAN_InitIO(chan);
        FLEXCAN_GetDefaultConfig(&flexcanConfig);
        flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
        flexcanConfig.baudRate = bandrate;
        switch(chan)
        {
            case CAN_CHANNEL0:
                tmpCAN = CAN0;
                break;
//            case CAN_CHANNEL1:
//                flexcanConfig.baudRate = CAN1_BANDRATE;
//                tmpCAN = CAN1;
                break;
            default:
                return;
        }
        /* set Priority*/
        NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 6);
        /* initialize the specify can channel */
        FLEXCAN_Init(tmpCAN, &flexcanConfig, CLOCK_GetFreq(kCLOCK_BusClk));
        /* Create FlexCAN handle structure and set call back function. */
        FLEXCAN_TransferCreateHandle(tmpCAN, &flexcanHandle[chan], cb, NULL);
        /* Sets up the transmit message buffer. */
        FLEXCAN_SetTxMbConfig(tmpCAN, CAN_TX_MB_NUM, true);
    }
}
/*******************************************************************************
*    Function: CAN_SendNoBlocking
*
*  Parameters: chan, spcecify the can channel. xfer point to the data buffer
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: send data without blocking
*******************************************************************************/
uint8_t CAN_SendNoBlocking(CAN_CHANNEL_T chan, flexcan_frame_t *frame)
{
    if(frame == NULL)
    {
        return 1;
    }
    else
    {
        CAN_Type *CANx = NULL;
        memcpy(txFrameMB[chan].frame, frame, sizeof(flexcan_frame_t));
        switch(chan)
        {
            case CAN_CHANNEL0:
                CANx = CAN0;
                break;
            default:
                return 1;
        }
        FLEXCAN_TransferSendNonBlocking(CANx, &flexcanHandle[chan], &txFrameMB[chan]);
        return 0;
    }
}

/*******************************************************************************
*    Function: CAN_SendNoBlocking
*
*  Parameters: chan, spcecify the can channel. xfer point to the data buffer
*     Returns: 0, sucess. 1, parameter error. 2, low lever error 
* Description: send data without blocking
*******************************************************************************/
uint8_t CAN_SendBlocking(CAN_CHANNEL_T chan, flexcan_frame_t *frame)
{
    if(frame == NULL)
    {
        return 1;
    }
    else
    {
        CAN_Type *CANx = NULL;
        memcpy(txFrameMB[chan].frame, frame, sizeof(flexcan_frame_t));
        switch(chan)
        {
            case CAN_CHANNEL0:
                CANx = CAN0;
                break;
            default:
                return 1;
        }
        FlEXCAN_TransferSendBlocking(CANx, CAN_TX_MB_NUM, frame);
        return 0;
    }
}

/*******************************************************************************
*    Function: CAN_InitIO
*
*  Parameters: chan, spcecify the can channel
*     Returns: none
* Description: Initialize the spcecified can IO port
*******************************************************************************/
static void CAN_InitIO(CAN_CHANNEL_T chan)
{
    if(chan < CAN_NUM_CHANNELS)
    {
        switch(chan)
        {
            case CAN_CHANNEL0:
                /* Initialize FlexCAN0 pins below */
                CLOCK_EnableClock(kCLOCK_PortA);

                /* Affects PORTB_PCR18 register */
                PORT_SetPinMux(PORTA, 12u, kPORT_MuxAlt2);
                /* Affects PORTB_PCR19 register */
                PORT_SetPinMux(PORTA, 13u, kPORT_MuxAlt2);
                break;
//            case CAN_CHANNEL1:
//                CLOCK_EnableClock(kCLOCK_PortE);
//                PORT_SetPinMux(PORTE, 24u, kPORT_MuxAlt2);
//                PORT_SetPinMux(PORTE, 25u, kPORT_MuxAlt2);
//                break;
            default:
                break;
        }
    }
}

/*******************************************************************************
*    Function: CAN_ReceiveFIFOConfig
*
*  Parameters: chan, spcecify the can channel
*               filter[], store the frame id we need
*               filterCnt, express the count of the frame id
*     Returns: none
* Description: config the receive fifo
*******************************************************************************/
void CAN_ReceiveFIFOConfig(CAN_CHANNEL_T chan, uint32_t filter[], uint8_t filterCnt)
{
    if(chan < CAN_NUM_CHANNELS && filter != NULL || filterCnt != 0)
    {
        flexcan_rx_fifo_config_t rxFifoConfig;
        flexcan_fifo_transfer_t can_fifo_transfer;
        CAN_Type *CANx;
        switch(chan)
        {
            case CAN_CHANNEL0:
                CANx = CAN0;
                break;
//            case CAN_CHANNEL1:
//                CANx = CAN1;
//                break;
            default:
                return;
        }
        rxFifoConfig.idFilterTable = &filter[1];
        rxFifoConfig.idFilterType = (flexcan_rx_fifo_filter_type_t)filter[0];
        rxFifoConfig.idFilterNum = filterCnt;
        rxFifoConfig.priority = kFLEXCAN_RxFifoPrioHigh;
        FlEXCAN_SetRxFifoConfig(CANx, &rxFifoConfig, true);
        can_fifo_transfer.frame = &CAN_RxframeBuffer[chan];       
        FLEXCAN_TransferReceiveFifoNonBlocking(CANx, &flexcanHandle[chan], &can_fifo_transfer);
    }
}
/*******************************************************************************
*    Function: CAN_ReceiveConfig
*
*  Parameters: chan, spcecify the can channel
*     Returns: none
* Description: config the receive fifo
*******************************************************************************/
void CAN_ReceiveConfig(CAN_CHANNEL_T chan)
{
    if(chan < CAN_NUM_CHANNELS)
    {
        flexcan_mb_transfer_t can_mb_transfer;
        CAN_Type *CANx;
        switch(chan)
        {
            case CAN_CHANNEL0:
                CANx = CAN0;
                break;
//            case CAN_CHANNEL1:
//                CANx = CAN1;
//                break;
            default:
                return;
        } 
        can_mb_transfer.frame = &CAN_RxframeBuffer[chan];
        can_mb_transfer.mbIdx = 6;
        FLEXCAN_TransferReceiveNonBlocking(CANx, &flexcanHandle[chan], &can_mb_transfer);   
    }
}
