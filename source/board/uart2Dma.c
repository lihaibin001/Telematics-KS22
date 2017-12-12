#include "fsl_uart_edma.h"
#include "fsl_dma_manager.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "fsl_uart.h"

#include "FreeRTOS.h"
#include "semphr.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define UART_TX_DMA_CHANNEL 0
#define UART_RX_DMA_CHANNEL 1
#define UART_TX_DMA_REQUEST kDmaRequestMux0UART1Tx
#define UART_RX_DMA_REQUEST kDmaRequestMux0UART1Rx

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uart_edma_handle_t g_uartEdmaHandle;
static edma_handle_t g_uartTxEdmaHandle;
static edma_handle_t g_uartRxEdmaHandle;
static uint16_t dataOut;
static uart_transfer_t sendXfer;
static uart_transfer_t receiveXfer;
static SemaphoreHandle_t TxBinary;
/*******************************************************************************
 * Code
 ******************************************************************************/

/* callback */
void UART_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
    userData = userData;
    if (kStatus_UART_TxIdle == status)
    {
        
    }

    if (kStatus_UART_RxIdle == status)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR( TxBinary, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

/* uart2 dma initialzie */
void uart2DmaInit(uint32_t baud)
{
    uart_config_t uartConfig;
    UART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = baud;
    uartConfig.enableTx = true;
    uartConfig.enableRx = true;

    UART_Init(DEMO_UART, &uartConfig, CLOCK_GetFreq(DEMO_UART_CLKSRC));

    /* Configure DMA. */
    DMAMGR_Init();

    /* Request dma channels from DMA manager. */
    DMAMGR_RequestChannel(UART_TX_DMA_REQUEST, UART_TX_DMA_CHANNEL, &g_uartTxEdmaHandle);
    DMAMGR_RequestChannel(UART_RX_DMA_REQUEST, UART_RX_DMA_CHANNEL, &g_uartRxEdmaHandle);

    /* Create UART DMA handle. */
    UART_TransferCreateHandleEDMA(DEMO_UART, &g_uartEdmaHandle, UART_UserCallback, NULL, &g_uartTxEdmaHandle,
                          &g_uartRxEdmaHandle);
    TxBinary = xSemaphoreCreateBinary();
    if(TxBinary == NULL)
    {
        for(;;)
        {
            DEBUG(DEBUG_LOW, "[UART2] Create TxBinary failed!\r\n");
        }
    }
}

bool uart2DmaSendData(void *pData, uint16_t dataLen)
{
    sendXfer.dataSize = dataLen;
    sendXfer.data = pData;
    UART_SendEDMA(UART2, &g_uartEdmaHandle, &sendXfer);
    if(xSemaphoreTake(TxBinary, pdMS_TO_TICKS(500)) != pdPASS)
    {
        DEBUG(DEBUG_LOW, "[UART2] Send failed!\r\n");
        return false;
    }
    return true;
}

uint16_t uart2DmaGetRxCnt(void)
{
    uint32_t cnt = 0;
    UART_TransferGetReceiveCountEDMA(UART2, g_uartEdmaHandle, &cnt);
    return cnt;
}

uint16_t uart2DmaGetData(void *pData)
{
    
    return NULL;
}

