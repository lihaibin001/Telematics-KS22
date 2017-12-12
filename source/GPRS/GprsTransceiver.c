#include "GprsTransceiver.h"

#define GPRS_SENDER_QUEUE_MAX_LEN       5
#define GPRS_RECIEVER_QUEUE_MAX_LEN     5
typedef struct
{
    void *pData;
    uint16_t len;
    bool isNeedAck;
    uint8_t sesionNum,
}GprsSender_t;


static QueueHandle_t sendQueue;
static QueueHandle_t receiveQueue;
static bool GprsSenderBusy = false;
bool GprsCreateTransceiverQueue(void)
{
    sendQueue = xQueueCreate(GPRS_SENDER_QUEUE_MAX_LEN, sizeof(GprsSender_t));
    if(sendQueue == NULL)
    {
        DEBUG(DEBUG_HIGH, "[GPRS] Create Sender Queue fail\r\n");
        for(;;)
    }
    receiveQueue = xQueueCreate(GPRS_SENDER_QUEUE_MAX_LEN, sizeof(GprsSender_t));
    if(sendQueue == NULL)
    {
        DEBUG(DEBUG_HIGH, "[GPRS] Create Receiver Queue fail\r\n");
        for(;;)
    }
    return true;
}
bool GprsSendData(void *pData, uint16_t dataLen, uint8_t sesion, bool isNeedAck)
{
    GprsSender_t sender;
    void *sendData = pvPortMalloc(dataLen);
    if(sendData == NULL)
    {
        DEBUG(DEBUG_HIGH, "[GPRS] Malloc error!\r\n");
    }
    else
    {
        memcpy(sender.pData, pData, dataLen);
        sender.len = dataLen;
        sender.isNeedAck = isNeedAck;
        sender.sesionNum = sesion;
        if(xQueueSend(sendQueue, &sender, pdMS_TO_TICKS(100)) != pdPASS)
        {
            return false;
        }
        return true;
    }
}
void GprsSender(void)
{
    if(!GprsSenderBusy)
    {
        GprsSender_t sender;
        if(xQueuePeek(sendQueue, &sender, 0) == pdPASS)
        {
            
        }
    }
}
bool GprsReceiver(void *pData, uint16_t dataLen, uint8_t sesion)
{
    
}
void GprsReceiveChecker(void)
{
    
}
