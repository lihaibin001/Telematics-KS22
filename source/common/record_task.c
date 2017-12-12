/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2016
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2016, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/

/**********************************************************************
   Title                      : record_task.c         
                                                                         
   Module Description         : Handle timing/distance related record tasks.

   Author                     : 
   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include "record_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps.h"
#include "GPRS.h"
#include "ATProtocol.h"
#include "TelmProtocol.h"
//#define USE_DEBUG
#include "Debug.h"
#include "ACC_Detect.h"
/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define DEVICE_UNMOUNT_LIMIT (1)
#define AD_TEMP 0 /* temperature channel */
#define GPS_UPLOAD_INTERVAL (85)
     
#define RECORD_INTERVAL            15000 
#define RECORD_ARARM_INTERVAL      1000
#define RECORD_CHK_INTERVAL        1000

#define RECORD_ALRM_BCKP_CNT       30

#define MAX_TIME_TASK_WAIT              10 //ms
#define DEFAULT_RECORD_INTERVAL         30000 //ms
#define DEFAULT_HEART_BEART_INTERVAL    15000
#define RECORD_SEND_TIMEOUT             60000

#define RECORD_SEND_MAX_CNT         3

#define RECORD_DATA_TYPE_DATA       0
#define RECORD_DATA_TYPE_ACK        1
     
#define ACK_TO_SERVER_LEN           5
#define RECORD_QUEUE_SIZE           5

/**********************************************************************
 * typedef
 *********************************************************************/
typedef enum
{
    dataIdle,
    dataInit,
    dataTrans,
}dataState_t;

typedef struct
{
    realTimeInfo_t rltmInfo[RECORD_ALRM_BCKP_CNT];
    struct
    {
        uint8_t isFull:1;
        uint8_t idx:7;
    };
}rltmInfoTbl_t;

typedef struct
{
    TelmRealtime_t data;
    dataState_t state;
    uint8_t tryCnt;
    TickType_t transTimer;
}dataTmp_t;
uint32_t rltmLen = sizeof(rltmInfoTbl_t);
/**********************************************************************
 * Lacal variable
 *********************************************************************/
static rltmInfoTbl_t rltmInfoTbl;
static uint32_t recordInteral = DEFAULT_RECORD_INTERVAL;
static TickType_t recordTimer;
static TickType_t heartbeatTimer;
//static TickType_t gbRecordTimer;
//static QueueHandle_t recordQueue;
//static TelmRealtime_t *pRealtimeData;
//static dataState_t dataState;
static dataTmp_t *pRealtimeTmp = NULL;
static dataTmp_t *pBackupTmp = NULL;

static TickType_t logtimeout;
static uint8_t loginTimes;
static TickType_t gbLogTimeout;
static uint8_t gbLoginTimes;
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/
/**********************************************************************
 * Function Definitions
 *********************************************************************/
static void logStateDetetor(void);
static void RecordUploader(void);
static void heartBeatUploader(void);
static void recordRlealtimeData(void);
static void realtimeSendResHandle(bool sendRes, uint8_t channle);
static void backupSendResHandle(bool sendRes, uint8_t channle);
/* event handler */
static void prvRecord_evt_nop(int16_t data);
static void prvRecord_evt_rltmTalRefresh(int16_t data);
static void prvRecord_evt_Level3Alarm(int16_t data);
static void prvRecord_evt_CleanRecord(int16_t data);
static void prvRecord_evt_disConGb(int16_t data);
/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static void_int16_fptr record_event_handler[]=
{
    prvRecord_evt_nop,					// EVT_NOP
    prvRecord_evt_rltmTalRefresh,       //RECORD_EVT_RLTM_DATA_REFRESH
    prvRecord_evt_Level3Alarm,          //RECORD_EVT_WARNING_LEVEL_3
    prvRecord_evt_CleanRecord,
	prvRecord_evt_disConGb,
};
/*********************************************************************/
/* User file include                                                 */
/********************************************************************/

/*******************************************************************************
*    Function:  Record_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle timing/distance records.
*******************************************************************************/
extern void Record_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint32_t csq_time = OS_Time();
    DEBUG(DEBUG_MEDIUM, "[Record]:Record TASK Started!\r\n");
    IOT_receiveDatahanlerRegester(vTelmProt_ParseIpData);
    IOT_ipConnectStateChangeNotifierRegester(TelmProt_connectHandler);
    while(PS_Running())
    {
        msg.all = 0;
        Status_Type state = OS_Wait_Message(OS_RECORD_TASK,
                                            &msg.all,
                                            MSec_To_Ticks(MAX_TIME_TASK_WAIT));
        if(OS_E_OK == state)
        {
            if(msg.parts.msg < RECORD_NUM_EVENTS)
            {
                if(NULL != record_event_handler[msg.parts.msg])
                {
                    (*record_event_handler[msg.parts.msg])(msg.parts.data);
                    continue;
                }
            }
        }
        logStateDetetor();
        heartBeatUploader();
        RecordUploader();
        recordRlealtimeData();
    }
    OS_Terminate_Task();
}
/*******************************************************************************
*    Function:  Record_GetRecord
*
*  Parameters:  None
*     Returns:  A pointer point to the g_record_Data
* Description:  
*******************************************************************************/
realTimeInfo_t *Record_GetLastRltm(void)
{
    if(rltmInfoTbl.idx == 0)
    {
        if(rltmInfoTbl.isFull)
        {
            return &rltmInfoTbl.rltmInfo[RECORD_ALRM_BCKP_CNT - 1];
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        return &rltmInfoTbl.rltmInfo[rltmInfoTbl.idx - 1];
    }
}

static void logStateDetetor(void)
{
    uint8_t *pSendBufer = NULL;
    uint16_t len = 0;
    if(loginTimes == 0xFF)
    {
        if(logtimeout <= xTaskGetTickCount())
        {
            IOT_IpOpenUnblock(TELM_ENTERPRISE_SESION);
            loginTimes = 0;
        }
    }
    if(loginTimes != 0xFF && TelmProt_getSesionState(TELM_ENTERPRISE_SESION) == TELM_SESION_ACTIVE)
    {
        pSendBufer = pvPortMalloc(128);
        if(pSendBufer == NULL)
        {
            return ;
        }
        len = TelmProt_Encode(pSendBufer, TELM_LOGIN,MSG_ENCRYPT_AES, 128);
        if(len == 0)
        {
            DEBUG(DEBUG_MEDIUM, "[RECORD] Encode login data length 0\r\n");
        }
        else if(IOT_IpNetSend(TELM_ENTERPRISE_SESION,pSendBufer, len, NULL ))
        {
            logtimeout = pdMS_TO_TICKS(60000) + xTaskGetTickCount();
            loginTimes++;
            TelmProt_setSesionState(TELM_ENTERPRISE_SESION, TELM_SESION_LOGINING);
        }
        else
        {
            vPortFree(pSendBufer);
            pSendBufer = NULL;
            return ;
        }
    }
    else if(TelmProt_getSesionState(TELM_ENTERPRISE_SESION) == TELM_SESION_LOGINING)
    {
        if(loginTimes >= 3)
        {
            loginTimes = 0xFF;
            logtimeout = pdMS_TO_TICKS(60000*30) + xTaskGetTickCount();
            TelmProt_setSesionState(TELM_ENTERPRISE_SESION, TELM_SESION_ACTIVE);
            IOT_IpClose(TELM_ENTERPRISE_SESION);
        }
        else if(logtimeout <= xTaskGetTickCount())
        {
            TelmProt_setSesionState(TELM_ENTERPRISE_SESION, TELM_SESION_ACTIVE);
        }
    }
    else if(TelmProt_getSesionState(TELM_ENTERPRISE_SESION) == TELM_SESION_LOGINED)
    {
        loginTimes = 0;
    }
    //check GB platform
    if(gbLogTimeout == 0xFF)
    {
        if(gbLogTimeout <= xTaskGetTickCount())
        {
            gbLogTimeout = 0;
        }
    }
    if(gbLogTimeout != 0xFF && TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_ACTIVE)
    {
        if(pSendBufer == NULL)
        {
            pSendBufer = pvPortMalloc(sizeof(TelmRealtime_t));
            if(pSendBufer == NULL)
            {
                return ;
            }
        }
        len = TelmProt_Encode(pSendBufer, TELM_LOGIN,MSG_ENCRYPT_AES, 128);
        if(IOT_IpNetSend(TELM_GB_SESION,pSendBufer, len, NULL ))
        {
            gbLogTimeout = pdMS_TO_TICKS(60000) + xTaskGetTickCount();
            loginTimes++;
            TelmProt_setSesionState(TELM_GB_SESION, TELM_SESION_LOGINING);
        }
    }
    else if(TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_LOGINING)
    {
        if(gbLoginTimes >= 3)
        {
            loginTimes = 0xFF;
            logtimeout = pdMS_TO_TICKS(60000*30) + xTaskGetTickCount();
            TelmProt_setSesionState(TELM_GB_SESION, TELM_SESION_ACTIVE);
        }
        else if(logtimeout <= xTaskGetTickCount())
        {
            
            TelmProt_setSesionState(TELM_GB_SESION, TELM_SESION_ACTIVE);
        }
    }
    else if(TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_LOGINED)
    {
        gbLoginTimes = 0;
    }
    if(pSendBufer)
    {
        vPortFree(pSendBufer);
        pSendBufer = NULL;
    }
}

static void heartBeatUploader(void)
{
    uint8_t *pSendBufer = NULL;
    uint16_t len = 0;
    if(TelmProt_getSesionState(TELM_ENTERPRISE_SESION) == TELM_SESION_LOGINED)
    {
        if(heartbeatTimer <= xTaskGetTickCount())
        {
            if(pSendBufer == NULL)
            {
                pSendBufer = pvPortMalloc(128);
                if(pSendBufer == NULL)
                {
                    return ;
                }
            }
            len = TelmProt_Encode(pSendBufer, TELM_HEARTBEAT,MSG_ENCRYPT_NONE, 128);
            if(len == 0)
            {
                DEBUG(DEBUG_HIGH, "[RECORD] encode heartbeat error:len=0\r\n");
            }
            else if(IOT_IpNetSend(TELM_ENTERPRISE_SESION, pSendBufer, len, NULL ))
            {
                 heartbeatTimer = pdMS_TO_TICKS(DEFAULT_HEART_BEART_INTERVAL) + xTaskGetTickCount();
            }           
        }
    }

#if 0 //gp platform do not need heartbeat
    if(TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_LOGINED)
    {
        if(heartbeatTimer <= xTaskGetTickCount())
        {
            if(pSendBufer == NULL)
            {
                pSendBufer = pvPortMalloc(sizeof(VHCL_log_t));
                if(pSendBufer == NULL)
                {
                    return ;
                }
            }
            pSendBufer->len = TelmProt_Encode(pSendBufer->data, TELM_HEARTBEAT,MSG_ENCRYPT_NONE, 256);
            if(pSendBufer->len == 0)
            {
                
            }
            if(!IOT_IpNetSend(TELM_ENTERPRISE_SESION, pSendBufer->data, pSendBufer->len, NULL ))
            {
                 heartbeatTimer = pdMS_TO_TICKS(DEFAULT_HEART_BEART_INTERVAL) + xTaskGetTickCount();
            }           
        }
    }
#endif
    if(pSendBufer)
    {
        vPortFree(pSendBufer);
        pSendBufer = NULL;     
    }
}

static void RecordUploader(void)
{
    if(TelmProt_getSesionState(TELM_ENTERPRISE_SESION) == TELM_SESION_LOGINED)
    {
        //check back up data
        if(Get_vhcl_Data_Total_Number())
        {
            if(pBackupTmp == NULL)
            {
                pBackupTmp = pvPortMalloc(sizeof(dataTmp_t));
                if(pBackupTmp == NULL)
                {
                    DEBUG(DEBUG_HIGH, "[RECORD] Malloc error. Line:%d\r\n", __LINE__);
                    return ;
                }
                if(!Read_vhcl_Data(&pBackupTmp->data, 1))
                {
                    vPortFree(pBackupTmp);
                    pBackupTmp = NULL;
                    return ;
                }
                pBackupTmp->state = dataInit;
                pBackupTmp->tryCnt = 0;
                pBackupTmp->transTimer = ~0;
            }
            if(pBackupTmp->transTimer <= xTaskGetTickCount() && pBackupTmp->state == dataTrans)
            {
                if(pBackupTmp->tryCnt >= RECORD_SEND_MAX_CNT)
                {
                    //delete current data if have send RECORD_SEND_MAX_CNT times
                    vPortFree(pBackupTmp);
                    pBackupTmp = NULL;
                    return ;
                }
                //timeout
                pBackupTmp->state = dataInit;
            }
            if(pBackupTmp->state == dataInit)
            {
                if(IOT_IpNetSend(TELM_ENTERPRISE_SESION, pBackupTmp->data.data, pBackupTmp->data.len, backupSendResHandle ))
                {
                    if(TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_LOGINED)
                    {
                        if(!IOT_IpNetSend(TELM_GB_SESION, pBackupTmp->data.data, pBackupTmp->data.len, NULL ))
                        {
                            //if gb platform need to send?
                        }
                    }
                    pBackupTmp->state = dataTrans;
                    pBackupTmp->tryCnt++;
                    pBackupTmp->transTimer = pdMS_TO_TICKS(RECORD_SEND_TIMEOUT) + xTaskGetTickCount();
                }                
            }
        }
        //check realtime
        else if(pRealtimeTmp)
        {

            if(pRealtimeTmp->transTimer <= xTaskGetTickCount() && pRealtimeTmp->state == dataTrans)
            {
                if(pRealtimeTmp->tryCnt >= RECORD_SEND_MAX_CNT)
                {
                    //delete current data if have send RECORD_SEND_MAX_CNT times
                    vPortFree(pRealtimeTmp);
                    pRealtimeTmp = NULL;     
                    return ;
                }
                pRealtimeTmp->state = dataInit;
            }
            if(pRealtimeTmp->state == dataInit)
            {
                if(pdPASS == IOT_IpNetSend(TELM_ENTERPRISE_SESION, pRealtimeTmp->data.data, pRealtimeTmp->data.len, realtimeSendResHandle ))
                {
                        if(TelmProt_getSesionState(TELM_GB_SESION) == TELM_SESION_LOGINED)
                        {
                            IOT_IpNetSend(TELM_GB_SESION, pRealtimeTmp->data.data, pRealtimeTmp->data.len, NULL );
                        }
                        pRealtimeTmp->state = dataTrans;
                        pRealtimeTmp->tryCnt++;
                        pRealtimeTmp->transTimer = pdMS_TO_TICKS(RECORD_SEND_TIMEOUT) + xTaskGetTickCount();
                }
            }
        }
    }
}

static void recordRlealtimeData(void)
{
    if(recordTimer <= xTaskGetTickCount())
    {
        if(pRealtimeTmp == NULL)
        {
            pRealtimeTmp = pvPortMalloc(sizeof(dataTmp_t));
            if(pRealtimeTmp)
            {
                recordTimer = pdMS_TO_TICKS(recordInteral) + xTaskGetTickCount();
                pRealtimeTmp->data.len = TelmProt_Encode(pRealtimeTmp->data.data, TELM_REALTIME,MSG_ENCRYPT_AES, 512);
                if(pRealtimeTmp->data.len == 0)
                {
                    vPortFree(pRealtimeTmp);
                    pRealtimeTmp = NULL;
                    return ;
                }
                pRealtimeTmp->state = dataInit;
                pRealtimeTmp->tryCnt = 0;
                pRealtimeTmp->transTimer = ~0;
            }
        }
        else
        {
            //backup data
            TelmRealtime_t *pSendBufer = pvPortMalloc(sizeof(TelmRealtime_t));
            if(pSendBufer == NULL)
            {
                DEBUG(DEBUG_HIGH, "[RECORD] Malloc err. Line:%d\r\n", __LINE__);
                return ;
            }
            pSendBufer->len = TelmProt_Encode(pSendBufer->data, TELM_REALTIME,MSG_ENCRYPT_AES, 512);
            recordTimer = pdMS_TO_TICKS(recordInteral) + xTaskGetTickCount();
            TelmProt_backupData(pSendBufer);
            vPortFree(pSendBufer);
            pSendBufer = NULL;
        }
    }
}

static void realtimeSendResHandle(bool sendRes, uint8_t channel)
{
    if(sendRes)
    {
        if(pRealtimeTmp)
        {
             vPortFree(pRealtimeTmp);
             pRealtimeTmp = NULL;
        }
        else
        {
            
        }
    }
    else
    {
        if(pRealtimeTmp)
        {
            pRealtimeTmp->state = dataInit;
        }
    }
}
static void backupSendResHandle(bool sendRes, uint8_t channel)
{
    if(sendRes)
    {
        //delte first data in flash
        if(pBackupTmp)
        {
            vPortFree(pBackupTmp);
            pBackupTmp = NULL;
            Set_vhcl_Data_Next_Read_Pointer(1);
        }
    }
    else
    {
        pBackupTmp->state = dataInit;
    }
}

static void prvRecord_evt_nop(int16_t data)
{
}
/*******************************************************************************
*    Function:  prvRecord_evt_rltmTalRefresh
*
*  Parameters:  'data' do not use.
*     Returns:  None.
* Description:  Refresh the realtime data table 
*******************************************************************************/
static void prvRecord_evt_rltmTalRefresh(int16_t data)
{
#if 1
    TelmProt_Encode_Vehicle((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].vhclInifo,
        sizeof(Telm_Data_Vehicle));
    TelmProt_Encode_GPS((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].gpsInfo,
        sizeof(Telm_Data_Position));
    TelmProt_Encode_EcEngine((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].mtrInfo,
        sizeof(Telm_Data_Ec_Engine));
    TelmProt_Encode_AbsVal((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].extrValInfo,
         sizeof(Telm_Data_Abs_Val));
    TelmProt_Encode_Alarm((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].alrmInfo,
         sizeof(Telm_Data_Alarm));
    TelmProt_Encode_Batt_Volt((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].batVoltInfo,
        sizeof(Telm_Data_Batt_Volt));
    TelmProt_Encode_Batt_Temp((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].batTempInfo,
         sizeof(Telm_Data_Batt_Temp));
    TelmProt_Encode_Extended((uint8_t *const)&rltmInfoTbl.rltmInfo[rltmInfoTbl.idx].extInof,
         sizeof(Telm_Data_Extended));
    rltmInfoTbl.idx++;

    if(rltmInfoTbl.idx == RECORD_ALRM_BCKP_CNT)
    {
        rltmInfoTbl.idx = 0;
        rltmInfoTbl.isFull = 1;
    }
#endif
}
/*******************************************************************************
*    Function:  prvRecord_evt_warningLevel3
*
*  Parameters:  'data' do not use.
*     Returns:  None.
* Description:  Handle level 3 alarm
*******************************************************************************/
static void prvRecord_evt_Level3Alarm(int16_t data)
{
}

static void prvRecord_evt_CleanRecord(int16_t data)
{
    if(pBackupTmp)
    {
        if(pBackupTmp->state != dataTrans)
        {
            vPortFree(pBackupTmp);
            pBackupTmp = NULL;
        }
    }
    if(pRealtimeTmp)
    {
        if(pRealtimeTmp->state != dataTrans)
        {
            vPortFree(pRealtimeTmp);
            pRealtimeTmp = NULL;
        }
    }
    Init_vhcl_Info();
}

static void disConnetOk(bool isOk, uint8_t chanel)
{
	IOT_IpClose((uint8_t)TELM_GB_SESION);
	TelmProt_setSesionState(TELM_GB_SESION, TELM_SESION_INACTIVE);
}

static void prvRecord_evt_disConGb(int16_t data)
{
	uint8_t *pSendBuff = pvPortMalloc(128);
	uint16_t len = 0;
	if(pSendBuff)
	{
		len = TelmProt_Encode(pSendBuff, TELM_LOGOUT, MSG_ENCRYPT_NONE, 128);
		IOT_IpNetSend(TELM_GB_SESION, pSendBuff, len, disConnetOk );
		TelmProt_setSesionState(TELM_GB_SESION, TELM_SESION_LOGOUTING);
	}
}

/*=====================================================================*\
 * File Revision History
 *======================================================================
 *
 * Date        userid  (Description on following lines:)
 * ----------- ------  ---------------------------------------------
 *
  =====================================================================*/

