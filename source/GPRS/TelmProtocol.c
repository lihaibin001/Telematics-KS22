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
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include "TelmProtocol.h"
#include "GPRS.h"
//#include "ATProtocol.h"
//#include "ATApp.h"
#include <stdio.h>
#include "vehicle.h"
#include "Debug.h"
#include "rtc.h"
#include "vehicle.h"
#include "aes.h"
#include "ACC_Detect.h"
#include "BMS_Detect.h"
#include <string.h>
#include "prj_config.h"
/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/     
#define TELM_SERVER_DATA_MAX_LEN    (uint16_t)256
//command flag     
#define	TELM_INFO_ID_LOGIN			(uint8_t)(0x01)
#define	TELM_INFO_ID_REALTIME		(uint8_t)(0x02)
#define	TELM_INFO_ID_BACKUP			(uint8_t)(0x03)
#define	TELM_INFO_ID_LOGOUT	    	(uint8_t)(0x04)
#define TELM_INFO_ID_HRTBEAT        (uint8_t)0x07
#define TELM_INFO_ID_TIMING         (uint8_t)0x08
#define TELM_INFO_ID_QUERY          (uint8_t)0x80
#define TELM_INFO_ID_CONFIG         (uint8_t)0x81
#define TELM_INFO_ID_CTRL           (uint8_t)0x82
     
#define TELM_INFO_ID_LOCATION       (uint8_t)0x83
#define TELM_INFO_ID_ASNY_UPLOAD    (uint8_t)0x09
//ack flag
#define TELM_ACK_SUCESS             (uint8_t)0x01     
#define TELM_ACK_FAIL               (uint8_t)0x02
#define TELM_ACK_VIN_DUPLICATION    (uint8_t)0x03
#define TELM_ACK_CMD                (uint8_t)0xFE
     
#define	TELM_INFO_ID_GB_UPLOAD_MAX	(0x05)

#define	TELM_INFO_ID_HEARTBEAT  	(0x07)
#define	TELM_INFO_ID_ASYNC_UPLOAD	(0x09)

#define	TELM_COMM_ID_CTL			(0x82)
#define	TELM_COMM_ID_POSITON		(0x83)

#define	TELM_INFO_CMD_REQ           (0x01)
#define	TELM_INFO_CMD_RESP          (0x02)
#define	TELM_INFO_CMD_EXEC          (0x04)

#define OTA_MAX_LEN                 (258048)

#define MAX_SUPPORTED_PID_PER_PACK  (26)

#define SPEED_DETECT_INTERVAL       (6) //3S is 6 times.
#define GPS_UPLOAD_MIN_INTERVAL     (3)

#define GPS_UPLOAD_BUFFER_MAX       (7)
#define MAX_BACKUP_GPS_ONE_PACK     (7)

#define HEADER_STX                  (0x23)

#define HEADER_ACK_SUCCESS_FLAG     (0x01)
#define HEADER_ACK_ERROR_FLAG       (0x02)
#define HEADER_ACK_VIN_ERR_FLAG     (0x03)
#define HEADER_ACK_CMD_FLAG         (0xFE)

#define HEADER_ENC_NONE_FLAG        (0x01)
#define HEADER_ENC_RSA_FLAG         (0x02)
#define HEADER_ENC_AES_FLAG         (0x03)
#define HEADER_ENC_ABNORMAL_FLAG    (0xFE)
#define HEADER_ENC_INVALID_FLAG     (0xFF)

#define MAX_BACKUP_GPS_UPLOAD_NUM (1)

#define TELM_REALTIME_DATA_LEN  512
#define TELM_LOG_DATA_LEN       128

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static void Get_Time(uint8_t *time_struct);
static uint16_t prvTelmProt_Aes_Decrypt(uint8_t *output, 
                                        uint8_t *input, 
                                        uint16_t length);

static uint8_t prvTelmProt_Encode_Header(TELM_DataType_t cmdType,
                                         TELM_AckType_t ackType,
                                         uint8_t* const encoded, 
                                         uint16_t len, 
                                         Msg_Encrypt_Type_t encryptType);
static uint8_t prvTelmProt_Encode_Chk(uint8_t *checksum, 
                                      uint8_t const *encoded, 
                                      uint16_t size);

static uint8_t prvTelmProt_Encode_Login(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Logout(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Vehicle(uint8_t* const encoded, 
                                         uint16_t bufSize);
static uint8_t prvTelmProt_Encode_EcEngine(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_FuelBatt(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Engine(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_GPS(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_AbsVal(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Alarm(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Resp(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Batt_Volt(uint8_t* const encoded,uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Batt_Temp(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Extended(uint8_t* const encoded, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_HeartBeat(uint8_t* const encoded, uint16_t bufSize);
/**********************************************************************
 * Variables with File Level Scope
 *********************************************************************/

//encode table
static uint8_t uploadDataTable[TELM_NUM][TELM_INFO_IN_PACK_MAX_NUM] = {
    {TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},
    {TELM_INFO_LOGIN,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_LOGIN
    {TELM_INFO_LOGOUT,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_LOGOUT
    {TELM_INFO_VEHICLE_DATA,TELM_INFO_EC_ENGINE_DATA,TELM_INFO_GPS_DATA,TELM_INFO_ABS_DATA,TELM_INFO_ALARM_DATA,TELM_INFO_BATT_VOLT,TELM_INFO_BATT_TEMP,TELM_INFO_EXTENDED},  // TELM_SEC_EVT_REALTIME
//    {TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_BACKUP
    {TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_HEARTBEAT
    {TELM_INFO_RESP,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_ENC_KEY
};
static uint8_t ota_packet_flag = 0;
static uint16_t loginSn;
static uint8_t sesionState[TELM_SESION_NUM];
static ipSendCb respHandler = NULL;
static uint8_t *pTmpKey = NULL;
static bool encodeLock;
/**********************************************************************
 * Private variable with File Level Scope
 *********************************************************************/
static const Telm_Info_Encode_Fun telm_info_encode_table[] =
{
    prvTelmProt_Encode_Login,
    prvTelmProt_Encode_Logout,
    prvTelmProt_Encode_Vehicle,
    prvTelmProt_Encode_EcEngine,
    prvTelmProt_Encode_FuelBatt,
    prvTelmProt_Encode_Engine,
    prvTelmProt_Encode_GPS,
    prvTelmProt_Encode_AbsVal,
    prvTelmProt_Encode_Alarm,
    prvTelmProt_Encode_Resp,
    prvTelmProt_Encode_Batt_Volt,
    prvTelmProt_Encode_Batt_Temp,
    prvTelmProt_Encode_Extended,
    prvTelmProt_Encode_HeartBeat,
};

static void prvTelmProt_ChangeKey(bool res, uint8_t channel)
{
    if(res)
    {
        if(pTmpKey != NULL)
        {
            Set_config_secretKey(pTmpKey);
            vPortFree(pTmpKey);
            pTmpKey = NULL;
        }
        else
        {
            DEBUG(DEBUG_HIGH, "[TELM] pTmpKey = NULL\r\n");
        }
    }
    encodeLock = false;
    TelmProt_setSesionState((TelmSesionNum_t)channel, TELM_SESION_ACTIVE);
}

//parse 0x82 command
static bool prvTelmProt_ParseControlCmd(uint8_t *pData, uint16_t dataLen)
{
    switch(pData[6])
    {
        case 0x80: //change key
            if(pTmpKey == NULL)
            {
                encodeLock = true;
                pTmpKey = pvPortMalloc(16);
                if(pTmpKey == NULL)
                {
                    DEBUG(DEBUG_HIGH, "[TELM] Malloc err. Line:%d\r\n",__LINE__);
                    return false;
                }
            }
            
            memcpy(pTmpKey, pData+7, 16);
            OS_Send_Message(OS_RECORD_TASK, Build_Message(RECORD_EVT_CLEAN_RECORD, 0));
            respHandler = prvTelmProt_ChangeKey;
            return true;
        case 0x07: //open sample link
            return IOT_IpOpenUnblock((uint8_t)TELM_GB_SESION);
        case 0x8E: //close sample link
        	//return IOT_IpClose((uint8_t)TELM_GB_SESION);
        	 OS_Send_Message(OS_RECORD_TASK, Build_Message(RECORD_EVT_CLOSE_GB, 0));
        	 return true;
        default:
            break;
    }
    return false;
}

//parse 0x83
static bool prvTelmProt_ParseLocatCmd(uint8_t *pData, uint16_t dataLen)
{
    return false;
}

extern void vTelmProt_ParseIpData(uint8_t *pData, uint16_t len, uint8_t sesionNum)
{
    Telm_data_header_t *pHeader = (Telm_data_header_t *)pData;
    uint8_t *pBody = &pData[sizeof(Telm_data_header_t)];
    uint16_t bodyLen = 0;
    uint16_t dataLen = 0;
    uint8_t *pDataTmp = NULL;
    uint8_t chk_tmp;

    bool isDataRight = false;
    if(pData == NULL || sesionNum >= 2)
    {
        return ;
    }

    if (len < sizeof(Telm_data_header_t)+1 || (pHeader->start != 0x2323))
    {
        return ;
    }

    bodyLen = (pHeader->len[0] << 8) + pHeader->len[1];  
    prvTelmProt_Encode_Chk(&chk_tmp, pData+2, len-3);
#if 0
    if ((chk_tmp != *(pData + len - 1)))
    {
        return ;
    }
#endif
    
    if(pHeader->ackFalg == 0x01) //positive resp from server
    {
        switch(pHeader->cmdType)
        {
            case 0x01:
                sesionState[sesionNum] = (uint8_t)TELM_SESION_LOGINED;
                break;;
            case 0x04:
                sesionState[sesionNum] = (uint8_t)TELM_SESION_LOGOUTED;
                break;
            default:
                break;    
        }
        return ;   //if message if response, return
    }
    
    if(pHeader->ackFalg == 0x02 || pHeader->ackFalg == 0x03) //negative resp from server
    {
        switch(pHeader->cmdType)
        {
            case 0x01:
                sesionState[sesionNum] = (uint8_t)TELM_SESION_ACTIVE;
                break;
            case 0x04:
                sesionState[sesionNum] = (uint8_t)TELM_SESION_LOGINED;
                break;
            default:
                break;
        }
        return ;//if message if response, return 
    }
    
    //parse body
    if (pHeader->cmdType != 0)
    {
        switch(pHeader->encType)
        {
            case 0x01:  //No encryption
                dataLen = bodyLen;
                break;
            case 0x02:  //RSA encryption
                break;
            case 0x03:  //AES128 encryption
                pDataTmp = pvPortMalloc(bodyLen);
                if(pDataTmp == NULL)
                {
                    DEBUG(DEBUG_HIGH, "[TELM] Malloc error. Line:%d\r\n", __LINE__);
                    return ;
                }
                dataLen = prvTelmProt_Aes_Decrypt(pDataTmp, pBody, bodyLen);
                if(dataLen != 0)
                {
                    //convert to hex
                    uint16_t idx;
                    dataLen /= 2;
                    for(idx=0; idx<dataLen; idx++)
                    {
                        pBody[idx] = StrtoHex(pDataTmp+idx*2);
                    }
                }
#if 0
                if(dataLen != 0)
                {
                    memcpy(pBody, pDataTmp, dataLen);
                    vPortFree(pDataTmp);
                    pDataTmp = NULL;
                }
                else
                {
                    //decrpytion error
                    vPortFree(pDataTmp);
                    pDataTmp = NULL;
                    return ;
                }
#endif
                break;
            default:
                break;
        }
        switch(pHeader->cmdType)
        {
            case 0x82:
                isDataRight = prvTelmProt_ParseControlCmd(pBody, dataLen);
                bodyLen = 7;
                break;
            case 0x83:
                isDataRight = prvTelmProt_ParseLocatCmd(pBody, dataLen);
                //encode location
                bodyLen = TelmProt_Encode_GPS(pBody+7,bodyLen-7);
                if(bodyLen == 0)
                {
                    DEBUG(DEBUG_HIGH, "[TELM] Encode GPS error\r\n");
                }
                bodyLen += 7;
                break;
            default:
                break;
        }
        //response
        if(isDataRight)
        {
            //encode response
            pHeader->ackFalg = 0x01;
            pHeader->len[0] = 0;
            pHeader->len[1] = 7;
            pHeader->encType = 0;
            Get_Time(pBody); //flush time
            //encode checksum
            prvTelmProt_Encode_Chk(&pData[sizeof(Telm_data_header_t) + bodyLen], pData+2, sizeof(Telm_data_header_t) + bodyLen -2);
            IOT_IpNetSend(sesionNum, pData, sizeof(Telm_data_header_t)+bodyLen+1,respHandler);
            respHandler = NULL;
        }
        if(pDataTmp)
        {
        	vPortFree(pDataTmp);
        	pDataTmp = NULL;
        }
    }
}


// return length, AES 128 encode
static uint16_t prvTelmProt_Aes_Encrypt(uint8_t *output, 
                                        uint8_t *input, 
                                        uint16_t length)
{
    uint16_t ret=0;
    mbedtls_aes_context aes_ctx;
    uint16_t remain_len=length;
    uint16_t i=0;
    uint8_t tmp_data[16];
    uint8_t secretKey[16] = "";
    Get_config_secretKey(secretKey);
    mbedtls_aes_init(&aes_ctx);

    mbedtls_aes_setkey_enc(&aes_ctx, secretKey, 128);
    
    for (i=0; i<length; i+=16)
    { 
        if (remain_len < 16)
        {
            //PKCS5 padding
            uint8_t pkcs5_padding = (uint8_t)(16 - remain_len);
            uint8_t j;
            memcpy(tmp_data, input+i, remain_len);
            for(j = remain_len; j < 16; j++)
            {
                tmp_data[j] = pkcs5_padding;
            }
        }
        else 
        {
            memcpy(tmp_data, input+i, 16);
            remain_len -= 16;
        }
#if 1
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, 
                              tmp_data, output+i);
#endif 
        ret+=16;
    }
    if(remain_len == 0)
    {
        memset(tmp_data, 0x0F, 16);
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, 
                              tmp_data, output+i);
        ret+=16;
    }
    return ret;
}

// return length, AES 128 decode
static uint16_t prvTelmProt_Aes_Decrypt(uint8_t *output, 
                                        uint8_t *input, 
                                        uint16_t length)
{
    uint16_t ret=0;
    mbedtls_aes_context aes_ctx;
    uint16_t i=0;
    uint8_t secretKey[16] = "";
    Get_config_secretKey(secretKey);
    mbedtls_aes_init(&aes_ctx);

    mbedtls_aes_setkey_dec(&aes_ctx, secretKey, 128);

    for (i=0;i<length;i+=16)
    {
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_DECRYPT, input+i, output+i);
        ret+=16;
    }
    ret -= output[i-1];
    return ret;
}



static uint8_t prvTelmProt_Encode_Chk(uint8_t *checksum, 
                                      uint8_t const *encoded, 
                                      uint16_t size)
{
    uint16_t i=0;
    uint8_t checksum_tmp=0;
    for (i=0;i<size;i++)
    {
        checksum_tmp ^= *(encoded+i);
    }
    *checksum=checksum_tmp;
    return 1;
}

static void Get_Time(uint8_t *time_struct)
{
    RTC_GetTime(time_struct);
}

static void Get_Iccid(uint8_t *iccid)
{
    uint8_t iccidHex[10];
    uint8_t iccidTmp[20];
    IOT_GetIccid(iccidHex);
    uint8_t idx;
    for(idx=0; idx<10; idx++)
    {
        HextoChar(iccidHex[idx], &iccidTmp[idx*2]);
    }
    memcpy(iccid, iccidTmp, 20);
}

static uint8_t prvTelmProt_Encode_Login(uint8_t* const encoded, uint16_t bufSize)
{
    uint8_t rt = TELM_INFO_LEN_LOGIN;
    if(rt > bufSize)
    {
        return 0;
    }
    Telm_Data_Login *tmp_data = (Telm_Data_Login *)encoded;
    Get_Time(tmp_data->structData.time);
    tmp_data->structData.daily_index[0] = (loginSn >> 8) & 0xFF;
    tmp_data->structData.daily_index[1] = loginSn & 0xFF;
    Get_Iccid(tmp_data->structData.iccid);
    tmp_data->structData.charge_sys = 1;
    tmp_data->structData.charge_code_len = 1;
    *(encoded+rt) = '0';
    rt++;
    return rt;
}

static uint8_t prvTelmProt_Encode_Logout(uint8_t* const encoded, uint16_t bufSize)
{
    uint8_t rt=TELM_INFO_LEN_LOGOUT;
    if(rt > bufSize)
    {
        return 0;
    }
    Telm_Data_Logout *tmp_data=(Telm_Data_Logout *)encoded;
    Get_Time(tmp_data->structData.time);
    tmp_data->structData.daily_index[0] = (loginSn >> 8) & 0xFF;
    tmp_data->structData.daily_index[1] = loginSn & 0xFF;
    loginSn++;
    if(loginSn >65531)
    {
        loginSn = 0;
    }
    return rt;
}
/*******************************************************************************
*    Function: prvTelmProt_Encode_Vehicle
*
*  Parameters: 'uploadInfo' not used
*               'encode' point to encoded data;
*               'bufSize' not used
*     Returns: None
* Description: Encode the vehicle's information data
*******************************************************************************/
static uint8_t prvTelmProt_Encode_Vehicle(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Vehicle);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->vhclInifo, len); 
        return len;
    }
    return 0;
}
static uint8_t prvTelmProt_Encode_EcEngine(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Ec_Engine);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->mtrInfo, len);
        return len;
    }
    return 0;
}
static uint8_t prvTelmProt_Encode_FuelBatt(uint8_t* const encoded, uint16_t bufSize)
{
    return 0;
}
static uint8_t prvTelmProt_Encode_Engine(uint8_t* const encoded, uint16_t bufSize)
{
    return 0;
}
static uint8_t prvTelmProt_Encode_GPS(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Position);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->gpsInfo, len);
        return len;
    }
    return 0;
}
static uint8_t prvTelmProt_Encode_AbsVal(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Abs_Val);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->extrValInfo, len);
        return len;
    }
    return 0;    
}
static uint8_t prvTelmProt_Encode_Alarm(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Alarm);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->alrmInfo, len);
        return len;
    }
    return 0;
}

static uint8_t prvTelmProt_Encode_Resp(uint8_t* const encoded, uint16_t bufSize)
{
    RTC_GetTime(encoded);
    
    return 7;
}

static uint8_t prvTelmProt_Encode_Batt_Volt( uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Batt_Volt);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->batVoltInfo, len);
        return len;
    }
    return 0;    
}
static uint8_t prvTelmProt_Encode_Batt_Temp(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Batt_Temp);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->batTempInfo, len);
        return len;
    }
    return 0;  
}

static uint8_t prvTelmProt_Encode_HeartBeat(uint8_t* const encoded, uint16_t bufSize)
{
    return 0;
}


static uint8_t prvTelmProt_Encode_Extended(uint8_t* const encoded, uint16_t bufSize)
{
    realTimeInfo_t *pData = Record_GetLastRltm();
    if(pData)
    {
        uint8_t len = sizeof(Telm_Data_Extended);
        if(len > bufSize)
        {
            return 0;
        }
        memcpy(encoded, &pData->extInof, len);
        return len;
    }
    return 0;      
}

/* Global function */
uint16_t TelmProt_Encode(uint8_t *pBuffer, 
                                   TELM_DataType_t dataType, 
                                   Msg_Encrypt_Type_t eccryptType,
                                   uint16_t bufferSzie)
{
    uint16_t dataLen = 0;
    uint16_t offset = 0;
    uint16_t tmp_ptn = 0;
    uint8_t cmdType = 0;
    uint16_t headLen = sizeof(Telm_data_header_t);
    uint8_t *pData = &pBuffer[headLen];
    uint8_t *tmp_data = pvPortMalloc(bufferSzie);
    uint8_t infoIndex;
    if(tmp_data == NULL)
    {
        DEBUG(DEBUG_HIGH, "[TP] Malloc error. Line:%d\r\n",__LINE__);
        return 0;
    }
    if(encodeLock)
    {
    	vPortFree(tmp_data);
    	tmp_data = NULL;
        return 0;
    }
    /* encode each type of information according to protocol */
    for (infoIndex = 0; infoIndex < TELM_INFO_IN_PACK_MAX_NUM; infoIndex++)
    {
        uint8_t encode_index = uploadDataTable[dataType][infoIndex];

        if (encode_index == TELM_INFO_NONE)
        {
            break;
        }
        else if(tmp_ptn < TELM_REALTIME_DATA_LEN)
        {
            offset = telm_info_encode_table[encode_index-1](&pData[tmp_ptn],
                                                            bufferSzie - tmp_ptn);
            if (offset == 0)
            {
                DEBUG(DEBUG_HIGH,"[TELM]: Encode empty message!\n\r");
                vPortFree(tmp_data);
                return 0;
            }
            tmp_ptn += offset;
        }
    }
#if 0
#if (DEBUG_LVL == DEBUG_LOW)
    {
        int i;
        DEBUG(DEBUG_LOW, "[TxSouce]:");
        for(i=0; i<tmp_ptn; i++)
        {
            DEBUG(DEBUG_LOW, "%02x",pData[i]);
        }
        DEBUG(DEBUG_LOW, "\r\n");
    }
#endif
#endif
    // convert hex to string

    // encrypt data
    switch(eccryptType)
    {
        case MSG_ENCRYPT_NONE:
            dataLen = tmp_ptn;
            break;
        case MSG_ENCRYPT_RSA:
            break;
        case MSG_ENCRYPT_AES:
            memcpy(tmp_data, pData, tmp_ptn);
            HexToStr(tmp_data, pData, tmp_ptn);
            tmp_ptn *= 2;
            dataLen = prvTelmProt_Aes_Encrypt(pData, tmp_data, tmp_ptn);
            break;
        default:
            DEBUG(DEBUG_HIGH, "[TELM] Eccrypt type error!\r\n");
            return 0;
    }
    switch(dataType)
    {
        case TELM_LOGIN:
            cmdType = 1;
            break;
        case TELM_LOGOUT:
            cmdType = 6;
            break;
        case TELM_REALTIME:
            cmdType = 2;
            break;
#if 0
        case TELM_BACKUP:
            cmdType = 3;
            break;
#endif
        case TELM_HEARTBEAT:
            cmdType = 7;
            break;
        default:
            break;
    }
    if(cmdType != 0)
    {    
        prvTelmProt_Encode_Header((TELM_DataType_t)cmdType, TELM_CMD, 
                                  pBuffer, dataLen, eccryptType);
        tmp_ptn = dataLen + headLen;

        prvTelmProt_Encode_Chk(pBuffer+tmp_ptn, pBuffer+2, (tmp_ptn-2));
        tmp_ptn += 1;
        
    }
    if(tmp_data)
    {
        vPortFree(tmp_data);
        tmp_data = NULL;
    }
    return tmp_ptn;
}

uint16_t TelmProt_Encode_Vehicle(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt=0;
    if(TELM_INFO_LEN_VEHICLE <= buffsize && encoded != NULL)
    {
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {
            Telm_Data_Vehicle *Vehicle=(Telm_Data_Vehicle *)encoded;
            RTC_GetTime(Vehicle->structData.time);
            Vehicle->structData.msg_id=0x01;
            if (ACC_GetStatus() == true)
            {
                Vehicle->structData.state=0x01;
            }
            else
            {
                Vehicle->structData.state=0x02;
            }
            switch(CanDataList[id_0xC0BA7F0].dataByte3)
            {
                case 0x00:
                    Vehicle->structData.charge_state = 0x03;
                    break;
                case 0x01:
                    Vehicle->structData.charge_state = 0x01;
                    break;
                default :
                    Vehicle->structData.charge_state = 0xFE;
                    break;
            }
            
            Vehicle->structData.run_state = 0x01;
         
            if(CanDataList[id_0xC0401D0].dataByte1 > 0x07)
            {
                Vehicle->structData.speed[0] = 0xFF;
                Vehicle->structData.speed[1] = 0xFE;
            }
            else
            {
                Vehicle->structData.speed[0] = CanDataList[id_0xC0401D0].dataByte1;
                Vehicle->structData.speed[1] = CanDataList[id_0xC0401D0].dataByte0;
            }
            
            if(CanDataList[id_0xC0401D0].dataByte5 > 0x05)
            {
                Vehicle->structData.mileage[0] = 0xFF;
                Vehicle->structData.mileage[1] = 0xFF;
                Vehicle->structData.mileage[2] = 0xFF;
                Vehicle->structData.mileage[3] = 0xFE;
            }
            else
            {
                Vehicle->structData.mileage[0] = CanDataList[id_0xC0401D0].dataByte5;
                Vehicle->structData.mileage[1] = CanDataList[id_0xC0401D0].dataByte4;
                Vehicle->structData.mileage[2] = CanDataList[id_0xC0401D0].dataByte3;
                Vehicle->structData.mileage[3] = CanDataList[id_0xC0401D0].dataByte2;
            }
            
            if(CanDataList[id_0x1801FFF4].dataByte1 > 0x27) //byte1 << 8 | byte0 > 10000
            {
                Vehicle->structData.voltage[0] = 0xFF;
                Vehicle->structData.voltage[1] = 0xFE;
            }
            else
            {
                Vehicle->structData.voltage[0] = CanDataList[id_0x1801FFF4].dataByte1;
                Vehicle->structData.voltage[1] = CanDataList[id_0x1801FFF4].dataByte0;
            }
            
            
            uint16_t current = CanDataList[id_0x1801FFF4].dataByte3 << 8 |  
                CanDataList[id_0x1801FFF4].dataByte2;
            
            current = current - 22000;//convert to GB data
            if(current > 20000)  //(dataByte3 << 8) | dataByte2 < 35535
            {
                Vehicle->structData.current[0] = 0xFE;
                Vehicle->structData.current[1] = 0xFF;
            }
            else
            {
                Vehicle->structData.current[0] = (current >> 8) & 0xFF ;
                Vehicle->structData.current[1] = current & 0xFF;
            }

            if(CanDataList[id_0x1801FFF4].dataByte5 > 0x03)
            {
                Vehicle->structData.soc =  0xFE;
            }
            else
            {
                Vehicle->structData.soc = ((CanDataList[id_0x1801FFF4].dataByte5 << 8) 
                                          | CanDataList[id_0x1801FFF4].dataByte4) / 10;
            }    
            

            Vehicle->structData.dc_state = 0xFF;

            switch(CanDataList[id_0xC0401D0].dataByte6 & 0x0F)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                    Vehicle->structData.shift = 
                        CanDataList[id_0xC0401D0].dataByte6 & 0x0F;
                    break;
                case 14:
                    Vehicle->structData.shift = 13;
                    break;
                case 15:
                    Vehicle->structData.shift = 14;
                    break;
                default :
                    Vehicle->structData.shift = 0xFE;
                    break;   
            }

            uint16_t  insulate_res = (((CanDataList[id_0x1804FFF4].dataByte1 << 8) 
                                       | CanDataList[id_0x1804FFF4].dataByte0)
                                     + ((CanDataList[id_0x1804FFF4].dataByte3 << 8) 
                                       | CanDataList[id_0x1804FFF4].dataByte2)) / 10;

            Vehicle->structData.insulate_res[0] = (insulate_res >> 8) & 0xFF;
            Vehicle->structData.insulate_res[1] = insulate_res & 0xFF;    
            
            if(CanDataList[id_0xC0BA7F0].dataByte4 > 100)
            {
                Vehicle->structData.accel_pedal = 0xFE;
            }
            else
            {
                Vehicle->structData.accel_pedal = 
                    CanDataList[id_0xC0BA7F0].dataByte4;
            }
            
            if(CanDataList[id_0xC0BA7F0].dataByte5 > 100)
            {
                Vehicle->structData.brake_pedal = 0xFE;        
            }
            else
            {
                Vehicle->structData.brake_pedal = 
                    CanDataList[id_0xC0BA7F0].dataByte5;
            }    
            
            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_VEHICLE;
        }
    }
    return rt;
}

uint16_t TelmProt_Encode_GPS(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt=0u;
    if(TELM_INFO_LEN_POSITION <= buffsize && encoded != NULL)
    {
        Telm_Data_Position *tmp_data=(Telm_Data_Position *)encoded;
        gps_data_t gps_data;
        vGps_Get_Gps_Info(&gps_data);
        tmp_data->structData.msg_id=0x05;
        if (gps_data.valid)
        {
            tmp_data->structData.state = 0x01;
            if(gps_data.east_or_west == 'W')
            {
                tmp_data->structData.state |= 0x02;
            }
            if(gps_data.north_or_sourth == 'S')
            {
                tmp_data->structData.state |= 0x40;
            }
            memcpy(tmp_data->structData.longitude,gps_data.longitude,4);
            memcpy(tmp_data->structData.latitude,gps_data.latitude,4);
        }
        else    
        {
            vGps_Get_Valid_Gps_Info(&gps_data);
            tmp_data->structData.state = 0; //gps data is not valid
            if(gps_data.east_or_west == 'W')
            {
                tmp_data->structData.state |= 0x02;
            }
            if(gps_data.north_or_sourth == 'S')
            {
                tmp_data->structData.state |= 0x40;
            }
            memcpy(tmp_data->structData.longitude,gps_data.longitude,4);
            memcpy(tmp_data->structData.latitude,gps_data.latitude,4);
        }

        rt = TELM_INFO_LEN_POSITION;
    }
    return rt;
}

uint16_t TelmProt_Encode_EcEngine(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt=0u;
    if(TELM_INFO_LEN_EC_ENGINE <= buffsize && encoded != NULL)
    {
        Telm_Data_Ec_Engine *motor = (Telm_Data_Ec_Engine *)encoded;

        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();

        if(CanDataList != NULL)
        {
            motor->structData.msg_id = 0x02;
            motor->structData.engine_num = 0x01;
            motor->structData.index = 1;
            
            /* motor state */
            motor->structData.state = 0x03;
            if(CanDataList[id_0xC08A7F0].dataByte7 & 0x01)
            {
                motor->structData.state = 0x04; /* Reay state */
            }
            if(CanDataList[id_0xC08A7F0].dataByte6 & 0x20)
            {
                motor->structData.state = 0x04;
                if(CanDataList[id_0xC08A7F0].dataByte6 & 0x08)
                {
                    motor->structData.state = 0x01; /* consumption state */
                }
                else
                {
                    motor->structData.state = 0x02; /* generate state */
                }
                
            }
            
             /* motor controller tempreture */
            if(CanDataList[id_0xC09A7F0].dataByte6 > 220) //base on vehicle
            {
                motor->structData.ctl_temp = 0xFE;
            }
            else
            {
                motor->structData.ctl_temp = CanDataList[id_0xC09A7F0].dataByte6;
            }  


            
            /* motor rpm */
            uint16_t rpm = (CanDataList[id_0xC08A7F0].dataByte3 << 8) | 
                CanDataList[id_0xC08A7F0].dataByte2; 
            
            if(rpm > 42000 || rpm < 32000)
            {
                motor->structData.rpm[0] = 0xFF;
                motor->structData.rpm[1] = 0xFE;
            }
            else
            {
                rpm = rpm - 32000 + 20000; //convert to GB data
                motor->structData.rpm[0] = (rpm >> 8) & 0xFF;
                motor->structData.rpm[1] = rpm & 0xFF;        
            }
            
            /* motor torque */
            uint16_t torque = (CanDataList[id_0xC08A7F0].dataByte1 << 8) | 
                CanDataList[id_0xC08A7F0].dataByte0;
            if(torque > 32110 || torque < 31890)
            {
                motor->structData.torque[0] = 0xFF;
                motor->structData.torque[1] = 0xFE;
            }
            else
            {
                torque = torque - 32000 + 20000; /* convert to GB data */
                motor->structData.torque[0] = torque >> 8 & 0xFF;
                motor->structData.torque[1] = torque & 0xFF;
            }
            
            /* motor tempreture */
            if(CanDataList[id_0xC09A7F0].dataByte7 > 220)
            {
                motor->structData.temp = 0xFE;
            }
            else
            {
                motor->structData.temp = CanDataList[id_0xC09A7F0].dataByte7;
            }
          
            /* controler voltage, can not get from vehicle!!!!*/
            motor->structData.ctl_vol[0] = 0xFF;
            motor->structData.ctl_vol[1] = 0xFF;
            
            /* current */
            uint16_t current = (CanDataList[id_0xC08A7F0].dataByte5 << 8) | 
                CanDataList[id_0xC08A7F0].dataByte4;
            if(current > 32500 || current < 31500)
            {
                motor->structData.ctl_current[0] = 0xFF;
                motor->structData.ctl_current[1] = 0xFE;
            }
            else
            {
                current = (current - 32000) * 10 + 10000;
                motor->structData.ctl_current[0] = (current >> 8) & 0xFF;
                motor->structData.ctl_current[1] = (current) & 0xFF;
            }          
            rt = TELM_INFO_LEN_EC_ENGINE;
            VHCL_CAN_GiveDataList();
        }
    }
    return rt;
}

uint16_t TelmProt_Encode_Engine(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt=0;
    if(buffsize >= TELM_INFO_LEN_ENGINE && encoded != NULL)
    {
        Telm_Data_Engine *tmp_data=(Telm_Data_Engine *)encoded;
        tmp_data->structData.msg_id=0x04;
        rt = TELM_INFO_LEN_ENGINE;
    }
    return rt;
}

uint16_t TelmProt_Encode_AbsVal(uint8_t* const encoded, uint16_t buffsize)
{ 
    uint16_t rt = 0;
    if(TELM_INFO_LEN_ABS_VAL <= buffsize && encoded != NULL)
    {
        Telm_Data_Abs_Val *extremum = (Telm_Data_Abs_Val *)encoded;
        //uint16_t tmpBuff16 = 0;
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {
  
            extremum->structData.msg_id = 0x06;
            /* sub-system number */
            if(CanDataList[id_0x1802FFF4].dataByte3 > 100 
               || CanDataList[id_0x1802FFF4].dataByte3 == 0)
            {
                extremum->structData.max_vol_sys_code = 0xFE;
            }
            else
            {
                extremum->structData.max_vol_sys_code = 
                    CanDataList[id_0x1802FFF4].dataByte3;
            }
            /* the code of single battery which has the maximum voltage */
            if(CanDataList[id_0x1802FFF4].dataByte2 > 100 
                || CanDataList[id_0x1802FFF4].dataByte2 == 0)
            {
                extremum->structData.max_vol_code = 0xFE;
            }
            else
            {
                extremum->structData.max_vol_code =
                    CanDataList[id_0x1802FFF4].dataByte2;
            }
            /* tht maximum voltage of single battery */
            //tmpBuff16 = CanDataList[id_0x1802FFF4].dataByte1 << 8;
            if((CanDataList[id_0x1802FFF4].dataByte1 << 8 
               || CanDataList[id_0x1802FFF4].dataByte0) > 10000 )
            {
                extremum->structData.max_vol[0] = 0xFE;
                extremum->structData.max_vol[1] = 0xFF;
            }
            else
            {
                extremum->structData.max_vol[0] = CanDataList[id_0x1802FFF4].dataByte1;
                extremum->structData.max_vol[1] = CanDataList[id_0x1802FFF4].dataByte0;
            }
            
            /* the sub-system code of the battery with the minimum voltage */
            if(CanDataList[id_0x1802FFF4].dataByte7 > 100 
               || CanDataList[id_0x1802FFF4].dataByte7 == 0)
            {
                extremum->structData.min_vol_sys_code = 0xFE;
            }
            else
            {
                extremum->structData.min_vol_sys_code = 
                    CanDataList[id_0x1802FFF4].dataByte7;
            }
            
            if(CanDataList[id_0x1802FFF4].dataByte6 > 100
               || CanDataList[id_0x1802FFF4].dataByte6 == 0 )
            {
                extremum->structData.min_vol_code = 0xFE;
            }
            else
            {
                extremum->structData.min_vol_code = 
                    CanDataList[id_0x1802FFF4].dataByte6;
            }
                
            if((CanDataList[id_0x1802FFF4].dataByte5 << 8
               | CanDataList[id_0x1802FFF4].dataByte4) > 10000)
            {
                extremum->structData.min_vol[0] = 0xFE;
                extremum->structData.min_vol[1] = 0xFF;
            }
            else
            {
                extremum->structData.min_vol[0] = 
                    CanDataList[id_0x1802FFF4].dataByte5;
                extremum->structData.min_vol[1] = 
                    CanDataList[id_0x1802FFF4].dataByte4;
            }
            
            /* maximum temperature */
            if(CanDataList[id_0x1803FFF4].dataByte2 > 100 
               || CanDataList[id_0x1803FFF4].dataByte2 == 0)
            {
                extremum->structData.max_temp_sys_code = 0xFE;
            }
            else
            {
                extremum->structData.max_temp_sys_code = 
                    CanDataList[id_0x1803FFF4].dataByte2;
            }
            
            if(CanDataList[id_0x1803FFF4].dataByte1 > 100 
               || CanDataList[id_0x1803FFF4].dataByte1 == 0)
            {
                extremum->structData.max_temp_code = 0xFE;
            }
            else
            {
                extremum->structData.max_temp_code = 
                    CanDataList[id_0x1803FFF4].dataByte1;
            }
            
            if(CanDataList[id_0x1803FFF4].dataByte0 > 250)
            {
                extremum->structData.max_temp = 0xFE;
            }
            else
            {
                extremum->structData.max_temp = CanDataList[id_0x1803FFF4].dataByte0;
            }
            
            /* minimum temperature */
            if(CanDataList[id_0x1803FFF4].dataByte5 > 100 
               || CanDataList[id_0x1803FFF4].dataByte5 == 0)
            {
                extremum->structData.min_temp_sys_code = 0xFE;
            }
            else
            {
                extremum->structData.min_temp_sys_code = 
                    CanDataList[id_0x1803FFF4].dataByte5;
            }
            
            if(CanDataList[id_0x1803FFF4].dataByte4 > 100 
               || CanDataList[id_0x1803FFF4].dataByte4 == 0)
            {
                extremum->structData.min_temp_code = 0xFE;
            }
            else
            {
                extremum->structData.min_temp_code = 
                    CanDataList[id_0x1803FFF4].dataByte4;
            }
            
            if(CanDataList[id_0x1803FFF4].dataByte3 > 250)
            {
                extremum->structData.min_temp = 0xFE;
            }
            else
            {
                extremum->structData.min_temp = CanDataList[id_0x1803FFF4].dataByte3;
            }    
            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_ABS_VAL;
        }
    }
    return rt;
}

uint16_t TelmProt_Encode_Alarm(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt=0;
    if(TELM_INFO_LEN_ALARM <= buffsize && encoded != NULL)
    {
        Telm_Data_Alarm *warningdata = (Telm_Data_Alarm *)encoded;
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {
            warningdata->structData.msg_id=0x07;
            warningdata->structData.alarm_level = 0xFF;
            warningdata->structData.temp_diff = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x80;
            warningdata->structData.batt_over_temp = 
                CanDataList[id_0x1801FFF4].dataByte7 & 0x08;
            warningdata->structData.batt_cell_over_vol = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x20;
            warningdata->structData.batt_under_vol = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x10;
            warningdata->structData.soc_under = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x01;
            warningdata->structData.batt_cell_over_vol = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x04;
            warningdata->structData.batt_cell_under_vol = 
                CanDataList[id_0x1801FFF4].dataByte6 & 0x02;
            warningdata->structData.soc_over = 
                CanDataList[id_0x1801FFF4].dataByte7 & 0x80;
            warningdata->structData.soc_leap = 0;
            warningdata->structData.batt_mismatch = 
                CanDataList[id_0x1801FFF4].dataByte7 & 0x40;
            warningdata->structData.batt_cell_disaccord = 
                CanDataList[id_0x1801FFF4].dataByte7 & 0x20;
            warningdata->structData.isolation = 
                CanDataList[id_0x1801FFF4].dataByte7 & 0x10;
            warningdata->structData.dc_temp = 0;
            warningdata->structData.brake = 0;
            warningdata->structData.dc_sta = 0;
            warningdata->structData.motor_ctrl_temp = 0;
            warningdata->structData.HVIL = 0;
            warningdata->structData.motor_temp = 0;
            warningdata->structData.batt_over_charge = 0;    
            
            encoded[6] = 0; //energy fault number
            encoded[7] = 0; //motor fault number
            encoded[8] = 0; //engine fault number
            encoded[9] = 0; //other fault number
          
            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_ALARM;
        }
    }
    return rt;
}

uint16_t TelmProt_Encode_FuelBatt(uint8_t* const encoded, uint16_t buffsize)
{
    uint8_t rt = 0;
    if(TELM_INFO_LEN_FUEL_BATT <= buffsize && encoded != NULL)
    {
        Telm_Data_Fuel_Batt *tmp_data=(Telm_Data_Fuel_Batt *)encoded;
        tmp_data->structData.msg_id=0x03;
        tmp_data->structData.tprobe_num[0]=0u;
        tmp_data->structData.tprobe_num[1]=1u;
        rt = TELM_INFO_LEN_FUEL_BATT;
    }
    return rt;
}

static bool singleBatVolPackge(Telm_Data_Batt_Volt *pBatt, 
                               uint8_t startNum,
                               uint8_t cellNum, 
                               const flexcan_frame_t *pData)
{
    uint8_t tmpData[8] = 
        {   pData->dataByte0,
            pData->dataByte1,
            pData->dataByte2,
            pData->dataByte3,
            pData->dataByte4,
            pData->dataByte5,
            pData->dataByte6,
            pData->dataByte7,
        };
    uint8_t idx = 0;
    if(cellNum > 4)
    {
        return false;
    }
    startNum *= 2;
    while(cellNum--)
    {
        if(((tmpData[idx+1] << 8) | tmpData[idx]) > 64255)
        {
            pBatt->structData.frame_batt_volt[startNum] = 0xFF;
            pBatt->structData.frame_batt_volt[startNum+1] = 0xFE;
        }
        else
        {
            pBatt->structData.frame_batt_volt[startNum] = tmpData[idx+1];
            pBatt->structData.frame_batt_volt[startNum+1] = tmpData[idx];
        } 
        idx += 2;
        startNum += 2;   
    }
    return true;
}

uint16_t TelmProt_Encode_Batt_Volt(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt = 0;
    if(TELM_INFO_LEN_BATT_VOLT <= buffsize && encoded != NULL)
    {
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {

            Telm_Data_Batt_Volt *tmp_data=(Telm_Data_Batt_Volt *)encoded;
            tmp_data->structData.msg_id=0x08;
            tmp_data->structData.sub_sys_id=1u;

            tmp_data->structData.single_batt_num[0] = (BATTERY_CELL_CNT >> 8) & 0xFF;
            tmp_data->structData.single_batt_num[1] = BATTERY_CELL_CNT & 0xFF;

            if(((CanDataList[id_0x1801FFF4].dataByte1<<8) | 
                CanDataList[id_0x1801FFF4].dataByte0) > 10000) //base on GB
            {
                tmp_data->structData.batt_volt[0] = 0xFF;
                tmp_data->structData.batt_volt[1] = 0xFE;
            }
            else
            {
                tmp_data->structData.batt_volt[0] = 
                    CanDataList[id_0x1801FFF4].dataByte1;
                tmp_data->structData.batt_volt[1] = 
                    CanDataList[id_0x1801FFF4].dataByte0;
            }
            
            uint16_t batt_current = (CanDataList[id_0x1801FFF4].dataByte3<<8) | 
                CanDataList[id_0x1801FFF4].dataByte2;
            batt_current = batt_current - 32000 + 10000; //convert to GB data
            if(batt_current > 20000)        //base on GB
            {
               tmp_data->structData.batt_current[0] = 0xFF;
               tmp_data->structData.batt_current[1] = 0xFE;
            }
            else
            {
                tmp_data->structData.batt_current[0] = 
                    (batt_current > 8) & 0xFF;
                tmp_data->structData.batt_current[1] = 
                    (batt_current) & 0xFF;
            }
            
            tmp_data->structData.batt_num = 1U;
            
            tmp_data->structData.frame_start_id[0]=0u;
            tmp_data->structData.frame_start_id[1]=1u;
                
            tmp_data->structData.frame_batt_total = 37U;
            
            /* single battery voltage */
            singleBatVolPackge(tmp_data, 0, 4, &CanDataList[id_0x1801D2F4]);
            singleBatVolPackge(tmp_data, 4, 4, &CanDataList[id_0x1802D2F4]);
            singleBatVolPackge(tmp_data, 8, 4, &CanDataList[id_0x1803D2F4]);
            singleBatVolPackge(tmp_data, 12, 4, &CanDataList[id_0x1804D2F4]);
            singleBatVolPackge(tmp_data, 16, 4, &CanDataList[id_0x1805D2F4]);
            singleBatVolPackge(tmp_data, 20, 4, &CanDataList[id_0x1806D2F4]);
            singleBatVolPackge(tmp_data, 24, 4, &CanDataList[id_0x1807D2F4]);
            singleBatVolPackge(tmp_data, 28, 4, &CanDataList[id_0x1808D2F4]);
            singleBatVolPackge(tmp_data, 32, 4, &CanDataList[id_0x1809D2F4]);
            singleBatVolPackge(tmp_data, 36, 1, &CanDataList[id_0x180AD2F4]);
           
            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_BATT_VOLT;
        }
    }
    return rt;
}

static bool singleBatTmpPackge(Telm_Data_Batt_Temp *pBatt, 
                               uint8_t startNum,
                               uint8_t cellNum, 
                               const flexcan_frame_t *pData)
{
    uint8_t tmpData[8] = 
        {   pData->dataByte0,
            pData->dataByte1,
            pData->dataByte2,
            pData->dataByte3,
            pData->dataByte4,
            pData->dataByte5,
            pData->dataByte6,
            pData->dataByte7,
        };
    uint8_t idx = 0;
    if(cellNum > 8)
    {
        return false;
    }
    while(cellNum--)
    {
        if(tmpData[idx] > 250)
        {
            pBatt->structData.frame_batt_temp[startNum] = 0xFE;
        }
        else
        {
            pBatt->structData.frame_batt_temp[startNum] = tmpData[idx];
        }
        
        idx++;
        startNum++;   
    }
    return true;
}

/*******************************************************************************
*    Function: ucTelmProt_Encode_Batt_Temp
*
*  Parameters: 'uploadInfo' not used
*               'encode' point to encoded data;
*               'bufSize' not used
*     Returns: None
* Description: Encode the battery's tempreture information
*******************************************************************************/
uint16_t TelmProt_Encode_Batt_Temp(uint8_t* const encoded, uint16_t buffsize)
{
    uint16_t rt = 0;
    if(TELM_INFO_LEN_BATT_TEMP <= buffsize && encoded != NULL)
    {
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {
            Telm_Data_Batt_Temp *tmp_data=(Telm_Data_Batt_Temp *)encoded;

            tmp_data->structData.msg_id=0x09;
            tmp_data->structData.batt_num=1u;
            tmp_data->structData.sub_sys_id=1u;
            tmp_data->structData.temp_probe_num[0]=0u;
            tmp_data->structData.temp_probe_num[1]=16u;
            singleBatTmpPackge(tmp_data, 0, 8, &CanDataList[id_0x1850D2F4]);
            singleBatTmpPackge(tmp_data, 8, 8, &CanDataList[id_0x1851D2F4]);

            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_BATT_TEMP;
        }
    }
    return rt;
}

/*******************************************************************************
*    Function: ucTelmProt_Encode_Batt_Temp
*
*  Parameters: 'uploadInfo' not used
*               'encode' point to encoded data;
*               'bufSize' not used
*     Returns: None
* Description: Encode the extending information
*******************************************************************************/
uint16_t TelmProt_Encode_Extended(uint8_t* const encoded, uint16_t buffsize)
{
    uint8_t rt = 0;
    if(TELM_INFO_LEN_EXTENDED <= buffsize && encoded != NULL)
    {
        const flexcan_frame_t *CanDataList = VHCL_CAN_GetDataList();
        if(CanDataList != NULL)
        {
            Telm_Data_Extended *tmp_data=(Telm_Data_Extended *)encoded;


            tmp_data->structData.msg_id = 0x80;
            tmp_data->structData.length[0] = (TELM_INFO_LEN_EXTENDED >> 8 & 0xFF);
            tmp_data->structData.length[1] = TELM_INFO_LEN_EXTENDED & 0xFF;
            
            tmp_data->structData.charge_finish_time[0] = 0x03;
            tmp_data->structData.charge_finish_time[1] = 0xFF;
            
            tmp_data->structData.remain_power = ((CanDataList[id_0x1804FFF4].dataByte7 << 8) 
                                                 | CanDataList[id_0x1804FFF4].dataByte6) / 1000;
            
            //lihaibin modify
            uint16_t drive_mileage = tmp_data->structData.remain_power * 7;
            tmp_data->structData.drive_mileage[0] = drive_mileage >> 8 & 0xFF;
            tmp_data->structData.drive_mileage[1] = drive_mileage & 0xFF;
            
            tmp_data->structData.power_percent = ((CanDataList[id_0x1801FFF4].dataByte5 << 8)
                                                 | CanDataList[id_0x1801FFF4].dataByte4) / 10;
            tmp_data->structData.acc_status = ACC_GetStatus() ? 1 : 0;
            tmp_data->structData.charge_pile_status = BMS_GetStatus() ? 1 : 0;
            
            tmp_data->structData.tie_fl = 0xFF;
            tmp_data->structData.tie_fr = 0xFF;
            tmp_data->structData.tie_rl = 0xFF;
            tmp_data->structData.tie_rr = 0xFF;
            tmp_data->structData.window_status = 0xFF;
            VHCL_CAN_GiveDataList();
            rt = TELM_INFO_LEN_EXTENDED;
        }
    }
    return rt;
}


static uint8_t prvTelmProt_Encode_Header(TELM_DataType_t cmdType,
                                         TELM_AckType_t ackType,
                                         uint8_t* const encoded, 
                                         uint16_t len, 
                                         Msg_Encrypt_Type_t encryptType)
{
    // return encoded length
    Telm_data_header_t *pHeader = (Telm_data_header_t *)encoded;
    pHeader->start = 0x2323;
    pHeader->cmdType = cmdType;
    pHeader->encType = encryptType;
    pHeader->ackFalg = ackType;
    PRM_GetVIN(pHeader->VIN);
    pHeader->len[0] = (len>>8) & 0xff;
    pHeader->len[1] = (len) & 0xff;
#if 0
    *encoded=HEADER_STX;
    *(encoded+1)=HEADER_STX;
    *(encoded+2)=uploadInfo->cmd_id;
    *(encoded+3)=uploadInfo->ack_flag;
    memcpy(encoded+4,uploadInfo->vin,17);
    *(encoded+21)=encryptType;
    *(encoded+22)=(len>>8) & 0xff;
    *(encoded+23)=(len) & 0xff;
#endif
    return sizeof(Telm_data_header_t);
}

void TelmProt_Set_New_OTA(uint8_t flag)
{
    ota_packet_flag = flag;
}

uint8_t TelmProt_Is_New_OTA(void)
{
    if(ota_packet_flag)
    {
        ota_packet_flag--;
        return 0;//0: Sending
    }
    else
    {
        return 1;//1: can send next packet.
    }
}

void TelmProt_Sleep_NV_Write(void)
{

}

// Save travel summary data
void TelmProt_Travel_Summary_Record(void)
{

}

TelmSesionState_t TelmProt_getSesionState(TelmSesionNum_t sesionNum)
{
    
    if(sesionNum < TELM_SESION_NUM)
    {
        switch(IOT_GetSessionState(sesionNum))
        {
            case ipSesionClosing:
            case ipSesionClose:
            case ipSesionOpening:
                break; 
            case ipSesionOpened:
                if(sesionState[sesionNum] == TELM_SESION_INACTIVE)
                {
                    sesionState[sesionNum] = TELM_SESION_ACTIVE;
                }
                break;
            default:
                break;
        }
        return (TelmSesionState_t)sesionState[sesionNum];
    }
    return TELM_SESION_INACTIVE;   
}

bool TelmProt_setSesionState(TelmSesionNum_t sesionNum, TelmSesionState_t state)
{
    if(sesionNum < TELM_SESION_NUM)
    {   
        sesionState[sesionNum] = (uint8_t)state;
    }   
    return false;
}

bool TelmProt_backupData(TelmRealtime_t *pBuffer)
{
    Telm_data_header_t *pHeader = (Telm_data_header_t *)pBuffer->data;
    //swtich to backup data
    uint8_t tmpCheckSum = pBuffer->data[pBuffer->len - 1] ^ pHeader->cmdType;
    pHeader->cmdType = 0x03;
    pBuffer->data[pBuffer->len - 1] = tmpCheckSum ^ pHeader->cmdType;
    return Write_vhcl_Data(pBuffer);
}

void TelmProt_connectHandler(bool state, uint8_t channel)
{
    if(state)
    {
        
    }
    else
    {
        sesionState[channel] = TELM_SESION_ACTIVE;
    }
}

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 *
\*=======================================================================================*/
