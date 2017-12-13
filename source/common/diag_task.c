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
   Title                      : Diag_Task.C

   Module Description         : 

   Author                     : 

   Created                    : 2016-08-29

 **********************************************************************/

/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/*********************************************************************/
#include "standard.h"
#include "gps.h"

//#define USE_DEBUG
#include "Debug.h"

#include "vehicle.h"
#include "fsl_rnga.h"
#include "stdlib.h"

#include "ATProtocol.h"
#include "ACC_Detect.h"

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
/*USER DEFINITION*/
#define UDS_RECEIVE_PAYLOAD_LENGTH  (uint8_t)64
#define UDS_SEND_DATA_LENGTH        (uint8_t)64
/* UDS data type define */
#define UDS_DATA_TYPE_SINGLE        (uint8_t)0
#define UDS_DATA_TYPE_FIRST         (uint8_t)1
#define UDS_DATA_TYPE_CONSECUTIVE   (uint8_t)2
#define UDS_DATA_TYPE_FLOWCONTROL   (uint8_t)3


/* UDS service type */
#define UDS_SRV_TYPE_SESSION_CTRL   (uint8_t)0x10   //session control service
#define UDS_SRV_TYPE_ECURESET		(uint8_t)0x11	//ECU reset
#define UDS_SRV_TYPE_SECURITY_ACS   (uint8_t)0x27   //security access service
#define UDS_SRV_TYPE_COM_CTRL		(uint8_t)0x28   //communication control
#define UDS_SRV_TYPE_TESTER_PRESENT (uint8_t)0x3E   //tester present
#define UDS_SRV_TYPE_CTRL_DCT_SET	(uint8_t)0x85	//control DTC setting

#define UDS_SRV_TYPE_RD_BY_ID       (uint8_t)0x22   //read data by id service
#define UDS_SRV_TYPE_RD_MEM_BY_AD	(uint8_t)0x23   //read memory by address
#define UDS_SRV_TYPE_RD_BY_PDID		(uint8_t)0x2A	//read data by periodic identifier
#define UDS_SRV_TYPE_DYDEF_DATA_ID  (uint8_t)0x3C 	//dynamically define data identifier
#define UDS_SRV_TYPE_WR_BY_ID       (uint8_t)0x2E   //write data by id service
#define UDS_SRV_TYPE_WR_MEM_BY_AD	(uint8_t)0x3D	//write memory by address
#define UDS_SRV_TYPE_TESTER_HOLDON  (uint8_t)0x3E   //test tool keep connection service

#define UDS_SRV_TYPE_RD_DTC_INFO	(uint8_t)0x19	//read DTC information
#define UDS_SRV_TYPE_CL_DIAG_INFO	(uint8_t)0x14	//clear diagnostic information

#define UDS_SRV_TYPE_IO_CTRL_BY_ID	(uint8_t)0x27 	//input output control by identifier

#define USD_SRV_TYPE_ROUTINE_CTRL	(uint8_t)0x31	//routine control

#define UDS_SRV_TYPE_REQ_DOWLOAD	(uint8_t)0x34	//request download
#define UDS_SRV_TYPE_DATA_TRANS		(uint8_t)0x36	//transfer data
#define UDS_SRV_TYPE_REQ_EXT_TRANS	(uint8_t)0x37	//request transfer exit
/* uds session type */
#define UDS_SESSION_TYPE_DEFAULT    (uint8_t)0x01   //default session
#define UDS_SESSION_TYPE_FLUSH      (uint8_t)0x02   //flush session
#define UDS_SESSION_TYPE_EXTEND     (uint8_t)0x03   //extend session
#define UDS_SESSION_TYPE_DEV        (uint8_t)0x40   //developer session
#define UDS_SESSION_TYPE_VENDER     (uint8_t)0x60   //vender session
/* uds security access mode */
#define UDS_SECURITY_STANDBY        (uint8_t)0x00
#define UDS_SECURITY_EXTEND         (uint8_t)0x01
#define UDS_SECURITY_EXTEND_OK      (uint8_t)0x02
#define UDS_SECURITY_DEVELOP        (uint8_t)0x03
#define UDS_SECURITY_DEVELOP_OK     (uint8_t)0x04
#define UDS_SECURITY_FLUSH          (uint8_t)0x05
#define UDS_SECURITY_FLUSH_OK       (uint8_t)0x06
#define UDS_SECURITY_GURAD          (uint8_t)0x09
#define UDS_SECURITY_GURAD_OK       (uint8_t)0x0A
#define UDS_SECURITY_VENDER         (uint8_t)0x61
#define UDS_SECURITY_VENDER_OK      (uint8_t)0x62
/* UDS security access level */
#define UDS_SECUR_LEL_1				(uint8_t)0x01
#define UDS_SECUR_LEL_FBL			(uint8_t)0x09

#define UDS_FIRSR_FRAME_MAX_LEN     (uint8_t)6

#define  diag_current_state()       (diag_current_state)
//#define AD_TEMP (1)
#define AD_TEMP (0)
#define TEMP_AD_REF_VOL (330)

//#define LOW_VOLTAGE_ALARM_THRESHOLD (290)
#define P2_CAN_SERVER_MAX           (uint16_t)0xFFFF
#define UDS_RX_TIME_OUT             MSec_To_Ticks(P2_CAN_SERVER_MAX)
#define P2_ENH_CAN_SERVER_MAX       (uint16_t)0xFFFF

#define UDS_RESPONSE_ID             FLEXCAN_ID_STD(0x7E8)
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef void (*uds_receiveCompleteHanler)(uint8_t *pParm);

typedef struct
{
	uds_receiveCompleteHanler handler;
    uint16_t length;            //express the data length
    uint16_t receivedLength;    //express the length of the data that has receive
    uint8_t payload[UDS_RECEIVE_PAYLOAD_LENGTH];
}uds_receive_t;
typedef struct
{
    uint16_t length;  //the length of the data to be send
    uint16_t sentLength;  //the length of the data that have sent
    uint8_t data[UDS_SEND_DATA_LENGTH];  //store the data to be send
}uds_send_t;
/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/

/*event handler*/
static void diag_evt_nop(int16_t data);
static void diag_evt_uds_evt(int16_t data);

/*********************************************************************
* variable
*********************************************************************/
static uint8_t uds_sessionType = UDS_SESSION_TYPE_DEFAULT;  //session type
static uint8_t uds_securityLevel;                           //security mode
//static Self_Diag_T diag_result;
static uds_receive_t uds_rx_buffer;                         //a buffer to store the received data
static uds_send_t uds_tx_buffer;                            //a buffer to store the sent data
static uint8_t tryCnt;                                      //security access try count 
static uint32_t seed = 0;                                   //a seed for security access
static uint8_t unitType;           //express the PDU type
/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

// Definition of the event handler function pointer array.
static const void_int16_fptr event_handler[DIAG_NUM_EVENTS] = 
{
	diag_evt_nop,	
    diag_evt_uds_evt,
};

/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/

/*********************************************************************/
/* ROM const Variables With File Level Scope                         */
/*********************************************************************/

/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/
// Test repeatly
//static void diag_self_test(void);
//static void diag_test_voltage(void);
//static void diag_test_temp(void);
//static void diag_test_light_sensor(void);
//static void diag_test_gps(void);
//static void diag_test_gprs(void);
/**********************************************************************
 *    Function: Diag_Task
 *  Parameters: None
 *     Returns: None
 * Description: Main routine called by the operating system
 *********************************************************************/
void Diag_Task(void *pvParameters)
{
    Data_Message_T msg;          // Holds message received from the system
//    uint32_t csq_time = 0;
    //csq_time = OS_Time();
    while(PS_Running())
    {
        if(OS_E_OK == OS_Wait_Message(OS_DIAG_TASK,&msg.all,100))
        {
            if((msg.parts.msg > 0) && ((msg.parts.msg) < DIAG_NUM_EVENTS))
            {
                (*event_handler[msg.parts.msg])(msg.parts.data);       // Run event handler routine
            }
        }
#if 0
        if ((csq_time + MSec_To_Ticks(3000)) < OS_Time())
        {
            //diag_self_test();
            csq_time = OS_Time();
        }
#endif
    }
    OS_Terminate_Task();
}
/**********************************************************************
 *    Function: 
 *  Parameters: 
 *     Returns: None
 * Description: 
 *********************************************************************/
static uint8_t diag_uds_getSessionTypeBySecurityMode(uint8_t mode)
{
    switch(mode)
    {
        case UDS_SECURITY_EXTEND:
        case UDS_SECURITY_GURAD:
            return UDS_SESSION_TYPE_EXTEND;
        case UDS_SECURITY_DEVELOP:
            return UDS_SESSION_TYPE_DEFAULT;
        case UDS_SECURITY_FLUSH:
            return UDS_SESSION_TYPE_FLUSH;
        case UDS_SECURITY_VENDER:
            return UDS_SESSION_TYPE_VENDER;
        default:
            return UDS_SESSION_TYPE_DEFAULT;
            
    }
}

/**********************************************************************
 *    Function: diag_uds_errorHandle
 *  Parameters: 'errId' express the error's id
 *              'errCode' express the error code
 *              'pFrame' point to the frame received
 *     Returns: None
 * Description: session control service
 *********************************************************************/
static bool diag_uds_errorHandle(uint8_t errId, uint8_t errCode)
{
    uds_tx_buffer.length = 4;
    uds_tx_buffer.data[0] = 0x03;
    uds_tx_buffer.data[1] = 0x7F;
    uds_tx_buffer.data[2] = errId;
    uds_tx_buffer.data[3] = errCode;
    memset(&uds_tx_buffer.data[4], 0, 4);
    return true;
}
/**********************************************************************
 *    Function: diag_uds_sessionControl
 *  Parameters: 'pFrame' point to the frame'd received 
 *     Returns: None
 * Description: session control service
 *********************************************************************/
static bool diag_uds_sessionControl()
{
    seed = 0;
    tryCnt = 0;
    //check length first
    if(uds_rx_buffer.length != 2)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        return false;
    }
    
    switch(uds_rx_buffer.payload[1])
    {
        case UDS_SESSION_TYPE_DEFAULT:
        case UDS_SESSION_TYPE_FLUSH:
        case UDS_SESSION_TYPE_EXTEND:
            uds_sessionType = uds_rx_buffer.payload[1];
            uds_tx_buffer.data[0] = 0x06;
            uds_tx_buffer.data[1] = 0x50;
            uds_tx_buffer.data[2] = uds_sessionType;
            uds_tx_buffer.data[3] = P2_CAN_SERVER_MAX >> 8;
            uds_tx_buffer.data[4] = P2_CAN_SERVER_MAX & 0xFF;
            uds_tx_buffer.data[5] = 0xFF;
            uds_tx_buffer.data[6] = 0xFF;
            uds_tx_buffer.data[7] = 0x00;
            uds_tx_buffer.length = 7;
            return true;
        default:
            //sub-function unsupported
            diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x12);
            return false;
    }
}
/**********************************************************************
 *    Function: diag_uds_SA_extenedRequesetSeed
 *  Parameters: 'mode' express the security access mode
 *     Returns: None
 * Description: handle the requestion of request for seed in extend session mode
 *********************************************************************/
static bool diag_uds_SA_reequesetSeed(uint8_t mode)
{
    if(uds_rx_buffer.length != 2)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        return false;
    }    
    if(uds_sessionType == UDS_SESSION_TYPE_DEFAULT)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x7F);
        return false;        
    }  
    if(uds_sessionType != diag_uds_getSessionTypeBySecurityMode(mode))
    {
        //current session is not extend session
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x7E);
        return false;        
    }
    RNGA_Init(RNG);
    do
    {
        RNGA_GetRandomData(RNG, &seed, sizeof(seed));  
    }while(seed == 0 || seed == 0xFFFFFFFF);
    uds_tx_buffer.data[0] = 0x06;
    uds_tx_buffer.data[1] = 0x67;
    uds_tx_buffer.data[2] = 0x01;
    uds_tx_buffer.data[3] = (seed >> 24) & 0xFF;
    uds_tx_buffer.data[4] = (seed >> 16) & 0xFF;
    uds_tx_buffer.data[5] = (seed >> 8) & 0xFF;
    uds_tx_buffer.data[6] = seed & 0xFF;
    uds_tx_buffer.data[7] = 0x00;
    uds_tx_buffer.length = 7;
    uds_securityLevel = mode;                   
    tryCnt = 0;
    return true;    
}
/**********************************************************************
 *    Function: diag_uds_SA_extenedVerificationKey
 *  Parameters: 'mode' express the security access mode
 *     Returns: None
 * Description: verificate the key in extend session mode
 *********************************************************************/
static bool diag_uds_SA_VerificationKey(uint8_t mode)
{
    uint32_t key = 0;
    if(uds_rx_buffer.length != 6)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        return false;        
    }
    if(tryCnt >= 2)
    {
         diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x36);
         
         return false;
    }
    if(seed == 0)
    {
        //if seed=0, express that the client has not requested for a seed yet
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x24);
        return false;          
    }
    if(uds_securityLevel != mode)
    {
        //the sequence of requestion is error
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x22);
        return false;
    }

    key = ((((seed>>4) ^ seed) <<3) ^ seed);                
    if(key == ((uds_rx_buffer.payload[2] << 24) 
             |(uds_rx_buffer.payload[3] << 16)
             |(uds_rx_buffer.payload[4] << 8)
             |(uds_rx_buffer.payload[5])))
    {
        //the key is right            
        uds_tx_buffer.data[0] = 0x02;
        uds_tx_buffer.data[1] = 0x67;
        uds_tx_buffer.data[2] = 0x02;
        memset(&uds_tx_buffer.data[3], 0, 5); 
        uds_tx_buffer.length = 3;
        uds_securityLevel = uds_securityLevel+1;
        return true;
    }
    else
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x35);
        tryCnt++;                               
        return false;
    }       
}
/**********************************************************************
 *    Function: diag_uds_securityAccess
 *  Parameters: 'pFrame' point to the frame'd received 
 *     Returns: None
 * Description: secrity access service
 *********************************************************************/
static bool diag_uds_securityAccess(void)
{
    switch(uds_rx_buffer.payload[1])
    {
        case UDS_SECURITY_EXTEND:  //recieved a request of request for seed in extend mode
        case UDS_SECURITY_DEVELOP:
        case UDS_SECURITY_FLUSH:
        case UDS_SECURITY_GURAD:
        case UDS_SECURITY_VENDER:
            return diag_uds_SA_reequesetSeed(uds_rx_buffer.payload[1]);
        case UDS_SECURITY_EXTEND+1: //recieved a key in extend mode
        case UDS_SECURITY_DEVELOP+1:
        case UDS_SECURITY_FLUSH+1:
        case UDS_SECURITY_GURAD+1:
        case UDS_SECURITY_VENDER+1:        
            return diag_uds_SA_VerificationKey(uds_rx_buffer.payload[1]-1);           
        default:
            if(uds_rx_buffer.length != 2 && uds_rx_buffer.length != 6)
            {
                 diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);                
            }
            else
            {
                diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x12);
            }
            return false;
    }
}
/**********************************************************************
 *    Function: diag_uds_readVIN
 *  Parameters: None
 *     Returns: None
 * Description: encode the response of "readDataByIdentifier" request of reading the vin code
 *********************************************************************/
static bool diag_uds_sendVIN(void)
{
    memset(uds_tx_buffer.data, 0, UDS_SEND_DATA_LENGTH);
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 0x14;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    PRM_GetVIN(&uds_tx_buffer.data[5]); 
    uds_tx_buffer.length = 22;
    return true;
}
static bool diag_uds_diagSimState(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = IOT_IsSimReady();
    uds_tx_buffer.length = 5;
    return true;
}
static bool diag_uds_diagGprsModule(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = IOT_IsIpTransOk();
    //memset(&uds_tx_buffer.data[5], 0, 3);
    uds_tx_buffer.length = 5;
    return true;
}
static bool diag_uds_diagGpsModules(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = vGps_Get_Gps_Status();
    //memset(&uds_tx_buffer.data[5], 0, 3);
    uds_tx_buffer.length = 5;
    return true;
}
static bool diag_uds_readVolage(void)
{
    uint16_t voltage = Pwr_Fail_Get_Voltage();
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = voltage & 0xFF;
    uds_tx_buffer.data[5] = (voltage >> 8) & 0xFF;
    //memset(&uds_tx_buffer.data[6], 0, 2);
    uds_tx_buffer.length = 6;
    return true;    
}
static bool diag_uds_readInternalVolage(void)
{
    uint16_t voltage = Pwr_Fail_Get_Int_Voltage();
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = voltage & 0xFF;
    uds_tx_buffer.data[5] = (voltage >> 8) & 0xFF;
    //memset(&uds_tx_buffer.data[6], 0, 2);
    uds_tx_buffer.length = 6;
    return true;    
}
static bool diag_uds_readCsq(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = IOT_GetCsq();
    //memset(&uds_tx_buffer.data[5], 0, 3);
    uds_tx_buffer.length = 5;
    return true;     
}
static bool diag_uds_readFlashInfo(void)
{
    uint8_t VenderId = 0, DevId[2] = ""; 
    spi_flash_readid(&VenderId, DevId);
    uds_tx_buffer.data[0] = 0x06;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = VenderId;
    uds_tx_buffer.data[5] = DevId[0];
    uds_tx_buffer.data[6] = DevId[1];
    //memset(&uds_tx_buffer.data[6], 0, 3);
    uds_tx_buffer.length = 5;
    return true;      
}
static bool diag_uds_diagFlash(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = spi_flash_is_ok();
    //memset(&uds_tx_buffer.data[5], 0, 3);
    uds_tx_buffer.length = 5;
    return true;
}
#define SWVR_LEN 16
static bool diag_uds_getSwv(void)
{
    uint8_t *pSwr = (uint8_t *)SY_Sw_Version();
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + SWVR_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    memcpy(&uds_tx_buffer.data[5], SY_Sw_Version(), SWVR_LEN);
    uds_tx_buffer.length = 5 + SWVR_LEN;
    return true;     
}
#define ICCID_LEN 10
static bool diag_uds_readIccid(void)
{ 
    uint8_t iccid[ICCID_LEN] = "";
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + ICCID_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    IOT_GetIccid(iccid);
    memcpy(&uds_tx_buffer.data[5], iccid, ICCID_LEN);
    uds_tx_buffer.length = 5 + ICCID_LEN;
    return true;     
}
#define IMSI_LEN    8
static bool diag_uds_readImsi(void)
{
    uint8_t imsi[8] = "";
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + IMSI_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    IOT_GetImsi(imsi);
    memcpy(&uds_tx_buffer.data[5], imsi, IMSI_LEN);
    uds_tx_buffer.length = 5 + IMSI_LEN;
    return true;        
}
#define IMEI_LEN 8
static bool diag_uds_readImei(void)
{
    uint8_t imei[8] = "";
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + IMEI_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    IOT_GetImei(imei);
    memcpy(&uds_tx_buffer.data[5], imei, IMEI_LEN);
    uds_tx_buffer.length = 5 + IMEI_LEN;
    return true;     
}

#define SN_LEN 8
static bool diag_uds_readSn(void)
{
    uint8_t sn[8] = "";
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + SN_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    PRM_GetSn(sn);
    memcpy(&uds_tx_buffer.data[5], sn, SN_LEN);
    uds_tx_buffer.length = 5 + SN_LEN;
    return true;       
}

#define HWVR_LEN 4
static bool diag_uds_readHwvr(void)
{
    uint8_t *pHwr = (uint8_t *)SY_Hwid();
    uds_tx_buffer.data[0] = 0x10;
    uds_tx_buffer.data[1] = 3 + SWVR_LEN;
    uds_tx_buffer.data[2] = 0x62;
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[4] = uds_rx_buffer.payload[2];
    memcpy(&uds_tx_buffer.data[5], SY_Sw_Version(), HWVR_LEN);
    uds_tx_buffer.length = 5 + HWVR_LEN;
    return true;     
}

static bool diag_uds_readAcc(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.data[4] = ACC_GetStatus();
    uds_tx_buffer.length = 5;  
    return true;
}

static bool diag_uds_resetFactory(void)
{
    
    DevInfo_Sct_t config;
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.length = 5; 
    if(Get_Manufacture_Setting(&config))
    {
        if(Set_Manufacture_Setting(config))
        {
            uds_tx_buffer.data[4] = 1;
            //memset(&uds_tx_buffer.data[5], 0, 3);

            return true;
        }
        return false;
    }
    return false;
}

static bool diag_uds_forceSleep(void)
{
    uds_tx_buffer.data[0] = 0x04;
    uds_tx_buffer.data[1] = 0x62;
    uds_tx_buffer.data[2] = uds_rx_buffer.payload[1];
    uds_tx_buffer.data[3] = uds_rx_buffer.payload[2];
    uds_tx_buffer.length = 5; 
    //OS_Send_Message(OS_PSYNC_TASK, PS_EVT_IGN_ON);
    PS_force_sleep_set(true);
    //RL_Force_Sleep();
    return true;
}
/**********************************************************************
 *    Function: diag_uds_readDataById
 *  Parameters: None
 *     Returns: None
 * Description: read data by ID service
 *********************************************************************/
static bool diag_uds_readDataById(void)
{
    if(uds_rx_buffer.length != 3)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        return false;
    }
    else
    {
        uint16_t id = (uds_rx_buffer.payload[1] << 8) | uds_rx_buffer.payload[2];
        switch(id)
        {
            case 0xF190: //VIN code identifier  
                return diag_uds_sendVIN();
            case 0xB001: //diagnose sim card state
                return diag_uds_diagSimState();
            case 0xB002: //diagnose gprs module state
                return diag_uds_diagGprsModule();
            case 0xB003: //diagnose gps module state
                return diag_uds_diagGpsModules();
            case 0xB004: //read internal voltage
                return diag_uds_readInternalVolage();
            case 0xB005: //read external voltage
                return diag_uds_readVolage();
            case 0xB006: //read current csq
                return diag_uds_readCsq();
            case 0xB007: //diagnose flash infomation
                return diag_uds_readFlashInfo();
            case 0xB008: //diagnose falsh
                return diag_uds_diagFlash();
            case 0xB009: //read software version
                return diag_uds_getSwv();
            case 0xB00A: //read iccid
                return diag_uds_readIccid();
            case 0xB00B: //read IMSI
                return diag_uds_readImsi();
            case 0xB00C: //read IMEI
                return diag_uds_readImei();
            case 0xB00D:
                return diag_uds_readHwvr();
            case 0xB00E:
                return diag_uds_readSn();
            case 0xB00F:
                return diag_uds_readAcc();
            case 0xB010:
                return diag_uds_resetFactory();
            case 0xB011:
                return diag_uds_forceSleep();
            default:
                diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x31);
                return false;
        }
    }
}

/**********************************************************************
 *    Function: diag_uds_flushVIN
 *  Parameters: 'phase' express the phase of recieving the VIN code
 *     Returns: None
 * Description: send vin to client
 *********************************************************************/
static void diag_uds_flushVIN(uint8_t *pParameter)
{
    if(true == PRM_SetVIN(&uds_rx_buffer.payload[3]))
    {
        uds_tx_buffer.data[0] = 0x03;
        uds_tx_buffer.data[1] = 0x6E;
        uds_tx_buffer.data[2] = 0xF1;
        uds_tx_buffer.data[3] = 0x90;
        memset(&uds_tx_buffer.data[4], 0, 4);
    }
}
/**********************************************************************
 *    Function: diag_uds_receiveVIN
 *  Parameters: None
 *     Returns: None
 * Description: send vin to client
 *********************************************************************/
static bool diag_uds_receiveVIN()
{
    bool ret = true;
    if(uds_rx_buffer.length != 0x14)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        ret = false;
    }
    if(UDS_SESSION_TYPE_DEFAULT == uds_sessionType)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x7F);  
        return false;        
    }
    else if(uds_securityLevel != UDS_SECURITY_EXTEND_OK)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x33);
        ret = false;
    }
    uds_rx_buffer.handler = diag_uds_flushVIN;
    return ret;
}

/**********************************************************************
 *    Function: diag_uds_encodeFlowControl
 *  Parameters: None
 *     Returns: None
 * Description: encode the flow control data
 *********************************************************************/
static bool diag_uds_encodeFlowControl(void)
{
    uds_tx_buffer.length = 8;
    uds_tx_buffer.data[0] = 0x30;
    memset(&uds_tx_buffer.data[1], 0, 7);
    return true;
}
/**********************************************************************
 *    Function: diag_uds_flushSn
 *  Parameters: 'phase' express the phase of recieving the VIN code
 *     Returns: None
 * Description: send vin to client
 *********************************************************************/
static void diag_uds_flushSn(uint8_t *pParameter)
{
    if(true == PRM_SetSn (&uds_rx_buffer.payload[3]))
    {
        uds_tx_buffer.data[0] = 0x03;
        uds_tx_buffer.data[1] = 0x6E;
        uds_tx_buffer.data[2] = 0xB1;
        uds_tx_buffer.data[3] = 0x00;
        memset(&uds_tx_buffer.data[4], 0, 4);
    }
}


/**********************************************************************
 *    Function: diag_uds_writeSn
 *  Parameters: None
 *     Returns: None
 * Description: Write SN to flash memery
 *********************************************************************/
static bool diag_uds_writeSn(void)
{
    bool ret = true;
    if(uds_rx_buffer.length != 3 + SN_LEN)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);
        ret = false;
    }
    uds_rx_buffer.handler = diag_uds_flushSn;
    return ret;    
}
/**********************************************************************
 *    Function: diag_uds_StartCharge
 *  Parameters: None
 *     Returns: None
 * Description: 
 *********************************************************************/
static bool diag_uds_StartCharge(void)
{
    IO_CHARGE_CTL(Bit_SET);
    uds_tx_buffer.data[0] = 0x03;
    uds_tx_buffer.data[1] = 0x6E;
    uds_tx_buffer.data[2] = 0xB1;
    uds_tx_buffer.data[3] = 0x01;
    uds_tx_buffer.data[4] = 0x01;
    return true;
}
/**********************************************************************
 *    Function: diag_uds_StopCharge
 *  Parameters: None
 *     Returns: None
 * Description: 
 *********************************************************************/
static bool diag_uds_StopCharge(void)
{
    IO_CHARGE_CTL(Bit_RESET);
    uds_tx_buffer.data[0] = 0x03;
    uds_tx_buffer.data[1] = 0x6E;
    uds_tx_buffer.data[2] = 0xB1;
    uds_tx_buffer.data[3] = 0x02;
    uds_tx_buffer.data[4] = 0x01;
    return true;
}
/**********************************************************************
 *    Function: diag_uds_writeDataById
 *  Parameters: None
 *     Returns: None
 * Description: write data by ID service
 *********************************************************************/
static bool diag_uds_writeDataById(void)
{
    uint16_t id = (uds_rx_buffer.payload[1] << 8) | uds_rx_buffer.payload[2];
    switch(id)
    {
        case 0xF190: //vin code

            if(true == diag_uds_receiveVIN())
            {
                diag_uds_encodeFlowControl();  
                return true;
            }
            return false;
        case 0xB100://write SN
            if(true == diag_uds_writeSn())
            {
                diag_uds_encodeFlowControl();  
                return true;
            }
            return false;
        case 0xB101://start charge
            return diag_uds_StartCharge();
        case 0xB102://stop charge
            return diag_uds_StopCharge();
        default:
            if(uds_rx_buffer.length <= 3)
            {
                diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);  
                return false;
            }
            if(UDS_SESSION_TYPE_DEFAULT == uds_sessionType)
            {
                diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x7F);  
                return false;        
            }
            diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x31);
            return false;
    }
}

/**********************************************************************
 *    Function: diag_uds_testerHoldOn
 *  Parameters: None
 *     Returns: None
 * Description: 
 *********************************************************************/
static bool diag_uds_testerHoldOn(uint8_t *pData)
{
    if(pData[0] != 2)
    {
        diag_uds_errorHandle(pData[1], 0x13);
        return false;        
    }
    if(pData[2] != 0)
    {
        diag_uds_errorHandle(pData[1], 0x12);
        return false;
    }
    uds_tx_buffer.data[0] = 2;
    uds_tx_buffer.data[1] = 0x7E;
    uds_tx_buffer.data[2] = 0;
    memset(&uds_tx_buffer.data[3],0,5);
    uds_tx_buffer.sentLength = 3;
    return true;
}

/**********************************************************************
 *    Function: diag_uds_unsuportedService
 *  Parameters: 'pFrame' point to the frame'd received 
 *     Returns: None
 * Description: 
 *********************************************************************/
static bool diag_uds_unsuportedService(void)
{
    if(uds_rx_buffer.length < 2)
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x13);        
    }
    else
    {
        diag_uds_errorHandle(uds_rx_buffer.payload[0], 0x7F);         
    }
    return true;
}
/**********************************************************************
 * Description: Do nothing event handler
 * Parameters: message data
 *     Returns: None
 *********************************************************************/
static void diag_evt_nop(int16_t data)
{
    
}


/**********************************************************************
 *    Function: diag_uds_rx_handleHook
 *  Parameters: None
 *     Returns: None
 * Description: define what to do after parsed the received data
 *********************************************************************/
static bool diag_uds_rx_handleHook(void)
{
    flexcan_frame_t sendData = {0};

    switch(unitType)
    {
        case UDS_DATA_TYPE_SINGLE:   
        case UDS_DATA_TYPE_FIRST:
            sendData.dataByte0 = uds_tx_buffer.data[0];
            sendData.dataByte1 = uds_tx_buffer.data[1];
            sendData.dataByte2 = uds_tx_buffer.data[2];
            sendData.dataByte3 = uds_tx_buffer.data[3];
            sendData.dataByte4 = uds_tx_buffer.data[4];
            sendData.dataByte5 = uds_tx_buffer.data[5];
            sendData.dataByte6 = uds_tx_buffer.data[6];
            sendData.dataByte7 = uds_tx_buffer.data[7];
            sendData.id = UDS_RESPONSE_ID;
            sendData.length = 8;
            sendData.type = kFLEXCAN_FrameFormatStandard;
            sendData.format = kFLEXCAN_FrameTypeData;
            if(0 != CAN_SendBlocking(CAN_CHANNEL0, &sendData))
            {
                //report error
            }
            break;
        case UDS_DATA_TYPE_CONSECUTIVE:
            if(uds_rx_buffer.length <= uds_rx_buffer.receivedLength)
            {
                //all data have received
                if(uds_rx_buffer.handler != NULL)
                {
                    uds_rx_buffer.handler(NULL);
                }
                sendData.dataByte0 = uds_tx_buffer.data[0];
                sendData.dataByte1 = uds_tx_buffer.data[1];
                sendData.dataByte2 = uds_tx_buffer.data[2];
                sendData.dataByte3 = uds_tx_buffer.data[3];
                sendData.dataByte4 = uds_tx_buffer.data[4];
                sendData.dataByte5 = uds_tx_buffer.data[5];
                sendData.dataByte6 = uds_tx_buffer.data[6];
                sendData.dataByte7 = uds_tx_buffer.data[7];
                sendData.id = UDS_RESPONSE_ID;
                sendData.length = 8;
                sendData.type = kFLEXCAN_FrameFormatStandard;
                sendData.format = kFLEXCAN_FrameTypeData;
                if(0 != CAN_SendBlocking(CAN_CHANNEL0, &sendData))
                {
                    //report error
                }
            }
            break;
        case UDS_DATA_TYPE_FLOWCONTROL:
            {
                uint8_t i;
                uint8_t dataSequence = 0x21;
                if(uds_rx_buffer.payload[0] != 0x22)
                {
                    return false;
                }
                for(i = 8; i < uds_tx_buffer.length;)
                {
                    sendData.dataByte0 = dataSequence++;
                    sendData.dataByte1 = uds_tx_buffer.data[i++];
                    sendData.dataByte2 = uds_tx_buffer.data[i++];
                    sendData.dataByte3 = uds_tx_buffer.data[i++];
                    sendData.dataByte4 = uds_tx_buffer.data[i++];
                    sendData.dataByte5 = uds_tx_buffer.data[i++];
                    sendData.dataByte6 = uds_tx_buffer.data[i++];
                    sendData.dataByte7 = uds_tx_buffer.data[i++];
                    sendData.id = UDS_RESPONSE_ID;
                    sendData.length = 8;
                    sendData.type = kFLEXCAN_FrameFormatStandard;
                    sendData.format = kFLEXCAN_FrameTypeData;
                    if(0 != CAN_SendBlocking(CAN_CHANNEL0, &sendData))
                    {
                        //report error
                    }
                }
                memset(&uds_rx_buffer, 0, sizeof(uds_receive_t));
                if((dataSequence & 0x0F) == 0x0F)
                {
                    return false;
                }
            }
            break;
        default:
            return false;
    }
    DEBUG(DEBUG_LOW, "[DIAG] Send to client:\r\n");
    for(uint8_t i=0; i<8; i++)
    {
        DEBUG(DEBUG_LOW, "%02x ",uds_tx_buffer.data[i]);
    }
    DEBUG(DEBUG_LOW, "\r\n\r\n"); 
    return true;
}
/**********************************************************************
 *    Function: diag_uds_stateIdle
 *  Parameters: 'pFrame' point to the frame'd received 
 *     Returns: None
 * Description: do something when uds is idle
 *********************************************************************/
static bool diag_uds_serviceHanndle(void)
{
    memset(&uds_tx_buffer, 0, sizeof(uds_send_t));
    switch(uds_rx_buffer.payload[0])
    {
        case UDS_SRV_TYPE_SESSION_CTRL: //session control
             return diag_uds_sessionControl();
        case UDS_SRV_TYPE_SECURITY_ACS: //security access
             return diag_uds_securityAccess();
        case UDS_SRV_TYPE_RD_BY_ID: //read data by id
             return diag_uds_readDataById();
        case UDS_SRV_TYPE_WR_BY_ID: //wirte data by id
             return diag_uds_writeDataById();
        default:
            diag_uds_unsuportedService();
            return false;
    }
    
}

/**********************************************************************
 *    Function: diag_uds_parseData
 *  Parameters: 'pFrame' point to the frame'd received 
 *     Returns: None
 * Description: parse the received data
 *********************************************************************/
static bool diag_uds_parseData(uint8_t *pFrame)
{
    static uint8_t sn = 0;  //it express the serial number when uds data type is consecutive 
    if(pFrame == NULL)
    {
        return false;
    }
    switch((pFrame[0] >> 4) & 0x0F)
    {
        case UDS_DATA_TYPE_SINGLE:   
            unitType = UDS_DATA_TYPE_SINGLE;
            if(pFrame[1]  == 0x3E)
            {
                //If the type of the received data is 'keep connection',
                //don't store the requesting, and response the requesting directly
                return diag_uds_testerHoldOn(pFrame);   
            }

            uds_rx_buffer.length = pFrame[0];
            memcpy(uds_rx_buffer.payload, &pFrame[1], 7);   
            sn = 0;
            break;
        case UDS_DATA_TYPE_FIRST:
            unitType = UDS_DATA_TYPE_FIRST;
            uds_rx_buffer.length = (((pFrame[0] << 8) & 0x0F ) | pFrame[1]);
            memcpy(uds_rx_buffer.payload, &pFrame[2], 6);
            uds_rx_buffer.receivedLength = 6;
            sn = 1;
            break;
        case UDS_DATA_TYPE_CONSECUTIVE:
            if(uds_rx_buffer.length > UDS_FIRSR_FRAME_MAX_LEN)
            {
                unitType = UDS_DATA_TYPE_CONSECUTIVE;
                if((pFrame[0] & 0x0F) !=  sn)
                {
                    return false;
                }
                sn++;
                memcpy(&uds_rx_buffer.payload[uds_rx_buffer.receivedLength], &pFrame[1], 7);
                uds_rx_buffer.receivedLength += 7;
            }
            return true;
        case UDS_DATA_TYPE_FLOWCONTROL:
            unitType = UDS_DATA_TYPE_FLOWCONTROL;
            return true;
        default:
            //should not enter into here
            break;
    }
    return diag_uds_serviceHanndle();
    
}

/**********************************************************************
 * Description: USD diagnosis
 * Parameters: message data
 *     Returns: None
 *********************************************************************/
static void diag_evt_uds_evt(int16_t data)
{
    static TickType_t uds_rx_previousTime;
    uint8_t canData[8] = "";
    flexcan_frame_t frame = {0};
    
    TickType_t osTime = OS_Time();
    int32_t interval = osTime - uds_rx_previousTime;
    DEBUG(DEBUG_LOW, "[DIAG] RX interval:%d\r\n", interval);
    interval = interval < 0 ?  (0xFFFFFFFF % interval) : interval;
        
    if(interval > UDS_RX_TIME_OUT)
    {
        uds_sessionType = UDS_SESSION_TYPE_DEFAULT;
        uds_securityLevel = UDS_SECURITY_STANDBY;      
    }
    uds_rx_previousTime = osTime;
    UDS_GetData(&frame);
    canData[0] = frame.dataByte0;
    canData[1] = frame.dataByte1;
    canData[2] = frame.dataByte2;
    canData[3] = frame.dataByte3;
    canData[4] = frame.dataByte4;
    canData[5] = frame.dataByte5;
    canData[6] = frame.dataByte6;
    canData[7] = frame.dataByte7;
    DEBUG(DEBUG_LOW, "\r\n[DIAG] Receive from client:\r\n");
    for(uint8_t i=0; i<8; i++)
    {
        DEBUG(DEBUG_LOW, "%02x ",canData[i]);
    }
    DEBUG(DEBUG_LOW, "\r\n\r\n"); 
    diag_uds_parseData(canData);
    diag_uds_rx_handleHook();
}
/********************************************************************** 
 *                                                             
 * REVISION RECORDS                                            
 *                                                             
*********************************************************************/
/* $HISTROY$

 *********************************************************************/
