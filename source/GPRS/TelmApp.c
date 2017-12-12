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
#include "TelmApp.h"
#include "TelmProtocol.h"
#include "ATProtocol.h"
#include "gps.h"
#include "GPRS.h"

//#define USE_DEBUG
#include "Debug.h"
/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
#define TELM_TIME_CMD_TIMEOUT		(MSec_To_Ticks(1000))

#define TELM_LEN_QUEUE_UPLOAD_INFO	(8)
#define TELM_LEN_QUEUE_COMMAND		(4)
#define	TELM_CAN_COMMAND_RETRY_MAX	(3)

#define TELM_LEN_BACKUP_LIST (20)

#define GPS_BACKUP_SECTOR_MAX (128)

#define GPS_BACKUP_RECORD_PER_SECTOR (70)

#define VEHICLE_STOP_GPS_CMP_LEN (7)

#define ENG_ON_OFF_NV_BUF_LEN (20)
#define ENG_ON_OFF_RAM_BUF_LEN   (20)

#define NUM_OF_SAVED_BATT_VOLT (6)
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

/* queue data type */
typedef struct Telm_Queue_tag
{
	void* const queue;
	const uint8_t elementSize;
	const uint8_t queueSize;
	uint8_t	num;
	uint8_t in;
	uint8_t out;
}Telm_Queue;

/* collect telmatics info encode function type */

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/

/* functions to control the sequence of the tranceieve between tbox and server */
/* function of upload data */
static void prvTelmApp_sendNextData(void);
/* function of execute command */
static void prvTelmApp_excuteNextCommand(void);
static void prvTelmApp_Execute_Command(void);
static void prvTelmApp_End_Execute_Command(void);
static void prvTelmApp_Command_Wait_TimeUp(void);
static bool prvTelmApp_Check_Command_Result(void);
static void prvTelmApp_Return_Cmd_Result(bool result);

/* functions to manage queues */
static uint8_t prvTelmApp_Queue_Push(Telm_Queue_Handler q,void const* added);
static void* prvTelmApp_Queue_Get(Telm_Queue_Handler q);
static void prvTelmApp_Queue_Shift(Telm_Queue_Handler q);
static bool prvTelmApp_Queue_isEmpty(Telm_Queue_Handler q);

//static uint8_t get_gps_buffer(uint8_t *drive_data);

//static void convert_gps_lon(uint8_t *gps_data, uint8_t *dst_buf);

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
#define	TELM_UPLOAD_RETRY_MAX		        (3)
#define TELM_TIME_SERVER_ACK_TIMEOUT		(MSec_To_Ticks(60000))

static uint8_t	telm_server_ack; //Check if have received server ack.
static uint8_t telm_server_ack_result;//Check the server ack is ok or ng.
static uint8_t telm_app_upload_retrytimes;//Upload information retry times, max is TELM_UPLOAD_RETRY_MAX .
static Tick_Type telm_app_server_ack_timer;//Server ACK timer, max is TELM_TIME_SERVER_ACK_TIMEOUT.

//static uint8_t gps_backup = 0; /* 0=no backup; 1=storing gps; 2=uploading gps */

//static uint8_t gps_is_first_fixed = 0;/*1:indicate first-fix after ignition*/

/* status flag */
static Telm_State telm_upload_state = TELM_STATE_IDLE;

/* memory space of queue of commands */
static Telm_Rcvd_Command telm_command_queue[TELM_LEN_QUEUE_COMMAND];
/* memory space of queue of uploading data */
static Telm_Upload_Info_Request	upload_info_req_queue[TELM_LEN_QUEUE_UPLOAD_INFO];
static Telm_Upload_Info_Request	gb_upload_req_queue[TELM_LEN_QUEUE_UPLOAD_INFO];

static uint8_t backup_list_num = 0;
static Telm_Security_Event event_backup[TELM_LEN_BACKUP_LIST];

/* object of two queues */
static Telm_Queue telm_queue[TELM_QUEUE_NUM] = {
	/* queue			elementSize				queueSize		num	head	tail*/
	{(void*)upload_info_req_queue,	sizeof(Telm_Upload_Info_Request), TELM_LEN_QUEUE_UPLOAD_INFO,	0,	0,		0},
	{(void*)gb_upload_req_queue,	sizeof(Telm_Upload_Info_Request), TELM_LEN_QUEUE_UPLOAD_INFO,	0,	0,		0},
	{(void*)telm_command_queue,	sizeof(Telm_Rcvd_Command),        TELM_LEN_QUEUE_COMMAND,	0,	0,		0}
};

static uint8_t uploadDataTable[TELM_SEC_EVT_NUM][TELM_INFO_IN_PACK_MAX_NUM] = {
    {TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},
    {TELM_INFO_LOGIN,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_LOGIN
    {TELM_INFO_LOGOUT,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_LOGOUT
    {TELM_INFO_VEHICLE_DATA,TELM_INFO_EC_ENGINE_DATA,TELM_INFO_GPS_DATA,TELM_INFO_ABS_DATA,TELM_INFO_ALARM_DATA,TELM_INFO_BATT_VOLT,TELM_INFO_BATT_TEMP,TELM_INFO_EXTENDED},  // TELM_SEC_EVT_REALTIME
    {TELM_INFO_VEHICLE_DATA,TELM_INFO_EC_ENGINE_DATA,TELM_INFO_GPS_DATA,TELM_INFO_ABS_DATA,TELM_INFO_ALARM_DATA,TELM_INFO_BATT_VOLT,TELM_INFO_BATT_TEMP,TELM_INFO_EXTENDED},  // TELM_SEC_EVT_BACKUP
    {TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_HEARTBEAT
    {TELM_INFO_RESP,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE,TELM_INFO_NONE},  // TELM_SEC_EVT_ENC_KEY
};

/*********************************************************************/
/* User file include                                                 */
/*********************************************************************/

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/*******************************************************************************
*  Function: eTelmApp_upload_state
*
*  Parameters: :void
*  Returns: Telm_State
*  Description: 
*******************************************************************************/
Telm_State eTelmApp_upload_state(void)
{
    return telm_upload_state;
}
/*******************************************************************************
*  Function: vTelmApp_AT_Var_Reset
*
*  Parameters: :void
*  Returns: none
*  Description: 
*******************************************************************************/
void vTelmApp_AT_Var_Reset(void)
{
    telm_upload_state = TELM_STATE_IDLE;

    /*Stop the telm timers*/
    TMR_Stop_Timer(TELM_COMMAND_TIMER);
}

/*******************************************************************************
*    Function:  prvTelmApp_Collect_UTC_Time
*
*  Parameters:  Telm_Upload_Info* const:data collected
*     Returns:  None
* Description:  Collect UTC time from GPS
*******************************************************************************/
static void prvTelmApp_Collect_UTC_Time(Telm_Upload_Info* const uploadInfo)
{
    uint32_t cur_time = 0;

    cur_time = sys_get_cur_sec_offset();
    //For test.
    uploadInfo->utcTime[0] = (cur_time >> 24) & 0xFF;
    uploadInfo->utcTime[1] = (cur_time >> 16) & 0xFF;
    uploadInfo->utcTime[2] = (cur_time >>  8) & 0xFF;
    uploadInfo->utcTime[3] = cur_time & 0xFF;
}

/*******************************************************************************
*    Function: vTelmApp_uploadData
*
*  Parameters: Telm_Security_Event:what happened
* 			   notifyResult:	function to notify caller 
								when finished data uploading
*     Returns: None
* Description: 
*******************************************************************************/
extern uint8_t vTelmApp_uploadData(Telm_Security_Event evt, notifyResult uploaded, Telm_Queue_Handler channel)
{
    Telm_Upload_Info_Request	upLoadInfoReq;

    /* Check activation  */
    /* save the call back */	
    upLoadInfoReq.informDataUploaded = uploaded;
    /* set event */
    upLoadInfoReq.evt = evt;
    /* copy information of which data need to upload to request */
    memcpy(upLoadInfoReq.reqTable, uploadDataTable,sizeof(upLoadInfoReq.reqTable));
    /* push the request into request queue */
    if(1 == prvTelmApp_Queue_Push(channel, &upLoadInfoReq))
    {
        return 1;
    }
    return 0;
}

extern uint16_t TelmApp_data_Package(uint8_t *data, Telm_DataType_t type)
{
    Telm_Upload_Info_Request upLoadInfoReq;
    Telm_Upload_Info upload_info;
    uint16_t rt=0;

    upLoadInfoReq.evt = TELM_SEC_EVT_REALTIME;
    prvTelmApp_Collect_UTC_Time(&upload_info);
    switch(type)
    {
        case TELM_DATA_TYPE_REALTIME:
            upload_info.cmd_id = 0x02;
            break;
        case TELM_DATA_TYPE_BACKUP:
            upload_info.cmd_id = 0x03;
            break;
        default:
            return 0;
    }

    upload_info.ack_flag=0xFE;
    PRM_GetVIN(upload_info.vin);
    memcpy(upLoadInfoReq.reqTable, uploadDataTable,sizeof(upLoadInfoReq.reqTable));

    rt=vTelmProt_Encode_Package(&upLoadInfoReq,&upload_info,data);
    return rt;
}

extern uint16_t TelmApp_Ack_Package(uint8_t *data, uint8_t reslt, uint8_t cmdId)
{
    Telm_Upload_Info_Request upLoadInfoReq;
    Telm_Upload_Info upload_info;
    uint16_t rt=0;

    upLoadInfoReq.evt = TELM_SEC_EVT_ACK;
    prvTelmApp_Collect_UTC_Time(&upload_info);
    upload_info.cmd_id = cmdId;
    upload_info.ack_flag = reslt;
    PRM_GetVIN(upload_info.vin);
    memcpy(upLoadInfoReq.reqTable, uploadDataTable,sizeof(upLoadInfoReq.reqTable));

    rt=vTelmProt_Encode_Package(&upLoadInfoReq,&upload_info,data);
    return rt;    
}

/*******************************************************************************
*    Function: vTelmApp_Inform_Received_Command
*
*  Parameters: Telm_Rcvd_Command*:recieved command
*     Returns: none
* Description: when a command is recieved, this function is called
				to notify the contents of the command 
*******************************************************************************/
extern void vTelmApp_Inform_Received_Command(Telm_Rcvd_Command* rcvdCmd)
{
    /* push the command into command queue */
    prvTelmApp_Queue_Push(TELM_QUEUE_COMMAND, rcvdCmd);
    prvTelmApp_excuteNextCommand();
}

/*******************************************************************************
*    Function: prvTelmApp_excuteNextCommand
*
*  Parameters: None
*     Returns: None
* Description: check if there is recieved command in the queue and excute it
*******************************************************************************/
static void prvTelmApp_excuteNextCommand()
{
	if (prvTelmApp_Queue_isEmpty(TELM_QUEUE_COMMAND) == false)
	{
		prvTelmApp_Execute_Command();
	}
}

/*******************************************************************************
*    Function: prvTelmApp_Execute_Command
*
*  Parameters: none
*     Returns: none
* Description: handle command
*******************************************************************************/
static void prvTelmApp_Execute_Command()
{
	Telm_Rcvd_Command const *lastCmd;

	lastCmd = prvTelmApp_Queue_Get(TELM_QUEUE_COMMAND);
	if (lastCmd == NULL)
	{
		return;
	}
	switch (lastCmd->commType)
	{
		case TELM_COMM_THEFT:
			break;
		case TELM_COMM_FMC:
			break;
		case TELM_COMM_GPS:
			break;
		case TELM_COMM_DRWN:
			break;
		case TELM_COMM_OTA:
			break;
		case TELM_COMM_CONFIG:
			break;
		case TELM_COMM_SVRCMD:
			break;
        case TELM_COMM_DEV:
            break;
		case TELM_COMM_SPEED:
		case TELM_COMM_LAMP:
		case TELM_COMM_HVAC:
		case TELM_COMM_START:
		default:
			break;
	}
	return;
}

/*******************************************************************************
*    Function: prvTelmApp_End_Execute_Command
*
*  Parameters: none
*     Returns: bool true:can command executed/false:no can command
* Description: handle command
*******************************************************************************/
static void prvTelmApp_End_Execute_Command(void)
{
	Telm_Rcvd_Command const *lastCmd;

	lastCmd = prvTelmApp_Queue_Get(TELM_QUEUE_COMMAND);
	if (lastCmd == NULL)
	{
		return;
	}
	switch (lastCmd->commType)
	{
		case TELM_COMM_FMC:
			break;
		case TELM_COMM_DRWN:
			break;
		case TELM_COMM_GPS:
		case TELM_COMM_THEFT:
		case TELM_COMM_SPEED:
		case TELM_COMM_LAMP:
      case TELM_COMM_GET_STATUS:
		case TELM_COMM_HVAC:
		case TELM_COMM_START:
		default:
			break;
	}
	return;
}

/*******************************************************************************
*    Function: prvTelmApp_Execute_Command
*  Parameters: void
*     Returns: void
* Description: check CAN signal and judge whether command is correctly executed	
*******************************************************************************/
static void prvTelmApp_Command_Wait_TimeUp(void)
{
	static uint8_t retried = 0;
	uint8_t	i = 0;
	bool found = false;
	retried++;

	if ((prvTelmApp_Check_Command_Result() == true)
	 || (retried >= TELM_CAN_COMMAND_RETRY_MAX))
	{
		/* clear retrying counter */
		retried = 0;
		
		/* clear can control command */
		prvTelmApp_End_Execute_Command();

		/* check if there is any data needed to send back to server */
		/* uploadDataTable[TELM_SEC_EVT_CMD_RES][]is refreshed in 
			prvTelmApp_Check_Command_Result */
		for (i = 0; (i < TELM_INFO_NUM) && (found == false); i++)
		{
		}
		/* if command need to send data back to server, make upload request */
		if (found == true)
		{
		}
		else
		{
			/* if command doesn't need to send back data, return ok to server */
			prvTelmApp_Return_Cmd_Result(true);
		}
	}
	else
	{
		/* try to excute command again */
		/* it is no necessary to check the return value,because this is a timeout callback
		   which means a CAN command is being executed so that we need to check the result */
		prvTelmApp_Execute_Command();
		/* set the timer */
		TMR_Start_Timer(TELM_COMMAND_TIMER,TELM_TIME_CMD_TIMEOUT, prvTelmApp_Command_Wait_TimeUp);
	}
}

/*******************************************************************************
*    Function: prvTelmApp_Check_Command_Result
*  Parameters: void
*     Returns: void
* Description: check CAN signal and judge whether command is correctly executed	
*******************************************************************************/
static bool prvTelmApp_Check_Command_Result(void)
{

	Telm_Rcvd_Command *lastCmd;
	bool	cmdOK = false;

	lastCmd = prvTelmApp_Queue_Get(TELM_QUEUE_COMMAND);
	if (lastCmd == NULL)
	{
		return true;
	}
	switch (lastCmd->commType)
	{
		default:
			cmdOK = true;
			break;
	}
	/* save result of last command to send back to server at the end of trancieve sequence */
	lastCmd->result = cmdOK;
	return cmdOK;
}

/*******************************************************************************
*    Function: prvTelmApp_Return_Cmd_Result
*
*  Parameters: bool : result of uploading Data
*     Returns: None
* Description: send result of command result to server
*******************************************************************************/
static void prvTelmApp_Return_Cmd_Result(bool result)
{
	Telm_Rcvd_Command const *lastCmd;
	lastCmd = prvTelmApp_Queue_Get(TELM_QUEUE_COMMAND);
	/* return execute result as ACK */
	{
		lastCmd->cmdExecuted(result);
	}
	/* delete command after finish it */
	prvTelmApp_Queue_Shift(TELM_QUEUE_COMMAND);
	
	prvTelmApp_excuteNextCommand();
}


/*******************************************************************************
*    Function: prvTelmApp_Queue_Push
*
*  Parameters: Telm_Queue_Handler:specify which queue to access
*			   &uploadReq:reqeust to push into the queue
*     Returns: None
* Description: push data into the queue
*******************************************************************************/
static uint8_t prvTelmApp_Queue_Push(Telm_Queue_Handler q,void const* added)
{
	if (telm_queue[q].num < telm_queue[q].queueSize)
	{
        uint8_t *tmp_ptr = (uint8_t *)telm_queue[q].queue;
        tmp_ptr += (telm_queue[q].in * telm_queue[q].elementSize);
        memcpy(tmp_ptr, added, telm_queue[q].elementSize);
        telm_queue[q].num++;
        telm_queue[q].in++;
        if (telm_queue[q].in == telm_queue[q].queueSize)
        {
            telm_queue[q].in = 0;
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

/*******************************************************************************
*    Function: prvTelmApp_Queue_Get
*
*  Parameters: Telm_Queue_Handler:specify which queue to access
*     Returns: none
* Description: get the first upload data request from request queue
*******************************************************************************/
static void* prvTelmApp_Queue_Get(Telm_Queue_Handler q)
{
    if (telm_queue[q].num > 0)
    {
        return (void *)((uint8_t *)telm_queue[q].queue+(telm_queue[q].out * telm_queue[q].elementSize));
    }
    else
    {
        return NULL;
    }
}



/*******************************************************************************
*    Function: prvTelmApp_Upload_Queue_Shift
*
*  Parameters: Telm_Queue_Handler:specify which queue to access
*     Returns: none
* Description: get the first upload data request out of request queue
*******************************************************************************/
static void prvTelmApp_Queue_Shift(Telm_Queue_Handler q)
{
    if (telm_queue[q].num > 0)
    {
        telm_queue[q].num--;
        telm_queue[q].out++;
        if (telm_queue[q].out == telm_queue[q].queueSize)
        {
            telm_queue[q].out = 0;
        }
    }
}

void TelmApp_Queue_Clear(Telm_Queue_Handler q)
{
    telm_queue[q].num = 0;
    telm_queue[q].out = 0;
    telm_queue[q].in = 0;
    memset(telm_queue[q].queue,0,telm_queue[q].elementSize);
//    memset()
}

/*******************************************************************************
*    Function: prvTelmApp_Queue_Query
*
*  Parameters:
*     Returns: none
* Description: check if the specific event is in queue
*******************************************************************************/
uint8_t prvTelmApp_Queue_Query(Telm_Security_Event event)
{
    Telm_Upload_Info_Request * pQueue = (Telm_Upload_Info_Request *)(telm_queue[TELM_QUEUE_UPLOAD].queue);
    uint8_t out_index = telm_queue[TELM_QUEUE_UPLOAD].out;
    uint8_t i;

    for(i = 0; i < telm_queue[TELM_QUEUE_UPLOAD].num; i++)
    {
        if(event == (*(pQueue+out_index)).evt)
        {
            return 1;
        }
        else
        {
            out_index ++;
            if(out_index >= telm_queue[TELM_QUEUE_UPLOAD].queueSize)
            {
                out_index = 0;
            }
        }
    }
    return 0;
}

/*******************************************************************************
*    Function: prvTelmApp_Queue_isEmpty
*
*  Parameters: Telm_Queue_Handler:specify which queue to access
*     Returns: bool true:empty false:not empty
* Description: tell whether the queue is empty
*******************************************************************************/
static bool prvTelmApp_Queue_isEmpty(Telm_Queue_Handler q)
{
	if (telm_queue[q].num == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t vTelmApp_Backup_Msg(Telm_Security_Event evt)
{
    // Push message into queue
    if (backup_list_num < TELM_LEN_BACKUP_LIST)
    {
        event_backup[backup_list_num] = evt;
        backup_list_num++;
        return 1;
    }
    else
    {
        backup_list_num = TELM_LEN_BACKUP_LIST;
        return 0;
    }
}

uint8_t vTelmApp_Upload_Backup(void)
{
    // If backup command not empty, upload message
    if (backup_list_num > 0)
    {
        backup_list_num--;
        vTelmApp_uploadData(event_backup[backup_list_num], NULL, AT_CONNECT_ID_MAIN);
        return 1;
    }
    else
    {
        return 0;
    }
}

static Telm_State TelmApp_Get_Upload_state(void)
{
   return telm_upload_state;
}

static void TelmApp_Set_Upload_state(Telm_State set_value)
{
   telm_upload_state = set_value;
}

void TelmApp_Set_Server_Ack_State(bool akc_or_nak)
{
    telm_server_ack = true;
    telm_server_ack_result = akc_or_nak;
}
/*******************************************************************************
*    Function: vTelmApp_Inform_Upload_Result
*
*  Parameters: bool:true uploaded OK,false:uploaded NG
*     Returns: none
* Description: when ACK from server is recieved,this function is called
				to notify the result of last upload
*******************************************************************************/
extern void vTelmApp_Inform_Upload_Result(Telm_Upload_Result res)
{
	Telm_Upload_Info_Request const *lastReq;
    Telm_Queue_Handler channel;
    if(ATProt_Get_Rx() == AT_CONNECT_ID_GB)
    {
        channel = TELM_QUEUE_GB_UPLOAD;
    }
    else
    {
        channel = TELM_QUEUE_UPLOAD;
    }
	/* get last request from queue */
	lastReq = (Telm_Upload_Info_Request *)prvTelmApp_Queue_Get(channel);
	if (lastReq != NULL)
	{
		/* get call back function and execute */
		if(lastReq->informDataUploaded != (notifyResult)NULL)
		{
			lastReq->informDataUploaded(res == TELM_UPLOAD_RES_OK? true:false);
		}
	}
    prvTelmApp_Queue_Shift(channel);
	//ACK of last message is received, send next message
	TelmApp_Set_Upload_state(TELM_STATE_IDLE);
}

Telm_Upload_Info_Request	*newReq;
/*******************************************************************************
*    Function: prvTelmApp_Upload_Preprocess
*
*  Parameters: the upload structure
*     Returns: true: preprocess check ok, can send out the data.
* Description: Check if the necessary data of upload info is prepared ok.
*******************************************************************************/
static bool prvTelmApp_Upload_Preprocess(Telm_Upload_Info_Request const * const newReq)
{
    bool process_result = true; //default is data ok.

    if (newReq != NULL)
    {
#if 0
        if (newReq->evt == TELM_SEC_EVT_BACKUP)
        {
            if(1 == gps_backup_empty())
                process_result = false; //Intend to upload backup gps, but gps data is empty.
        }
        else if(newReq->evt == TELM_SEC_EVT_GPS)
        {
        }
        else
        {
            //Other upload information check, please add here.
        }
#endif
    }
    else
    {
        process_result = false;
    }
    return process_result;
}

#if 0
static void prvTelmApp_SetHeader(Telm_Upload_Info *info)
{
}
#endif

/*******************************************************************************
*    Function: prvTelmApp_sendNextData
*
*  Parameters: None
*     Returns: None
* Description: check if there is upload request in the queue and send it
*******************************************************************************/
static void prvTelmApp_sendNextData()
{
    Telm_Upload_Info_Request	*newReq;
    Telm_Queue_Handler channel = TELM_QUEUE_UPLOAD;
    /* get request from queue */
    newReq = (Telm_Upload_Info_Request *)prvTelmApp_Queue_Get(TELM_QUEUE_UPLOAD);
    ATProt_Select_Tx(AT_CONNECT_ID_MAIN);
    if(newReq == NULL)
    {
        //if there has no data for enterprise platform, check data for GB platform
         newReq = (Telm_Upload_Info_Request *)prvTelmApp_Queue_Get(TELM_QUEUE_GB_UPLOAD);
         channel = TELM_QUEUE_GB_UPLOAD;
         ATProt_Select_Tx(AT_CONNECT_ID_GB);
    } 
    if(true == prvTelmApp_Upload_Preprocess(newReq))
    {//First check, the upload necessary data must be prepared ok. 
        prvTelmApp_Collect_UTC_Time(&lastInfo);
        switch(newReq->evt)
        {
            case TELM_SEC_EVT_LOGIN:
                lastInfo.cmd_id=0x01;
                break;
            case TELM_SEC_EVT_LOGOUT:
                lastInfo.cmd_id=0x04;
                break;
            case TELM_SEC_EVT_REALTIME:
                lastInfo.cmd_id=0x02;
                break;
            case TELM_SEC_EVT_BACKUP:
                lastInfo.cmd_id=0x03;
                break;
            case TELM_SEC_EVT_HEARTBEAT:
                lastInfo.cmd_id=0x07;
                break;  
            case TELM_SEC_EVT_ACK:
#if 0
            case TELM_SEC_EVT_ENC_KEY:
                lastInfo.cmd_id=0x80;
                break;
#endif
            default:
                lastInfo.cmd_id=0x00;
                break;
        }
        lastInfo.ack_flag=0xFE;
        PRM_GetVIN(lastInfo.vin);
        if(true == vTelmProt_UploadInfo(newReq,&lastInfo))
        {
            //Second check, the upload data must be decoded ok and start the upload process. 
            /* queue check ok, set to busy mode.*/
            TelmApp_Set_Upload_state(TELM_STATE_BUSY);

            /* Start up the server ack timer.*/
            telm_app_server_ack_timer = OS_Time() + TELM_TIME_SERVER_ACK_TIMEOUT;
        }
        else
        {
            prvTelmApp_Queue_Shift(channel);
            TelmApp_Set_Upload_state(TELM_STATE_IDLE);
        }
    }
    else
    {
        //Preprocess check error, drop the upload information and set upload state to idle.
        prvTelmApp_Queue_Shift(channel);
        TelmApp_Set_Upload_state(TELM_STATE_IDLE);
    }
}

static void prvTelmApp_Initial(void)
{
    telm_server_ack = false;
    telm_server_ack_result = false;
    telm_app_upload_retrytimes = 0;
    telm_app_server_ack_timer = 0;
}

static void prvTelmProt_Check_Server_Response(void)
{
    if (true == telm_server_ack)
    {
		/* when recieved ACK, Stop Retrying and Timeout Timer */
		telm_app_upload_retrytimes = 0;
		telm_app_server_ack_timer = 0;
		
		/* Check server ACK or NAK, current is always ACK, need check */
		if(telm_server_ack_result == true) 
		{
			/* Receive the ACK,now report the result:ACK */
			vTelmApp_Inform_Upload_Result(TELM_UPLOAD_RES_OK);
		}
		else
		{
			/* Receive the NAK,now report the result:NG */
			vTelmApp_Inform_Upload_Result(TELM_UPLOAD_RES_NG);			
		}				
	}
    else
    {
        if (telm_app_server_ack_timer < OS_Time())
        {
            if (0 == telm_app_server_ack_timer)
            {
                /* Receive the ACK,now report the result:ACK */
                vTelmApp_Inform_Upload_Result(TELM_UPLOAD_RES_OK);
                return;
            }
            telm_app_upload_retrytimes++;
            if ((TELM_UPLOAD_RETRY_MAX) <= telm_app_upload_retrytimes )
            {
//                vATProt_sendAT_Command(AT_CMD_QICLOSE, NULL, NULL);
                /* Retry max times ,now report the result:timeout */
                 //2017.9.28 lihaibin modify, notify  record_task
#if 1
                OS_Send_Message(OS_RECORD_TASK, 
                                Build_Message(RECORD_EVT_RESET, 
                                (int16_t)ATProt_GetCurrentDataSession()));
#endif
                //end 2017.9.28 lihaibin modify, notify  record_task
                vATProt_Com_Reset();

                vATProt_Power_Off();
                rl_delay_without_schedule(1100);
                vATProt_Power_On();
                vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
                vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
                
                telm_app_upload_retrytimes = 0;
                vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
            }
            else
            {
                /* send data by tcp/ip to server again , get the next data is the same data*/
                //vATProt_sendAT_Command(AT_CMD_SEND_POST_DATA, NULL, NULL);
                //telm_app_server_ack_timer = OS_Time() +  TELM_TIME_SERVER_ACK_TIMEOUT;
            }
        }
    }
}

static void TelmApp_checkUploadState(void)
{
    switch(TelmApp_Get_Upload_state())
    {
        case TELM_STATE_IDLE:
            prvTelmApp_Initial();
            prvTelmApp_sendNextData();
            break;
        case TELM_STATE_BUSY:
            prvTelmProt_Check_Server_Response();
            //vTelmApp_Inform_Upload_Result(TELM_UPLOAD_RES_OK);
            break;
        default:
            break;
    }
}
#if 0
static void TelmApp_checkCommandState(void)
{

}
#endif
/* The loop is executed after AT receive, before AT send. */
/* To ensure the received message is checked at once, and can send out the latest upload information quickly*/
extern void vTelmApp_main_loop(void)
{
    //GPRS connected.
    if (NET_TCP_CONNECTED <= vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
    {
        /*Server command may fill upload info to queue, so check command first. */
        TelmApp_checkUploadState();
    }
}

#if 0
static uint8_t gps_last_pos[2][VEHICLE_STOP_GPS_CMP_LEN];

void gps_write_nv(void)
{
    gps_data_t gps_info;
    Gps_Store_Type_t tmp_gps;
    if (gps_backup == 0)
        return;
    vGps_Get_Gps_Info(&gps_info);
    {
        if ((0 == strncmp((char *)gps_info.longitude,(char *)gps_last_pos[0],VEHICLE_STOP_GPS_CMP_LEN)) &&
            (0 == strncmp((char *)gps_info.latitude,(char *)gps_last_pos[1],VEHICLE_STOP_GPS_CMP_LEN)))
            return;
    }
    {
        if (tmp_gps.utc_time[0] != '2')//invalid UTC time,remark as invalid
        {
            tmp_gps.valid = 0;
        }
        else if(gps_info.valid)
        {
           tmp_gps.valid = 1;
           if(gps_is_first_fixed == 0)
           {
               tmp_gps.valid = 2;//mark this gps message as first-fixed
               gps_is_first_fixed = 1;
           }
        }
        else
        {
            tmp_gps.valid = 0;
        }
        tmp_gps.temp = 0;
        tmp_gps.intake_map = 0;
        tmp_gps.intake_temp = 0;
        tmp_gps.throttle = 0;
#ifdef UPLOAD_GPS_COG
        ;
#elif defined (UPLOAD_GPS_SIG)
        {
            uint8_t sig_data[8];
            GPS_Get_Sig(sig_data);
            tmp_gps.temp = ((sig_data[0] << 2)& 0xf0) + ((sig_data[1] >> 2) & 0x0f);
            tmp_gps.intake_map = ((sig_data[2] << 2)& 0xf0) + ((sig_data[3] >> 2) & 0x0f);
            tmp_gps.intake_temp = ((sig_data[4] << 2)& 0xf0) + ((sig_data[5] >> 2) & 0x0f);
            tmp_gps.throttle = ((sig_data[6] << 2)& 0xf0) + ((sig_data[7] >> 2) & 0x0f);
        }
#endif
#if 0
        if(2 == tmp_gps.valid)
        {
               /* Save travel start to flash */
               travel_start_nv_write((uint8_t *)&tmp_gps);
               return;
        }
#endif
#if 0
        sFLASH_WriteBuffer((unsigned char *)&tmp_gps,GPS_BACKUP_ADDR+(gps_backup_tail*0x1000)+(backup_info[gps_backup_tail].store_pos*sizeof(Gps_Store_Type_t)),sizeof(Gps_Store_Type_t));

        if (backup_info[gps_backup_tail].store_pos < GPS_BACKUP_RECORD_PER_SECTOR)
        {
            backup_info[gps_backup_tail].store_pos++;
        }
        else
        {
            if (gps_backup_tail < (GPS_BACKUP_SECTOR_MAX-1))
            {
                gps_backup_tail++;
            }
            else
            {
                gps_backup_tail = 0;
            }
            clear_nv_gpsdata(gps_backup_tail);
            backup_info[gps_backup_tail].store_pos=0;
            backup_info[gps_backup_tail].read_pos=0;
        }
#endif
    }
}

static uint8_t get_nv_gpsdata(uint8_t *data)
{
    return 0;
}

uint8_t gps_backup_empty(void)
{
    {
        return 0;
    }
}

void clear_nv_gpsdata(uint8_t sector)
{
//    if (sector < GPS_BACKUP_SECTOR_MAX)
//        sFLASH_EraseSector(GPS_BACKUP_ADDR+sector*0x1000);
}

void set_gps_upload_backup(uint8_t enable)
{
    gps_backup = enable;
}

uint8_t gps_upload_status(void)
{
    return gps_backup;
}


static uint8_t gps_buffer_head = 0;
static uint8_t gps_buffer_tail = 0;
static void clear_gps_buffer(void)
{
    gps_buffer_head = 0;
    gps_buffer_tail = 0;
}

void save_gps_buffer(void)
{
    gps_data_t gps_info;
    Gps_Store_Type_t tmp_gps;

    vGps_Get_Gps_Info(&gps_info);

    {
        uint16_t cog = GPS_Parse_Cog(gps_info.cog);
        if (tmp_gps.utc_time[0] != '2')//invalid UTC time,remark as invalid
        {
            tmp_gps.valid = 0;
        }
        else if(gps_info.valid)
        {
            tmp_gps.valid = 1;
            if(gps_is_first_fixed == 0)
            {
                tmp_gps.valid = 2;//mark this gps message as first-fixed
                gps_is_first_fixed = 1;
            }
        }
        else
        {
            tmp_gps.valid = 0;
        }
        tmp_gps.temp = 0;
        tmp_gps.intake_map = 0;
        tmp_gps.intake_temp = 0;
        tmp_gps.throttle = 0;
#ifdef UPLOAD_GPS_COG
        tmp_gps.intake_map = cog/255;
        tmp_gps.intake_temp = cog%255;
#elif defined (UPLOAD_GPS_SIG)
        {
            uint8_t sig_data[8];
            GPS_Get_Sig(sig_data);
            tmp_gps.temp = ((sig_data[0] << 2)& 0xf0) + ((sig_data[1] >> 2) & 0x0f);
            tmp_gps.intake_map = ((sig_data[2] << 2)& 0xf0) + ((sig_data[3] >> 2) & 0x0f);
            tmp_gps.intake_temp = ((sig_data[4] << 2)& 0xf0) + ((sig_data[5] >> 2) & 0x0f);
            tmp_gps.throttle = ((sig_data[6] << 2)& 0xf0) + ((sig_data[7] >> 2) & 0x0f);
        }
#endif
    }  
    if (gps_info.valid)
        store_last_pos((uint8_t *)&gps_info);
}

static uint8_t get_gps_buffer(uint8_t *drive_data)
{

}


static void load_last_pos(void)
{

}

void store_last_pos(uint8_t *data)
{

}

void write_flash_last_pos(void)
{
}

static void get_last_pos(uint8_t *lon, uint8_t *lat)
{

}

uint8_t gps_buffer_empty(void)
{
    if (gps_buffer_tail > (gps_buffer_head))
    {
        return 0;
    }
    else
    {
        clear_gps_buffer();
        return 1;
    }
}
#endif 

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
