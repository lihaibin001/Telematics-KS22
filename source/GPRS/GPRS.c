#include "GPRS.h"
#include "debug.h"
#include "uart.h"
#include "board.h"
#include "timers.h"
#include "prj_config.h"
#include "MKS22F12.h"
/****
 *define
 *****/
#define AT_CMD_MAX_CNT 20   
#define EVENT_MAX_CNT 12
#define RX_DATA_MAX_CNT 4
#define TX_DATA_MAX_CNT 4
#define POWERKEY_PULL_UP()  IO_GSM_PWR_ON_OUT(0)	
#define POWERKEY_PULL_DOWN() IO_GSM_PWR_ON_OUT(1) 
#define GET_MODULE_POEWR_STATE() IO_GSM_STAUS()
#define POWER_SOURCE_ON() IO_4V_CTRL_OUT(1)
#define POWER_SOURCE_OFF() IO_4V_CTRL_OUT(0)
#define TASK_WAIT_MASK_TIME 20
#define MAX_MODLE_CMD_LEN 128
#define MAX_SEND_FAIL_CNT 3
#define MAX_HEARTBEAT_CNT 3
#define MAX_RECONNECT_CNT 3
#define HEX_IMEI_LEN 8
#define HEX_ICCID_LEN 10
#define HEX_IMSI_LEN 8
#define IP_SEND_TIMEOUT 60000
/*****
 *typedef
 ****/
/* function return value */
typedef enum
{
	ret_sucess,
	ret_fail,
	ret_malloc_err,
	ret_parm_err,
	ret_fatal_err,
}FnRet_t;

/* functoin pointor definetion */
typedef FnRet_t (*fpCmdHandler)(uint8_t *pData); 
typedef FnRet_t (*fpEventFnx)(void *pParm);

typedef struct
{
	uint8_t imei[HEX_IMEI_LEN];
	uint8_t imsi[HEX_IMSI_LEN];
	uint8_t iccid[HEX_ICCID_LEN];
}moduleInfo_t;

typedef enum
{
	PowerOnEvt,
	PowerOffEvt,
	GotoSleepEvt,
	ResetEvt,
	TestEvt,
	CheckCsqEvt,
	InitEvt,
	//	EstablishEvt,
	OpenIpEvt,
	DeactiveGprs,
	CloseSesion,
	IpDataReceiveEvt,
	EvtCnt,
}EventIdx_t;

/* URC code */
typedef enum
{
	UrcNotReady = 0,
	UrcNotInserted,
	UrcNormalPowd,
	UrcUnderVolPowd,
	UrcUnderVolWarn,
	UrcOverVolPowd,
	UrcOverVolWarn,
	UrcRdy,
	UrcCallReady,
	UrcSmsReady,
	UrcPdpDeact,
	UrcCgreg,
	UrcCreg,
	UrcIpd,
	UrcReceive,
	/*  urc below need to condition is multi connection */
	UrcConnectOk,
	UrcConnectFail,
	UrcAlreadyConnect,
	UrcSendOk,
	UrcSendFail,
	UrcClose,

	UrcNum,
}UrcCdoe_t;

/* query code */
typedef enum
{
	queryCsq,
	queryCgatt,
	queryNum,
}QueryCode_t;

/* urc handler object */
typedef struct
{
	uint8_t *pCode;
	fpCmdHandler handler;
}DecodeTbl_t;

/* gprs event type */
typedef struct
{
	uint8_t event;
	uint8_t *pData;
}Event_t;

/* net sesion's information */
typedef struct
{
	uint8_t *pHost;
	uint8_t *type;
	uint16_t port;
	netSesionState_t state;
	//    bool isActive;
}netSessionInfo_t;

/* express the module state */
typedef enum
{
	StateRecover,
	StatePowerOff,      //module is power off
	StatePowerDown,     //module is going to power off
	StatePowerUp,       //module is going to power on
	StatePowerOn,       //module is power on
	StateSleep,		    //module is sleep
	StateReady,		    //module is ready
	StateNetRegest,     //module is regested to network
	StateGprsRegest,    //module is regested to gprs network
	StateAttached,      //module is attached network
	//	StateConneted,	    //module has already connect to net
	StateError,
}ModuleState_t;

/* at command object */
typedef struct 
{
	uint8_t *pCommand;
	fpCmdHandler decoder;
	uint16_t len;
	uint32_t timeOut;
	uint16_t tryCnt;
}atCmd_tbl_t;

/* send data buffer */
typedef struct
{
	uint8_t *pData;
	uint16_t dataLen;
	ipSendCb cb;
	uint8_t channel;
}TxData_t;

/* receive data buffer */
typedef struct
{
	uint8_t *pData;
	uint16_t dataLen;
}RxData_t;
/***
 ** Local function
 **/
/* encode event */
static void ModuleInit(void);
static void ModulePowerDown(void);
static void ModulePowerUp(void);
static void ModuleRestart(void);
static void ModuleIpOpen(uint8_t channel);
static void ModuleSelfTeste(void);
static void ModuleCloseSesion(uint8_t channel);
static void ModuleCsqCheck(TimerHandle_t xTimer);
static void ModuleReceiveIpData(uint8_t channel);
//static void ModuleDeactive(void);
/* event handle function */
static FnRet_t modulePowerOnHandle(void *pParm);
static FnRet_t modulePowerOffHandle(void *pParm);
static FnRet_t moduleSleepHandle(void *pParm);
static FnRet_t moduleResetHandle(void *pParm);
static FnRet_t moduleTestHandle(void *pParm);
static FnRet_t moduleCsqCheckHandle(void *pParm);
static FnRet_t moduleInitializeHandle(void *pParm);
//static FnRet_t moduleEstbalishWirelessLinkHandle(void *pParm);
static FnRet_t moduleConnectToServerHandle(void *pParm);
static FnRet_t moduleDeactiveGprsHandle(void *pParm);
static FnRet_t moduleCloseSesionHandle(void *pParm);
static FnRet_t moduleIpDataReveiveHandle(void *pRram);

/* URC handler */
static FnRet_t UrcFatalHandler(uint8_t *pData);
static FnRet_t UrcNormalPowdHandler(uint8_t *pData);
static FnRet_t UrcRdyHandler(uint8_t *pData);
static FnRet_t UrcCallReadyHandler(uint8_t *pData);
static FnRet_t UrcSmsReadyHandler(uint8_t *pData);
static FnRet_t UrcConnectedHandler(uint8_t *pData);
static FnRet_t UrcDisConnectedHandler(uint8_t *pData);
static FnRet_t UrcSendSucessHandler(uint8_t *pData);
static FnRet_t UrcReceivedHandler(uint8_t *pData);
static FnRet_t UrcPdpDeactHandler(uint8_t *pData);
static FnRet_t UrcCgregHandler(uint8_t *pData);
static FnRet_t UrcCreghandler(uint8_t *pData);
static FnRet_t UrcSendFailHandler(uint8_t *pData);

/* query handler */
static FnRet_t QueryCsqHandler(uint8_t *pData);
static FnRet_t QueryCgattHandler(uint8_t *pData);

/***
 ** Local variable
 **/
static  netSessionInfo_t netSesionTbl[] = 
{
	{
		//.pHost = "101.37.87.65",
		.pHost = "59.44.43.234",
		.port = 30032,
		.type = (uint8_t *)"TCP",
		.state = ipSesionInactive,
		//        .isActive = true
	},
	{
		.pHost = "101.37.87.65",
		.port = 30032,
		.type = (uint8_t *)"TCP",
		.state = ipSesionInactive,
		//        .isActive = false
	}
};
static moduleInfo_t moduleInfo;
static const uint8_t netSesionNum = sizeof(netSesionTbl) / sizeof(netSessionInfo_t);
static TaskHandle_t TaskHandle;

static QueueHandle_t AtCmdQueue;
static QueueHandle_t EventQueue;

static TimerHandle_t csqCheckTimer;

//static QueueHandle_t TxDataQueue[sizeof(netSesionTbl) / sizeof(netSessionInfo_t)];
static QueueHandle_t TxDataQueue;
static QueueHandle_t RxDataQueue[sizeof(netSesionTbl) / sizeof(netSessionInfo_t)];
//static TickType_t ipDataSendTimer[sizeof(netSesionTbl) / sizeof(netSessionInfo_t)];
static TickType_t ipTransTimer;
//uncomment this if use SMS
//static QueueHandle_t MesageQueue;
//uncomment this if use Ftp
//static QueueHandle_t RxFtpQueue;

static TickType_t cmdExecuteTime;
static uint16_t cmdTryCnt;

static uint8_t cmdBuff[MAX_MODLE_CMD_LEN];
static uint16_t cmdBuffIdx;
static RxData_t RxTmp;
static uint8_t RxingSesion;
static uint16_t ipRxDataIdx;
static ipConnectCb ipOperateCb;

static ModuleState_t moduleState;

struct _linkDetect
{
	uint8_t heartbeat : 5;
	uint8_t reconnectCnt : 3;
};
struct _linkDetect linkDetecter[sizeof(netSesionTbl) / sizeof(netSessionInfo_t)];

struct _flag_strut
{
	uint8_t isTransmitterBusy : 1;
	uint8_t isNetReceving : 1;
	uint8_t isMsgReceving : 1;
	uint8_t isFtpReceving : 1;
	uint8_t powerDownReq : 1;
//	uint8_t sleepReq : 1;
	uint8_t isMulti : 1;
	uint8_t isEcho : 1;
	uint8_t isSimReady : 1;
	uint8_t isIpTransOk : 1;
//	uint8_t ipCloseReq : 1;
	uint8_t cmdState : 1; 
	uint8_t lastCsq : 5;
};
static struct _flag_strut localFlag = 
{
	.isEcho = 1,
};
/* event function table */
static fpEventFnx EventFucntion[EvtCnt] = 
{
	modulePowerOnHandle,
	modulePowerOffHandle,
	moduleSleepHandle,
	moduleResetHandle,
	moduleTestHandle,
	moduleCsqCheckHandle,
	moduleInitializeHandle,
	//	moduleEstbalishWirelessLinkHandle,
	moduleConnectToServerHandle,
	moduleDeactiveGprsHandle,
	moduleCloseSesionHandle,
	moduleIpDataReveiveHandle,
};

static uint8_t ownIp[4] = "";
static DecodeTbl_t urcTable[UrcNum] = 
{
	{"+CPIN: NOT READY", UrcFatalHandler},
	{"+CPIN: NOT INSERTED", UrcFatalHandler},
	{"NORMAL POWER DOWN", UrcNormalPowdHandler},
	{"UNDER-VOLTAGE POWER DOWN", UrcFatalHandler},
	{"UNDER-VOLTAGE WARNNING", UrcFatalHandler},
	{"OVER-VOLTAGE POWER DOWN", UrcFatalHandler},
	{"OVER-VOLTAGE WARNNING", UrcFatalHandler},
	{"RDY", UrcRdyHandler},
	{"Call Ready", UrcCallReadyHandler},
	{"SMS Ready", UrcSmsReadyHandler},
	{"+PDP: DEACT", UrcPdpDeactHandler},
	{"+CGREG:", UrcCgregHandler},
	{"+CREG:", UrcCreghandler},
	{"+IPD", UrcReceivedHandler},
	{"+RECEIVE", UrcReceivedHandler},

	{"CONNECT OK", UrcConnectedHandler},
	{"CONNECT FAIL", UrcDisConnectedHandler},
	{"ALREADY CONNECT", UrcConnectedHandler},
	{"SEND OK", UrcSendSucessHandler},
	{"SEND FAIL", UrcSendFailHandler},
	{"CLOSED", UrcDisConnectedHandler},
};

DecodeTbl_t queryTbl[queryNum] = 
{
	{"+CSQ:", QueryCsqHandler},
	{"+CGATT:", QueryCgattHandler},
};

ipRxDataHanle ipRxDataHandler = NULL;
/***
 ** Local function
 **/


/**************************Interface****************************/
static void interfaceInit(void)
{
	Uart_Initialize(UART_GSM_CHANNEL);
	/* Communication intterface init  */
}

static bool getOneByte(uint8_t *pByte)
{
	return Uart_Get_Char(UART_GSM_CHANNEL,pByte);
}

static FnRet_t portSend(uint8_t *pData, uint16_t len)
{
	uint16_t i;

	for(i = 0; i < len; i++)
	{
		Uart_Put_Char(UART_GSM_CHANNEL,*(pData + i));
	}
	return ret_sucess;
}

/***************************************************************/

static void handlerInit()
{
	EventQueue = xQueueCreate(EVENT_MAX_CNT, sizeof(Event_t));
	if(EventQueue == NULL)
	{
		for(;;)
		{
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Create event queue fail!\r\n");
		}
	}
	AtCmdQueue = xQueueCreate(AT_CMD_MAX_CNT, sizeof(atCmd_tbl_t));
	if(AtCmdQueue == NULL)
	{
		for(;;)
		{
			DEBUG(DEBUG_HIGH,  "[IOT] ERROR: Create AT command queue failed!\r\n");
		}
	}
	csqCheckTimer = xTimerCreate("csq checker", pdMS_TO_TICKS(1000 * 60), pdTRUE, (void *)0, ModuleCsqCheck);
	if(csqCheckTimer == NULL)
	{
		for(;;)
		{
			DEBUG(DEBUG_HIGH,  "[IOT] ERROR: Create csqCheckTimer failed!\r\n");
		}
	}
	xTimerStart(csqCheckTimer, 0);
}

static bool cleanAtCmdQueue(void)
{
#if 0
    bool isHeaderExe = false;
	atCmd_tbl_t command;
    atCmd_tbl_t TmpCommand;
	while(xQueueReceive(AtCmdQueue,&command,0) == pdPASS)
	{
        if(!isHeaderExe && localFlag.cmdState == 1)
        {
            memcpy(&TmpCommand, &command, sizeof(atCmd_tbl_t));
            isHeaderExe = true;
            continue;
        }
		if(command.pCommand)
		{
			vPortFree(command.pCommand);
		}
		command.pCommand = NULL;
	}

    if(isHeaderExe)
    {
        if(xQueueSend(AtCmdQueue, &command, 0) != pdPASS)
        {
            DEBUG(DEBUG_HIGH, "[IOT] Add command to queue failed!\r\n");
        }
    }
#endif
    atCmd_tbl_t command;
    while(xQueueReceive(AtCmdQueue,&command,0) == pdPASS)
	{
		if(command.pCommand)
		{
			vPortFree(command.pCommand);
		}
		command.pCommand = NULL;
	}

	return true;
}

static void cleanTxDataQueue(void)
{
	TxData_t data;
	while(xQueueReceive(TxDataQueue,&data,pdMS_TO_TICKS(50)) == pdPASS)
	{
        if(data.cb)
		{
			data.cb(false, data.channel);
		}
		if(data.pData != NULL)
		{
			vPortFree(data.pData);
			data.pData = NULL;
		}

	}
}

/********************************module operation***********************************************/
static bool isModuleStartUp(void)
{
	uint8_t debouncing = 10;
	while(GET_MODULE_POEWR_STATE())
	{
		vTaskDelay(pdMS_TO_TICKS(1));
		if(debouncing == 0)
		{
			return true;
		}
		debouncing--;
	}
	return false;
}

static bool isModulePowerOff(void)
{
	uint8_t debouncing = 10;
	while(!GET_MODULE_POEWR_STATE())
	{
		vTaskDelay(pdMS_TO_TICKS(1));
		if(debouncing == 0)
		{
			return true;
		}
		debouncing--;
	}
	return false;	
}

static bool powerUp(void)
{
	uint8_t timeOut = 0xFF;
	POWER_SOURCE_ON();
	if(isModuleStartUp())
	{
		moduleState = StatePowerOn;
		DEBUG(DEBUG_MEDIUM, "[IOT] ERROR: module already power on!\r\n"); 
		return true;
	}
	moduleState = StatePowerUp;
	POWERKEY_PULL_UP();
	vTaskDelay(pdMS_TO_TICKS(600)); //wait power stable
	POWERKEY_PULL_DOWN();
	vTaskDelay(pdMS_TO_TICKS(1100));
	POWERKEY_PULL_UP();
	vTaskDelay(pdMS_TO_TICKS(2100)); //wait power on
	while(!isModuleStartUp())
	{
		if(timeOut--)
		{
			vTaskDelay(10);
		}
		else
		{
			moduleState = StatePowerOff;
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Power up failed\r\n");
			return false;
		}
	}
	DEBUG(DEBUG_MEDIUM, "[IOT] Power on\r\n");
	moduleState = StatePowerOn;
	return true;
}

static bool powerDown(void)
{
	uint8_t timeOut = 0xFF;
	if(isModulePowerOff())
	{
		moduleState = StatePowerOff;
		DEBUG(DEBUG_MEDIUM, "[IOT] module already power off!\r\n"); 
		return true;
	}
	moduleState = StatePowerDown;
	POWERKEY_PULL_DOWN();
	vTaskDelay(pdMS_TO_TICKS(1800));
	while(!isModulePowerOff())
	{
		if(timeOut--)
		{
			vTaskDelay(10);
		}
		else
		{
			moduleState = StatePowerOn;
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Power off failed\r\n");
			return false;
		}
	}
	POWERKEY_PULL_UP();
	DEBUG(DEBUG_MEDIUM, "[IOT] Power off\r\n");
	moduleState = StatePowerOff;
	return true;	
}

static bool reset(void)
{
	powerDown();
	powerUp();
	return true;
}
/*****************************************************************************/

/*************************event encode*****************/
static void ModuleInit(void)
{
	Event_t event;
	event.event = InitEvt;
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add InitEvt event failed!\r\n");
		return ;
	}
}

static void ModulePowerDown(void)
{
	Event_t event;
	event.event = PowerOffEvt;
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add PowerOffEvt event failed!\r\n");
		return ;
	}

}

static void ModulePowerUp(void)
{
	Event_t event;
	event.event = PowerOnEvt;
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add PowerOnEvt event failed!\r\n");
		return ;
	}
}

static void ModuleRestart(void)
{
	Event_t event;
	event.event = ResetEvt;
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsResetEvt event failed!\r\n");
		return ;
	}		
}

static void ModuleIpOpen(uint8_t channel)
{
	Event_t event;
	if(channel >= netSesionNum)
	{
		return ;
	}
	if(localFlag.powerDownReq)
	{
		return ;
	}
	event.event = OpenIpEvt;
	event.pData = pvPortMalloc(sizeof(uint8_t));
    *event.pData = channel;
	if(event.pData == NULL)
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Malloc error. Line:%d!\r\n", __LINE__);
		return ;
	}
	memcpy(event.pData, &channel, sizeof(uint8_t));
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add OpenIpEvt event failed!\r\n");
		return ;
	}
	linkDetecter[channel].reconnectCnt++;
//	netSesionTbl[channel].state = ipSesionOpening;
}

static void ModuleSelfTeste(void)
{
	Event_t event;
	event.event = TestEvt;
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsTestEvt event failed!\r\n");
		return ;
	}
}

static void ModuleCloseSesion(uint8_t channel)
{
	Event_t event;
	event.event = CloseSesion;
	event.pData = pvPortMalloc(sizeof(uint8_t));
	if(channel >= netSesionNum)
	{
		return ;
	}

	if(event.pData == NULL)
	{
		DEBUG(DEBUG_HIGH, "[IOT] Malloc error. Line:%d\r\n",__LINE__);
		return ;
	}
	*event.pData = channel;
	if(pdPASS != xQueueSend(EventQueue, &event, pdMS_TO_TICKS(100)))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsTestEvt event failed!\r\n");
		return ;
	}	
	//netSesionTbl[channel].state = ipSesionClosing;  
}

static void ModuleCsqCheck(TimerHandle_t xTimer)
{
	Event_t event;
	event.event = CheckCsqEvt;    
	event.pData = NULL;
	if(pdPASS != xQueueSend(EventQueue, &event, pdMS_TO_TICKS(100)))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add CheckCsqEvt event failed!\r\n");
        return ;
	}	    
}

static void ModuleReceiveIpData(uint8_t channel)
{
	Event_t event;
	event.event = IpDataReceiveEvt;
	if(channel < netSesionNum)
	{
		event.pData = pvPortMalloc(sizeof(uint8_t));

		if(event.pData == NULL)
		{
			DEBUG(DEBUG_HIGH, "[IOT] Malloc error. Line:%d\r\n",__LINE__);
			return ;
		}
		*event.pData = channel;
		if(pdPASS != xQueueSend(EventQueue, &event, pdMS_TO_TICKS(100)))
		{
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsTestEvt event failed!\r\n");
			return ;
		}	
	}    
}
#if 0
static void ModuleDeactive(void)
{
	Event_t event;
	event.event = DeactiveGprs;
    event.pData = NULL;
    if(pdPASS != xQueueSend(EventQueue, &event, pdMS_TO_TICKS(100)))
    {
        DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsTestEvt event failed!\r\n");
    }
}
#endif
/*****************************************************************************/


//Recover records sesion after restart module
static void ModuleRecoverSesion(void)
{
	uint8_t sesionIdx;
	for(sesionIdx=0; sesionIdx<netSesionNum; sesionIdx++)
	{
		if(netSesionTbl[sesionIdx].state != ipSesionInactive)
		{
			ModuleIpOpen(sesionIdx);
		}
	}
}

/* used to push command object to at command queue */
static FnRet_t cmdSend(uint8_t *pCmd, uint16_t cmdLen, uint32_t timeOut, 
		uint16_t tryCunt, fpCmdHandler fnx)
{
	atCmd_tbl_t cmd;

	cmd.pCommand = pvPortMalloc(cmdLen+1);
	if(cmd.pCommand == NULL)
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Maolloc AT commond error\r\n");
		return ret_malloc_err;
	}
	memcpy(cmd.pCommand, pCmd, cmdLen);
    cmd.pCommand[cmdLen] = 0;
	cmd.decoder = fnx;
	cmd.timeOut = timeOut;
	cmd.tryCnt = tryCunt;
	cmd.len = cmdLen;
	if(pdPASS != xQueueSend(AtCmdQueue, &cmd, pdMS_TO_TICKS(100)))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Push command to command queue error\r\n");
		return ret_fatal_err;
	}
	return ret_sucess;
}

/* check the at command queue periodically, if find command item,send it */
static void cmdSender(void)
{
	if(moduleState < StatePowerOn)
	{
		return ;
	}
	atCmd_tbl_t command;
	if(pdPASS == xQueuePeek(AtCmdQueue, &command, 0))
	{
		//send command
		if(localFlag.cmdState == 0)
		{
			if(ret_sucess != portSend(command.pCommand, command.len))
			{
				DEBUG(DEBUG_HIGH, "[IOT] ERROR: Port send data failed\r\n");
				return ;
			}
			DEBUG(DEBUG_LOW,"[IOT] send cmd: %s\r\n",command.pCommand);
			cmdTryCnt = 1;
			cmdExecuteTime = pdMS_TO_TICKS(command.timeOut) + xTaskGetTickCount();
			localFlag.cmdState = 1;
		}
	}
	return ;	
}

/* detect the urc */
static UrcCdoe_t urcDetecter(uint8_t *pData)
{
	UrcCdoe_t idx = UrcNotReady;
	uint8_t startPos = 0;
	if(localFlag.isMulti == 1)
	{
		startPos = 3;
	}
	for(idx=UrcNotReady; idx<UrcConnectOk; idx++)
	{
		if(!memcmp(urcTable[idx].pCode, pData, strlen((char *)urcTable[idx].pCode)))
		{
			urcTable[idx].handler(pData);
			return idx;
		}
	}

	for(idx=UrcConnectOk; idx<UrcNum; idx++)
	{
		if(!memcmp(urcTable[idx].pCode, &pData[startPos], strlen((char *)urcTable[idx].pCode)))
		{
			urcTable[idx].handler(pData);
			return idx;
		}
	}
	return idx;
}

/* detect the query result */
static QueryCode_t queryDetecter(uint8_t *pData)
{
	QueryCode_t idx;
	for(idx=queryCsq; idx<queryNum; idx++)
	{
		if(!memcmp(queryTbl[idx].pCode, pData, strlen((char *)queryTbl[idx].pCode)))
		{
			queryTbl[idx].handler(pData);
			return idx;
		}
	}
	return idx;
}

/* this decoder wait for "OK"*/
static FnRet_t commonRespDecoder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }
	if(!memcmp(pResp, "OK", strlen("OK")))
	{
		return ret_sucess;
	}
	return ret_fail;
}

static FnRet_t iccidDeocder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return  ret_fail;
    }
    if(strlen((char *)pResp) == HEX_ICCID_LEN*2)
    {
        uint8_t idx;
        for(idx=0; idx<HEX_ICCID_LEN; idx++)
        {
            moduleInfo.iccid[idx] = StrtoHex(&pResp[2*idx]);
        }
    }
    if(memcmp(pResp, "OK", 2))
    {
        return ret_sucess;
    }
    return ret_fail;
}

static FnRet_t imeiDeocder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return  ret_fail;
    }
    if(strlen((char *)pResp) == HEX_IMEI_LEN * 2 - 1)
    {
        uint8_t idx;
        pResp[HEX_IMEI_LEN * 2 - 1] = 'f'; //
        for(idx=0; idx<HEX_IMEI_LEN; idx++)
        {
            moduleInfo.imei[idx] = StrtoHex(&pResp[2*idx]);
        }
    }
    if(memcmp(pResp, "OK", 2))
    {
        return ret_sucess;
    }
    return ret_fail;
}

static FnRet_t echoCloseDecoder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }
	if(!memcmp(pResp, "OK", strlen("OK")))
	{
		localFlag.isEcho = 0;
		return ret_sucess;
	}
	return ret_fail;    
}

/* if module has regested to network, return sucess */
static FnRet_t checkNetWorkRegest(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }    
	if(!memcmp(pResp, "OK", strlen("OK")))
	{
		if(moduleState >= StateNetRegest)
		{
			return ret_sucess;
		}
	}
	return ret_fail;
}

/* if module has registered to gprs network, return sucess; */
static FnRet_t checkGprsRegest(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }
	if(!memcmp(pResp, "OK", strlen("OK")))
	{
		if(moduleState >= StateGprsRegest)
		{
			return ret_sucess;
		}
	}
	return ret_fail;
}

/* if gprs server has attached to module, return sucess */
static FnRet_t cgattRespDecoder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }
	if(!memcmp(pResp, "OK", strlen("OK")))
	{
		if(moduleState >= StateAttached)
		{
			return ret_sucess;
		}
	}
	return ret_fail;
}

/* Decode local ip address */
static FnRet_t cifsrRespDecoder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return ret_fail;
    }
	int byte0 = 0, byte1 = 0, byte2 = 0, byte3 = 0;
	sscanf((char *)pResp, "%d.%d.%d.%d", &byte0,&byte1,&byte2,&byte3);
	ownIp[0] = byte0;
	ownIp[1] = byte1;
	ownIp[2] = byte2;
	ownIp[3] = byte3;
	if(ownIp[0] == 10)
	{
		return ret_sucess;
	}
	return ret_fail;
}

/* Deocde ip send command. wait for '>' */
static FnRet_t ipSendRespDecoder(uint8_t *pData)
{
	FnRet_t ret = ret_fail;
	if(pData == NULL)
	{
		return ret_fail;
	}

	if(!memcmp(pData, "> ", 2))
	{
		atCmd_tbl_t command;
		uint8_t sesionNum;
        uint16_t dataLen;
		if(pdPASS == xQueuePeek(AtCmdQueue, &command, 0))
		{
			TxData_t Data;
			if(localFlag.isMulti)
			{
				sesionNum = atoi((char *)&command.pCommand[11]);
                dataLen = atoi((char *)&command.pCommand[13]);
				if(sesionNum >= netSesionNum)
				{
					DEBUG(DEBUG_HIGH, "[IOT] ERROR: sesion index error\r\n");   
				}
			}
			else
			{
				sesionNum = 0;
			}
			if(TxDataQueue != NULL)
			{
				if(pdPASS == xQueuePeek(TxDataQueue, &Data, 0))
				{
					if(Data.pData == NULL)
					{
						DEBUG(DEBUG_HIGH, "[IOT] ERROR: Receive >, but data in queue error!\r\n");
					}
                    if(dataLen != Data.dataLen)
                    {
                        DEBUG(DEBUG_HIGH, "[IOT] ERROR: data len error\r\n");
                    }
					else
					{
                        #if DEBUG_LOW == DEBUG_LVL
                        {
                            uint16_t idx;
                            DEBUG(DEBUG_LOW, "[IOT] Tx:");
                            for(idx=0; idx<Data.dataLen; idx++)
                            {
                                DEBUG(DEBUG_LOW, "%02X",Data.pData[idx]);
                            }
                            DEBUG(DEBUG_LOW, "\r\n");
                        }
                        #endif
                            
						portSend(Data.pData, Data.dataLen);	
						ret = ret_sucess;
					}
				}
			}
			else
			{
				DEBUG(DEBUG_HIGH, "[IOT] ERROR: Receive >, but have no data to sent!\r\n");
			} 
		}
	}
	else
	{
		DEBUG(DEBUG_LOW, "[IOT] \"> \"decode error\r\n")
	}
	return ret;
}

/* Decode CSQ value */
static FnRet_t csqCheckDecoder(uint8_t *pData)
{   
	if(localFlag.lastCsq > 0)
	{
		return ret_sucess;
	}
	return ret_fail;
}

static FnRet_t waitCloseOk(uint8_t *pData)
{
    atCmd_tbl_t command;
    uint8_t channel = 0;
    if(pData == NULL)
    {
    	return ret_fail;
    }
	if(localFlag.isMulti)
	{
		channel = pData[0] - '0';
		if(!memcmp(pData+3, "CLOSE OK", strlen("CLOSE OK")))
		{
			netSesionTbl[channel].state = ipSesionInactive;
			cleanAtCmdQueue();
			return ret_sucess;
		}
	}

    return ret_fail;
}
#if 0
/* check power down */
static FnRet_t waitPowerDownOk(uint8_t *pData)
{
    if(moduleState == StatePowerOff)
    {
        return ret_sucess;
    }
    return ret_fail;
}
#endif
/* Check receive data */
static void receiveChecker(void)
{
	atCmd_tbl_t command;
	FnRet_t ret = ret_fail;
	uint8_t byte;
	bool findEnd = false;
	BaseType_t hasCmd = xQueuePeek(AtCmdQueue, &command, 0);
	while(getOneByte(&byte))
	{
		if(localFlag.isNetReceving)
		{
			if(RxTmp.pData != NULL)
			{
				RxTmp.pData[ipRxDataIdx++] = byte;
			}
			if(ipRxDataIdx == RxTmp.dataLen)
			{
				if(RxDataQueue[RxingSesion] != NULL)
				{   
					if(pdPASS != xQueueSend(RxDataQueue[RxingSesion], &RxTmp, pdMS_TO_TICKS(500)))
					{
						vPortFree(RxTmp.pData);
						DEBUG(DEBUG_HIGH, "[IOT] send rx data queue error!\r\n");
					}
					else
					{
#if DEBUG_LOW == DEBUG_LVL
                        {
                            uint16_t idx = 0;
                            DEBUG(DEBUG_LOW, "[IOT] RX:");
                            for(idx=0; idx<RxTmp.dataLen; idx++)
                            {
                                DEBUG(DEBUG_LOW, "%02X", RxTmp.pData[idx]);
                            }
                            DEBUG(DEBUG_LOW, "\r\n");
                        }
#endif  
						ModuleReceiveIpData(RxingSesion);
					}
				}
				else
				{
					vPortFree(RxTmp.pData);
					DEBUG(DEBUG_HIGH, "[IOT] Has no Rx queue\r\n");
				}
				localFlag.isNetReceving = 0;
				RxTmp.pData = NULL;
			}
		}
		else if(localFlag.isMsgReceving)
		{

		}
		else
		{
			//recieve module response
			switch(byte)
			{
				case '\r':
					continue;
				case '\n':
					if(cmdBuffIdx >= 1)
					{
						findEnd = true;
					}
					break;
				case ' ':
					cmdBuff[cmdBuffIdx++] = byte;
					if(cmdBuffIdx == 2 && cmdBuff[cmdBuffIdx-2] == '>')
					{
						findEnd = true;
						break;
					}			
					continue;
				default:
					cmdBuff[cmdBuffIdx++] = byte;
					continue;
			}
			if(findEnd)
			{
				findEnd = false;

				if(cmdBuffIdx == 0)
				{
					continue;
				}
				cmdBuffIdx = 0;
				DEBUG(DEBUG_LOW, "[IOT] Module resp:%s\r\n", cmdBuff);
				//parse urc first 
				if(UrcNum != urcDetecter(cmdBuff))
				{
					memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
					continue ;
				}
				// parse query command
				if(queryNum != queryDetecter(cmdBuff))
				{
					memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
					continue ;
				}
				//if has at command in at command queue, check it
				if(pdPASS == hasCmd && localFlag.cmdState == 1)
				{

					//if(localFlag.isEcho )
					if(!memcmp(command.pCommand, cmdBuff, strlen((char *)cmdBuff)))
					{
						memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
						// echo, continue
						continue ;
					}
					else if(!memcmp("ERROR", cmdBuff, strlen("ERROR")))
					{
						memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
						localFlag.cmdState = 0;

						cmdTryCnt = 0;
						if(moduleState != StateRecover)
						{
							cleanAtCmdQueue();
							moduleState = StateRecover;
							ModuleSelfTeste();
							ModuleRestart();
							ModuleInit();
							ModuleRecoverSesion();
						}
						//response error, reset module?
						DEBUG(DEBUG_HIGH, "[IOT] ERROR\r\n");
					}
					else
					{
						//decode response
						if(command.decoder)
						{
							ret = command.decoder(cmdBuff);
							memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
							if(ret == ret_sucess)
							{

								// Receive expected response, if can delay forever????????????????
								if(pdPASS == xQueueReceive(AtCmdQueue, &command, pdMS_TO_TICKS(100)))
								{
									if(command.pCommand)
									{
										vPortFree(command.pCommand);	
										command.pCommand = NULL;
									}
								}
								else
								{
									DEBUG(DEBUG_HIGH, "[IOT] Error: when delete the unused command!\r\n");
								}
							}
							else
							{

								if(xTaskGetTickCount() >= cmdExecuteTime)
								{
                                    if(cmdTryCnt > command.tryCnt)
                                    {
                                        cleanAtCmdQueue();
                                        ModuleRestart();
                                        ModuleInit();
                                        ModuleRecoverSesion();
                                        return ;
                                    }
									cmdExecuteTime = pdMS_TO_TICKS(command.timeOut) + xTaskGetTickCount();
									cmdTryCnt++;
								}
							}

						}
						else
						{

							memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
							// Receive expected response, if can delay forever????????????????
							if(pdPASS == xQueueReceive(AtCmdQueue, &command, pdMS_TO_TICKS(100)))
							{
								if(command.pCommand)
								{
									vPortFree(command.pCommand);	
									command.pCommand = NULL;
								}

							}
							else
							{
								DEBUG(DEBUG_HIGH, "[IOT] Error: when delete the unused command!\r\n");
							}
							DEBUG(DEBUG_HIGH, "[IOT] Error: decoder = NULL!\r\n");
						}
						localFlag.cmdState = 0;
						continue ;
					}					
				}
			}
		}
	}
	//if uart buffer has no data, check the at command queue.
	if(pdPASS == hasCmd && localFlag.cmdState == 1)
	{
#if 1
        if(command.decoder)
        {
            if(command.decoder(NULL) == ret_sucess)
            {
                // Receive expected response, if can delay forever????????????????
                if(pdPASS == xQueueReceive(AtCmdQueue, &command, pdMS_TO_TICKS(100)))
                {
                    if(command.pCommand)
                    {
                        vPortFree(command.pCommand);	
                        command.pCommand = NULL;
                    }
                }
                else
                {
                    DEBUG(DEBUG_HIGH, "[IOT] Error: when delete the unused command!\r\n");
                }                
            }
        }
#endif

		if(xTaskGetTickCount() >= cmdExecuteTime)
		{
            if(cmdTryCnt > command.tryCnt)
            {
                cleanAtCmdQueue();
                localFlag.cmdState = 0;
                cmdTryCnt = 0;
                ModuleRestart();
                ModuleInit();
                ModuleRecoverSesion();
            }
			cmdExecuteTime = pdMS_TO_TICKS(command.timeOut) + xTaskGetTickCount();            
			localFlag.cmdState = 0;
			cmdTryCnt++;
		}
	}
}

/****************************** AT command package *****************************/
static FnRet_t ipOpen(uint8_t channel, uint8_t *pHost, uint32_t port, uint8_t *type)
{
	uint8_t ipOpenCmd[128] = "";
	if(localFlag.isMulti)
	{
		if(sprintf((char *)ipOpenCmd, "AT+CIPSTART=%d,\"%s\",\"%s\",%d\r\n", channel, (char *)type, pHost, port) == -1)
		{
                DEBUG(DEBUG_HIGH, "[IOT] Error: When encode TCP/IP open command");
			return ret_fail;
		}
	}
	else
	{
		if(sprintf((char *)ipOpenCmd, "AT+CIPSTART=\"%s\",\"%s\",%d\r\n", (char *)type, pHost, port) == -1)
		{
			DEBUG(DEBUG_HIGH, "[IOT] Error: When encode TCP/IP open command");
			return ret_fail;
		}        
	}
	if(cmdSend(ipOpenCmd, strlen((char *)ipOpenCmd), 7500, 1, commonRespDecoder) != ret_sucess)
	{
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t gotoSleep(void)
{
	if(cmdSend("AT+CSCLK=0\r\n", strlen("AT+CSCLK=0\r\n") ,500, 1, commonRespDecoder) != ret_sucess)
	{
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t test(void)
{
	if(cmdSend("AT\r\n", strlen("AT\r\n") ,500, 100, commonRespDecoder) != ret_sucess)
	{
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t deactiveGprs(void)
{
	if(cmdSend("at+cipshut\r\n", strlen("at+cipshut\r\n"), 500, 1, commonRespDecoder))
	{
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t closeSesion(uint8_t sesion)
{
	if(sesion >= netSesionNum)
	{
		DEBUG(DEBUG_HIGH, "[IOT] close sesion num error\r\n");
		return ret_fail;
	}
	if(localFlag.isMulti == 0)
	{
		if(cmdSend("at+cipclose=1\r\n", strlen("at+cipclose=1\r\n"), 500, 1, waitCloseOk))
		{
			DEBUG(DEBUG_HIGH, "[IOT] closeSesion add At com error\r\n");
			return ret_fail;
		}        
	}
	else
	{
		uint8_t cmd[64] = "";
		sprintf((char *)cmd, "at+cipclose=%d,1\r\n", sesion);
		if(cmdSend(cmd, strlen((char *)cmd), 1000, 1, waitCloseOk))
		{
			DEBUG(DEBUG_HIGH, "[IOT] closeSesion add At com error\r\n");
			return ret_fail;
		}
	}
	return ret_sucess;
}

static FnRet_t csqCheck(void)
{
	cmdSend("AT+CSQ\r\n",strlen("AT+CSQ\r\n"),2000,1,csqCheckDecoder);
	return ret_sucess;
}

#if 0
static FnRet_t powerDownByCommand(void)
{
	cmdSend("AT+CPOWD=1\r\n",strlen("AT+CPOWD=1\r\n"),5000,1,waitPowerDownOk);
	return ret_sucess;    
}
#endif
static FnRet_t netInit(void)
{
	uint8_t cmd[64] = "";
	cmdSend("AT+IPR=115200\r\n",strlen("AT+IPR=115200\r\n"),500,1,commonRespDecoder);

	cmdSend("AT+CSQ\r\n", strlen("AT+CSQ\r\n"), 500, 500, csqCheckDecoder);
    cmdSend("AT+CCID\r\n",strlen("AT+CCID\r\n"),2000,1,iccidDeocder);
    cmdSend("AT+GSN\r\n",strlen("AT+GSN\r\n"),500,1,imeiDeocder);    
	localFlag.isMulti = 1;	
	cmdSend("AT+CREG?\r\n", strlen("AT+CREG?\r\n"), 500, 500, checkNetWorkRegest);	
	cmdSend("AT+CGREG?\r\n", strlen("AT+CGREG?\r\n"), 500, 500, checkGprsRegest);	
	cmdSend("AT+CGATT?\r\n", strlen("AT+CGATT?\r\n"), 500, 500, cgattRespDecoder);
	cmdSend("AT+CIPMUX=1\r\n", strlen("AT+CIPMUX=1\r\n"), 500, 1, commonRespDecoder);
	sprintf((char *)cmd, "AT+CSTT=\"%s\"\r\n", (char *)"CMNET");
	cmdSend(cmd, strlen((char *)cmd), 500, 1, commonRespDecoder);
	cmdSend("AT+CIICR\r\n",strlen("AT+CIICR\r\n"), 85000, 1, commonRespDecoder);
	cmdSend("AT+CIFSR\r\n", strlen("AT+CIFSR\r\n"), 500, 1, cifsrRespDecoder);

	cmdSend("ATE0\r\n", strlen("AT0\r\n"), 500, 1, echoCloseDecoder);
	return ret_sucess;
}
/****************************************************************************/

#if 0
static FnRet_t estbalishWirelessLink(uint8_t *pApn)
{
	return ret_sucess;
}
#endif

static void ipTransmitter(void)
{
	uint8_t cmd[32] = "";
	TxData_t data;
	if(ipTransTimer <= xTaskGetTickCount())
	{
		localFlag.isTransmitterBusy = 0;
	}
	if(localFlag.isTransmitterBusy || TxDataQueue == NULL)
	{
		return;
	}

	if(localFlag.isMulti)
	{    
		if(pdPASS == xQueuePeek(TxDataQueue, &data, 0))
		{  
			sprintf((char *)cmd, "AT+CIPSEND=%d,%d\r\n",data.channel, data.dataLen);
		}
		else
		{
			return ;
		}
	}
	else
	{
		if(pdPASS == xQueuePeek(TxDataQueue, &data, 0))
		{
			sprintf((char *)cmd, "AT+CIPSEND=%d\r\n", data.dataLen);
		}
		else
		{
			return;
		}
	}
	if(linkDetecter[data.channel].heartbeat >= MAX_HEARTBEAT_CNT)
	{
		//check current sesion state
		ModuleCloseSesion(data.channel);
	}
	if(netSesionTbl[data.channel].state == ipSesionOpened)
	{
		if(cmdSend(cmd, strlen((char *)cmd) ,IP_SEND_TIMEOUT-10, 1, ipSendRespDecoder) != ret_sucess)
		{ 
			DEBUG(DEBUG_HIGH,"[IOT] Error: When send data!\r\n");
			return ;
		}
		localFlag.isTransmitterBusy = 1;
		linkDetecter[data.channel].heartbeat++;
		ipTransTimer = pdMS_TO_TICKS(IP_SEND_TIMEOUT) + xTaskGetTickCount();
	}
}

/* event handle */
static FnRet_t modulePowerOnHandle(void *pParm)
{
	powerUp();
	return ret_sucess;
}

static FnRet_t modulePowerOffHandle(void *pParm)
{
	powerDown();
	return ret_sucess;
}

static FnRet_t moduleInitializeHandle(void *pParm)
{
	netInit();
	return ret_sucess;
}

static FnRet_t moduleSleepHandle(void *pParm)
{
	gotoSleep();
	return ret_sucess;
}

static FnRet_t moduleResetHandle(void *pParm)
{
	reset();
	return ret_sucess;
}

static FnRet_t moduleTestHandle(void *pParm)
{
	test();
	return ret_sucess;
}

static FnRet_t moduleCsqCheckHandle(void *pParm)
{
	csqCheck();
	return ret_sucess;
}

#if 0
static FnRet_t moduleEstbalishWirelessLinkHandle(void *pParm)
{
	estbalishWirelessLink("CMNET");
	return ret_sucess;
}
#endif

static FnRet_t moduleConnectToServerHandle(void *pParm)
{
	uint8_t sid = *(uint8_t *)pParm;
	ipOpen(sid, netSesionTbl[sid].pHost, netSesionTbl[sid].port, netSesionTbl[sid].type);
	return ret_sucess;
}

static FnRet_t moduleDeactiveGprsHandle(void *pParm)
{
	deactiveGprs();
	return ret_sucess;
}

static FnRet_t moduleIpDataReveiveHandle(void *pParm)
{
	uint8_t sesionNum = *(uint8_t *)pParm;
	if(sesionNum >= netSesionNum)
	{
		DEBUG(DEBUG_HIGH, "[IOT] sesion num error\r\n");
	}
	if(RxDataQueue[sesionNum] != 0)
	{
		RxData_t data;
		if(xQueueReceive(RxDataQueue[sesionNum], &data, 0) == pdPASS)
		{
			if(data.pData != NULL)
			{ 
				if(ipRxDataHandler)
				{
					ipRxDataHandler(data.pData, data.dataLen, sesionNum);
				}
				vPortFree(data.pData);
				data.pData = NULL;
				return ret_sucess;
			}
		}
	}
	return ret_fail;
}

static FnRet_t moduleCloseSesionHandle(void *pParm)
{
	closeSesion(*(uint8_t *)pParm);
	return ret_sucess;
}

/* URC handle */
static FnRet_t UrcFatalHandler(uint8_t *pData)
{
    if(moduleState <= StatePowerOn)
    {
        cleanAtCmdQueue();
#if 0
        if(moduleState == StateAttached) //??????????????
        {
            uint8_t idx;
            //close all sesion
            for(idx=0; idx<netSesionNum; idx++)
            {
                if(netSesionTbl[idx].state == ipSesionOpened)
                {
                    ModuleCloseSesion(idx);
                }
            }
            //detach net work
            ModuleDeactive();
        }
#endif
        moduleState = StateRecover;
        ModuleRestart();
        localFlag.cmdState = 0;
        ModuleInit();
        ModuleRecoverSesion(); 
    }
	return ret_sucess;
}
static FnRet_t UrcNormalPowdHandler(uint8_t *pData)
{
    //do not handle normal powerdown
	return ret_sucess;
}
static FnRet_t UrcRdyHandler(uint8_t *pData)
{
	moduleState = StateReady;
	return ret_sucess;
}

static FnRet_t UrcCallReadyHandler(uint8_t *pData)
{
    //do not handle
	return ret_sucess;
}

static FnRet_t UrcSmsReadyHandler(uint8_t *pData)
{
	return ret_sucess;
}

static FnRet_t UrcConnectedHandler(uint8_t *pData)
{
	uint8_t sesionNum = 0;
	if(localFlag.isMulti)
	{
		sesionNum = (uint8_t)atoi((char *)pData);
		if(sesionNum >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Session index error!\r\n");
			return ret_fail;
		}
	}
	linkDetecter[sesionNum].reconnectCnt = 0;
	netSesionTbl[sesionNum].state = ipSesionOpened;
    localFlag.isTransmitterBusy = 0;
	if(ipOperateCb)
	{
		ipOperateCb(true, sesionNum);
	}
	return ret_sucess;
}

static FnRet_t UrcDisConnectedHandler(uint8_t *pData)
{
	uint8_t sesionNum = 0;
	if(localFlag.isMulti)
	{
		sesionNum = (uint8_t )atoi((char *)pData);
		if(sesionNum >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: Session index error!\r\n");
			return ret_fail;            
		}
	}
	//cleanTxDataQueue();
    if(localFlag.powerDownReq == 1 || netSesionTbl[sesionNum].state == ipSesionClosing)
    {
        netSesionTbl[sesionNum].state = ipSesionInactive;
    }
	else
    { 
		//reconnect
		if(moduleState == StateAttached)
		{
			if(linkDetecter[sesionNum].reconnectCnt >= MAX_RECONNECT_CNT)
			{
				uint8_t idx=0;

				for(idx=0; idx<netSesionNum; idx++)
				{
					linkDetecter[idx].reconnectCnt = 0;
					linkDetecter[idx].heartbeat = 0;
				}
				moduleState = StateRecover;
				ModuleRestart();
				localFlag.cmdState = 0;
				cleanAtCmdQueue();
				ModuleInit();
				ModuleRecoverSesion();
			}

			else
			{
				ModuleIpOpen(sesionNum);
			}
		}
		netSesionTbl[sesionNum].state = ipSesionClose;
	}
    if(ipOperateCb)
    {
        ipOperateCb(false, sesionNum);
    }
	return ret_sucess;
}

static FnRet_t UrcSendSucessHandler(uint8_t *pData)
{
	TxData_t data;
	uint8_t sesionNum;
	if(localFlag.isMulti)
	{
		sesionNum = (uint8_t)atoi((char *)pData);
		if(sesionNum >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[MBCOM] Error: sesion num error!\r\n");
			return ret_fail;                   
		}
	}
	else
	{
		sesionNum = (uint8_t)0;
	}
	linkDetecter[sesionNum].heartbeat = 0;
	//    linkDetecter[sesionNum].reconnectCnt = 0;
	netSesionTbl[sesionNum].state = ipSesionOpened;
	localFlag.isTransmitterBusy = 0;
#if 1
	if(TxDataQueue != NULL)
	{
		if(xQueueReceive(TxDataQueue, &data, pdMS_TO_TICKS(TASK_WAIT_MASK_TIME)) == pdPASS)
		{
			if(data.cb)
			{
				data.cb(true, data.channel);
			}
			vPortFree(data.pData);	
			data.pData = NULL;
			data.cb = NULL;
			return ret_sucess;
		}
		DEBUG(DEBUG_HIGH, "[GPRS] Error: When delete data queue\r\n");
	}
#endif
	return ret_fail;
}

static FnRet_t UrcReceivedHandler(uint8_t *pData)
{
	uint8_t len = strlen("+RECEIVE,");
	localFlag.isNetReceving = true;
	pData+=len;
	if(*pData >= '0' && *pData <= '9')
	{
		RxingSesion = (uint8_t)atoi((char *)pData);
	}
	else
	{
		DEBUG(DEBUG_HIGH, "[MBCOM] Parse +RECEIVE error\r\n");
		return ret_fail;
	}
	while(*pData != ',' && *pData != 0)
	{
		pData++;
	}
	if(*pData != 0)
	{
		pData++;
		RxTmp.dataLen = (uint16_t)atoi((char *)pData);
	}
	else
	{
		RxTmp.dataLen = 0;
		DEBUG(DEBUG_HIGH, "[MBCOM] Parse +RECEIVE error\r\n");
		return ret_fail;        
	}
	if(RxDataQueue[RxingSesion] == NULL)
	{
		RxDataQueue[RxingSesion] = xQueueCreate(RX_DATA_MAX_CNT, sizeof(RxData_t));
		if(RxDataQueue[RxingSesion] == NULL)
		{
			RxTmp.dataLen = 0;
			DEBUG(DEBUG_HIGH, "[MBCOM] Create Rx queue error\r\n");
			return ret_fail;  
		}
	}
	RxTmp.pData = pvPortMalloc(RxTmp.dataLen);
	ipRxDataIdx = 0;
	if(RxTmp.pData == NULL)
	{
		RxTmp.dataLen = 0;
		DEBUG(DEBUG_HIGH, "[MBCOM] Malloc rx data error\r\n");
		return ret_fail;  
	}
	return ret_sucess;
}

static FnRet_t UrcPdpDeactHandler(uint8_t *pData)
{
#if 0
    if(moduleState == StateAttached)
    {
        cleanAtCmdQueue();
        moduleDeactive();
    }
	moduleInit();
#endif
	return ret_sucess;
}

static FnRet_t UrcCreghandler(uint8_t *pData)
{
	if(pData[9] == '1')
	{
		moduleState = StateNetRegest;
	}
	else
	{
	}
	return ret_sucess;
}

static FnRet_t UrcCgregHandler(uint8_t *pData)
{
	if(pData[10] == '1')
	{
		moduleState = StateGprsRegest;
	}
	else
	{

	}
	return ret_sucess;
}

static FnRet_t UrcSendFailHandler(uint8_t *pData)
{
	localFlag.isTransmitterBusy = 0;
	//check 
	return ret_sucess;
}


/* query handler */
static FnRet_t QueryCsqHandler(uint8_t *pData)
{
	uint8_t csq = atoi((char *)&pData[6]);
	localFlag.lastCsq = csq <= 31 ? csq : 0;
	return ret_sucess;	
}

static FnRet_t QueryCgattHandler(uint8_t *pData)
{
	if(pData[8] == '1')
	{
		moduleState = StateAttached;
	}
	return ret_sucess;		
}

void IOT_Task(void *pParm)
{
	interfaceInit();
	handlerInit();
	ModulePowerDown();
	ModulePowerUp();
	ModuleSelfTeste();
	ModuleInit();
	ModuleIpOpen(0);
	//ModuleIpOpen(1);
	for(;;)
	{
		Event_t event; 
		if(xQueueReceive(EventQueue, &event, pdMS_TO_TICKS(TASK_WAIT_MASK_TIME)) == pdPASS)
		{
			EventFucntion[event.event](event.pData);
			if(event.pData)
			{
				vPortFree(event.pData);
				event.pData = NULL;
			}
		}
		receiveChecker();
		ipTransmitter();
		cmdSender();
	}
}

TaskHandle_t IOT_Gethandle(void)
{
	return TaskHandle;
}

bool IOT_IpNetSend(uint8_t channel, uint8_t *pData, uint16_t len, ipSendCb cb)
{
	if(channel >= netSesionNum || pData == NULL || len == 0)
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: param error,\r\n");
		return  false;
	}
	if(IOT_GetSessionState(channel) == ipSesionOpened && !localFlag.powerDownReq)
	{
		TxData_t data;
		if(TxDataQueue == NULL)
		{
			TxDataQueue = xQueueCreate(TX_DATA_MAX_CNT, sizeof(TxData_t));
			if(TxDataQueue == NULL)
			{
				DEBUG(DEBUG_HIGH, "[GPRS] ERROR: Create tx queue failed!\r\n");
				return  false;                
			}
		}
		data.pData = pvPortMalloc(len);
		if(data.pData == NULL)
		{
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: When malloc the ip data!\r\n");
			return false;
		}	
		memcpy(data.pData, pData, len);
		data.dataLen = len;
		data.cb = cb;
		data.channel = channel;

		if(pdPASS == xQueueSend(TxDataQueue, &data, pdMS_TO_TICKS(500)))
		{
			return true;
		}
		else
		{
			DEBUG(DEBUG_HIGH, "[GPRS] ERROR: When Add Data to tx data queue!\r\n");
			vPortFree(data.pData);
		}

	}
	return false;
}

bool IOT_Init(void)
{
    if(localFlag.powerDownReq)
    {
        return false;
    }
	ModuleInit();
	return true;
}

// wait to upload all data to server, and close the opened sesion before power down
static void modulePowerDownPreHandle(void)
{
	uint32_t timeOut = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
	uint8_t sesionNum;
	uint8_t allSesion;
	// wait data queue send finish
	do
	{
		if(TxDataQueue == NULL)
		{
			break;
		}
		if(uxQueueMessagesWaiting(TxDataQueue) == 0)
		{
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}while(xTaskGetTickCount() < timeOut);
	//check all sesion state, if any sesion is connected close it
	for(sesionNum=0; sesionNum<netSesionNum; sesionNum++)
	{
		if(netSesionTbl[sesionNum].state == ipSesionOpened)
		{
			ModuleCloseSesion(sesionNum);
		}
	}
	timeOut = xTaskGetTickCount() +  pdMS_TO_TICKS(2000);
	do
	{
		allSesion = 0;
		for(sesionNum=0; sesionNum<netSesionNum; sesionNum++)
		{
			if(netSesionTbl[sesionNum].state < ipSesionOpened)
			{
				allSesion++;
			}
			if(allSesion == netSesionNum)
			{
				break;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}while(xTaskGetTickCount() < timeOut);
}

bool IOT_PowerDown(void)
{
	TickType_t timeOut = xTaskGetTickCount() +  pdMS_TO_TICKS(3000);
	localFlag.powerDownReq = 1;
	modulePowerDownPreHandle();
	//send power down event
	ModulePowerDown();
	timeOut = xTaskGetTickCount() +  pdMS_TO_TICKS(5000);
	do
	{
		if(moduleState == StatePowerOff)
		{
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}while(xTaskGetTickCount() < timeOut);
	return true;
}

bool IOT_PowerUp(void)
{
	localFlag.powerDownReq = 0;
	ModulePowerUp();
	return true;
}

#if 0
bool IOT_IpOpenBlock(uint8_t channel, uint32_t waitTime)
{
	TickType_t timeOut = xTaskGetTickCount() +  pdMS_TO_TICKS(waitTime);
	if(moduleState != StateAttached || channel >= netSesionNum)
	{
		return false;
	}
	if(netSesionTbl[channel].state == ipSesionInactive)
	{
		ModuleIpOpen(channel);
		netSesionTbl[channel].state = ipSesionOpening;
	}
	while(timeOut >= xTaskGetTickCount())
	{
		if(netSesionTbl[channel].state == ipSesionOpened)
		{
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	return true;
}
#endif
bool IOT_IpOpenUnblock(uint8_t channel)
{
	if(moduleState != StateAttached|| channel >= netSesionNum)
	{
		return false;
	}  
	ModuleIpOpen(channel);
	netSesionTbl[channel].state = ipSesionOpening;
	return true;
}

bool IOT_IpCloseBlock(uint8_t channle, uint32_t waitTIme)
{
	return true;
}

bool IOT_IpClose(uint8_t channel)
{
	if(channel >= netSesionNum)
	{
		return false;
	}
	ModuleCloseSesion(channel);
	netSesionTbl[channel].state = ipSesionClosing;
	return true;
}

bool IOT_Restart(void)
{
	ModuleRestart();
	return true;
}

//when module attached to network, return ture
bool IOT_isModuleReady(void)
{
	if(moduleState == StateAttached)
	{
		return true;
	}
	return false;
}

bool IOT_SelfTeste(void)
{
	ModuleSelfTeste();
	return true;
}

netSesionState_t IOT_GetSessionState(uint8_t num)
{
	if(num >= netSesionNum)
	{
		return ipSesionInactive;
	}
	return netSesionTbl[num].state;
}

bool IOT_GetImei(uint8_t *pBuffer)
{
	if(pBuffer == NULL)
	{
		return false;
	}
	memcpy(pBuffer, &moduleInfo.imei, HEX_IMEI_LEN);
	return true;
}

bool IOT_GetImsi(uint8_t *pBuffer)
{
	if(pBuffer == NULL)
	{
		return false;
	}
	memcpy(pBuffer, &moduleInfo.imsi, HEX_IMSI_LEN);
	return true;
}

bool IOT_GetIccid(uint8_t *pBuffer)
{
	if(pBuffer == NULL)
	{
		return false;
	}
	memcpy(pBuffer, &moduleInfo.iccid, HEX_ICCID_LEN);
	return true;    
}

uint8_t IOT_GetCsq(void)
{
	return localFlag.lastCsq;
}

uint8_t IOT_IsIpTransOk(void)
{
	return localFlag.isIpTransOk;
}

uint8_t IOT_IsSimReady(void)
{
	return localFlag.isSimReady;
}

uint8_t IOT_ModuleSleep(void)
{
	return true;
}

void IOT_receiveDatahanlerRegester(ipRxDataHanle handle)
{
	ipRxDataHandler = handle;
}

void IOT_ipConnectStateChangeNotifierRegester(ipConnectCb notifier)
{
	ipOperateCb = notifier;
}
