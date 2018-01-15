#include "GPRS.h"
#include "debug.h"
#include "uart.h"
#include "board.h"
#include "timers.h"
#include "prj_config.h"
#include "str_lib.h"
#include <stdio.h>
/****
 *define
 *****/
/* 3 session: 0 for major session; 1 for minor session; 3 for ftp session */
#define TCP_SESION_1    0
#define TCP_SESION_2    1
#define FTP_SESION_NUM  2

#define AT_CMD_MAX_CNT 25
#define EVENT_MAX_CNT 12
#define RX_DATA_MAX_CNT 4
#define TX_DATA_MAX_CNT 4

/* common command resend interval */
#define CMD_COMMON_INTERVAL  300
/* pdp active command resend interval */
#define CMD_PDP_ACTIVE_INTERVAL 5000

#define CMD_QUERY_SIM_TIMEOUT 20000
#define CMD_QUERY_CS_TIMEOUT  90000
#define CMD_QUERY_PS_TIMEOUT 60000

/* define context id */
#define TCP_CMIOT_ID	0
/* define context number */
#define CONTEXT_NUM		1
/* define the conext status */
#define	CONTEXT_STATE_DEACTIVE (uint8_t)0
#define CONTEXT_STATE_ACTIVE	(uint8_t)1

#define CONTEXT_DISABLE			(uint8_t)0
#define CONTEXT_ENABLE			(uint8_t)1

#if 0
#define POWERKEY_PULL_UP()  (void)0
#define POWERKEY_PULL_DOWN() (void)(0) 
#define GET_MODULE_POEWR_STATE() 1
#define POWER_SOURCE_ON() (void)(0)
#define POWER_SOURCE_OFF() (void)(0)
#endif

#if 1
#define POWERKEY_PULL_UP()  IO_GSM_PWR_ON_OUT(0)	
#define POWERKEY_PULL_DOWN() IO_GSM_PWR_ON_OUT(1) 
#define GET_MODULE_POEWR_STATE() IO_GSM_STAUS()
#define POWER_SOURCE_ON() IO_4V_CTRL_OUT(1)
#define POWER_SOURCE_OFF() IO_4V_CTRL_OUT(0)
#endif
/* define the timeout value of IOT task */
#define TASK_WAIT_MAX_TIME 20
/* define the maxmum length of the module's command */
#define MAX_MODLE_CMD_LEN 128
//#define MAX_SEND_FAIL_CNT 3
/* if heartbeat overstep MAX_HEARTBEAT_CNT, reconnect */
#define MAX_HEARTBEAT_CNT 3
/* if reconnect count overstep MAX_RECONNECT_CNT, repowerup module */
#define MAX_RECONNECT_CNT 3

#define HEX_IMEI_LEN 8
#define HEX_ICCID_LEN 10
#define HEX_IMSI_LEN 8
/* define tcp/ip send data timeout value */
#define IP_SEND_TIMEOUT 60000
/*****
 *typedef
 ****/
/* function return value */
typedef enum
{
	ret_sucess,
	ret_pass,
	ret_fail,
	ret_malloc_err,
	ret_parm_err,
	ret_fatal_err,
}FnRet_t;

/* function pointer declaration*/
typedef FnRet_t (*fpCmdHandler)(uint8_t *pData); 
typedef FnRet_t (*fpEventFnx)(void *pParm);

typedef struct
{
	uint8_t imei[HEX_IMEI_LEN];
	uint8_t imsi[HEX_IMSI_LEN];
	uint8_t iccid[HEX_ICCID_LEN];
}moduleInfo_t;

typedef struct
{
    uint8_t *pUsr;
    uint8_t *pPwr;
    uint8_t *pUrc;
}ftpInfo_t;

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
    RecoverSence,
	EvtCnt,
}EventIdx_t;

/* URC code */
typedef enum
{
	UrcCREG = 0,
	UrcCGREG,
	UrcCMTI,
	UrcCMT,
	UrcCRING,
	UrcRDY,
	UrcCFUN,
	UrcCPIN,
	UrcQIND,
	UrcPOWDN,
	UrcCGEV,
	UrcCEREG,
	UrcQIURC,
	UrcQFTPCLOSE,
	UrcQFTPOPEN,

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

/* context table */
typedef struct
{
	struct
	{
		uint8_t contextId : 4; //0-15 map to  1-16
		uint8_t contextState : 1; //0 Deactivated; 1 Activated;
		uint8_t isContexEnable : 1;
	};
	uint8_t ip[4];
	uint8_t *APN;
}context_t;
/* net sesion's information */
typedef struct
{
	uint8_t *pHost;
	uint8_t *type;
	uint16_t port;
	netSesionState_t state;
	bool isActive;
	context_t *context;
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
	StateNetRegest,     //module is registed to network
	StateGprsRegest,    //module is registed to  network
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
	//uint8_t cmdType;
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
static FnRet_t moduleRecoverSesoinHandle(void *pParm);

/* URC handler */
static FnRet_t UrcCreghandler(uint8_t *pData);
static FnRet_t UrcCgregHandler(uint8_t *pData);
static FnRet_t UrcCmtiHandler(uint8_t *pData);
static FnRet_t UrcCmtHandler(uint8_t *pData);
static FnRet_t UrcCringHandler(uint8_t *pData);
static FnRet_t UrcRdyHanler(uint8_t *pData);
static FnRet_t UrcCfunHandler(uint8_t *pData);
static FnRet_t UrcCpinHandler(uint8_t *pData);
static FnRet_t UrcQindHandler(uint8_t *pData);
static FnRet_t UrcPowerDownHander(uint8_t *pData);
static FnRet_t UrcCgevHandler(uint8_t *pData);
static FnRet_t UrcCeregHandler(uint8_t *pData);
static FnRet_t UrcQiurcHandler(uint8_t *pData);
static FnRet_t UrcQftpOpenHandler(uint8_t *pData);
static FnRet_t UrcQftpCloseHandler(uint8_t *pData);
//static FnRet_t UrcReceivedHandler(uint8_t *pData);
//static FnRet_t UrcPdpDeactHandler(uint8_t *pData);
//static FnRet_t UrcConnectedHandler(uint8_t *pData);
//static FnRet_t UrcDisConnectedHandler(uint8_t *pData);
//static FnRet_t UrcSendFailHandler(uint8_t *pData);


/* AT command package function */
static FnRet_t ipQuerySendState(uint8_t channel);
static FnRet_t ipOpen(uint8_t channel);
static FnRet_t gotoSleep(void);
static FnRet_t test(void);
static FnRet_t deactiveGprs(void);
static FnRet_t ipClose(uint8_t sesion);
static FnRet_t csqCheck(void);
static FnRet_t netInit(void);
static FnRet_t reconnectSesion(uint8_t num);
static FnRet_t activateContext(uint8_t num);
/***
 ** Local variable
 **/
static context_t context[CONTEXT_NUM] =
{
    {
        .contextId = 1, // context id 1 map to TCP context
        .contextState = CONTEXT_STATE_DEACTIVE, // context id state
        .isContexEnable = CONTEXT_ENABLE,
        .APN = (uint8_t *)"CMIOT"
    },
};
static  netSessionInfo_t netSesionTbl[] = 
{
	{
		.pHost = "101.37.87.65",
		//.pHost = "59.44.43.234",
		.port = 30032,
		.type = (uint8_t *)"TCP",
		.state = ipSesionClose,
        .isActive = true,
		.context = &context[TCP_CMIOT_ID],
	},
	{
		.pHost = "101.37.87.65",
		.port = 30032,
		.type = (uint8_t *)"TCP",
		.state = ipSesionClose,
        .isActive = false,
		.context = &context[TCP_CMIOT_ID],

	},
	{
		.pHost = "101.37.87.65",
		.port = 22,
		.type = (uint8_t *)"FTP",
		.state = ipSesionClose,
        .isActive = false,
		.context = &context[TCP_CMIOT_ID],
	}
};
static ftpInfo_t ftpInfo = 
{
    .pUsr = "derenftp",
    .pPwr = "deren@123",
    .pUrc = "\\"
};
static uint8_t moduleState;
static moduleInfo_t moduleInfo;
static const uint8_t netSesionNum = sizeof(netSesionTbl) / sizeof(netSessionInfo_t);

static TaskHandle_t TaskHandle;

static QueueHandle_t EventQueue;
static QueueHandle_t AtCmdQueue;
static QueueHandle_t TxDataQueue;
/* each session has it own's queue */
static QueueHandle_t RxDataQueue[sizeof(netSesionTbl) / sizeof(netSessionInfo_t)];
/* a timer check the csq peridic */
static TimerHandle_t csqCheckTimer;
/* used to detect transmit timeout */
static TickType_t ipTransTimer;
//SMS buffer
//static QueueHandle_t MesageQueue;
//FTP buffer
//static QueueHandle_t RxFtpQueue;

/* used to detect command excute timeout */
static TickType_t cmdExecuteTime;
/* express AT command try count */
static uint16_t cmdTryCnt;

/* command buffer */
static uint8_t cmdBuff[MAX_MODLE_CMD_LEN];
static uint16_t cmdBuffIdx;

/* Store the data received temporary */
static RxData_t RxTmp;
/* Express the number of bytes have received */
static uint16_t ipRxDataIdx;
/* express the connection's number that is receiving data  */
static uint8_t RxingSesion;
/* A pointer point to a function that will execute when connection state changed */
static ipConnectCb ipOperateCb;
/* A pointer point to a function that will execute when receive data */
ipRxDataHanle ipRxDataHandler;
/* Internal heartbeat, if count over maximum value, reconnect.  */
static uint8_t interHeartBeat;
struct _flag_strut
{
	uint8_t isTransmitterBusy : 1;
	uint8_t isNetReceving : 1;
	uint8_t isMsgReceving : 1;
	uint8_t isFtpReceving : 1;
	uint8_t powerDownReq : 1;
//	uint8_t sleepReq : 1;
//	uint8_t isMulti : 1;
	uint8_t isEcho : 1;
	uint8_t isSimReady : 1;
	uint8_t isIpTransOk : 1;
//	uint8_t ipSesionKillReq : 1;
	uint8_t cmdState : 1; 
	uint8_t lastCsq : 5;
};
static struct _flag_strut localFlag;
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
    moduleRecoverSesoinHandle
};

//static uint8_t ownIp[4] = "";
static DecodeTbl_t urcTable[UrcNum] = 
{
	{"+CREG: ", UrcCreghandler},
	{"+CGREG: ", UrcCgregHandler},
	{"+CMTI: ", UrcCmtiHandler},
	{"+CMT: ", UrcCmtHandler},
	{"+CRING: ", UrcCringHandler},
	{"+RDY: ", UrcRdyHanler},
	{"+CFUN: ", UrcCfunHandler},
	{"+CPIN: ", UrcCpinHandler},
	{"+QIND: ", UrcQindHandler},
	{"POWERED DOWN", UrcPowerDownHander},
	{"+CGEV: ", UrcCgevHandler},
	{"+CEREG: ", UrcCeregHandler},
	{"+QIURC: ", UrcQiurcHandler},
	{"+QFTPOPEN: ",UrcQftpOpenHandler},
	{"+QFTPCLOSE: ", UrcQftpCloseHandler}
};

/***
 ** Local function
 **/

/**************************Interface****************************/
static void interfaceInit(void)
{
	Uart_Initialize(UART_GSM_CHANNEL);
	/* Communication interface initialize  */
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
    uint8_t idx;
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
    TxDataQueue = xQueueCreate(TX_DATA_MAX_CNT, sizeof(TxData_t));
    if(TxDataQueue == NULL)
    {
		for(;;)
		{
			DEBUG(DEBUG_HIGH,  "[IOT] ERROR: Create TxData queue failed!\r\n");
		}        
    }
    for(idx=0; idx<netSesionNum; idx++)
    {
        RxDataQueue[idx] = xQueueCreate(RX_DATA_MAX_CNT, sizeof(RxData_t));
        if(RxDataQueue[idx] == NULL)
        {
            DEBUG(DEBUG_HIGH,  "[IOT] ERROR: Create RxData queue failed!\r\n");
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
    atCmd_tbl_t command;
    while(xQueueReceive(AtCmdQueue,&command,0) == pdPASS)
	{
		if(command.pCommand)
		{
			vPortFree(command.pCommand);
			command.pCommand = NULL;
		}

	}
	return true;
}

static void cleanTxDataQueue(void)
{
	TxData_t data;
	while(xQueueReceive(TxDataQueue,&data,pdMS_TO_TICKS(100)) == pdPASS)
	{
        if(xQueueReceive(TxDataQueue,&data,pdMS_TO_TICKS(100)) != pdPASS)
        {
            return;
        }
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
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Allocate memory error. Line:%d!\r\n", __LINE__);
		return ;
	}
	memcpy(event.pData, &channel, sizeof(uint8_t));
	if(pdPASS != xQueueSend(EventQueue, &event, 0))
	{
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add OpenIpEvt event failed!\r\n");
		return ;
	}
	//linkDetecter[channel].reconnectCnt++;
	netSesionTbl[channel].state = ipSesionOpening;
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
	netSesionTbl[channel].state = ipSesionClosing;
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

static void ModuleRecoverSesion(void)
{
	Event_t event;
	event.event = RecoverSence;  
    event.pData = NULL;
    if(pdPASS != xQueueSend(EventQueue, &event, pdMS_TO_TICKS(100)))
    {
        DEBUG(DEBUG_HIGH, "[IOT] ERROR: Add GprsTestEvt event failed!\r\n");
        return ;
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
	if(moduleState < StatePowerOn || localFlag.cmdState == 1)
	{
		return ;
	}
	atCmd_tbl_t command;
	if(pdPASS == xQueuePeek(AtCmdQueue, &command, 0))
	{
        //if command is IP send command,verify the data's length
        if(!memcmp(command.pCommand, "AT+QISEND=", 10))
        {
            uint8_t channel = command.pCommand[strlen("AT+QISEND=")] - '0';
            uint16_t len = atoi((char *)command.pCommand+12);
            TxData_t data;
            if(pdPASS == xQueuePeek(TxDataQueue, &data, 0))
            {
                if(len != data.dataLen)
                {
                    if(pdPASS == xQueueReceive(TxDataQueue, &data, 0))
                    {
                        if(data.cb)
                        {
                            data.cb(false, channel);
                        }
                        if(data.pData)
                        {
                            vPortFree(data.pData);
                            data.pData = NULL;
                        }
                        if(pdPASS == xQueueReceive(AtCmdQueue,&command, 0))
                        {
                            if(command.pCommand)
                            {
                                vPortFree(command.pCommand);
                                command.pCommand = NULL;
                            }
                        }
                    }
                    else
                    {
                        DEBUG(DEBUG_HIGH, "[IOT] error when delete the at queue item\r\n");
                    }
                    return;
                }
            }
        }
        if(ret_sucess != portSend(command.pCommand, command.len))
        {
            DEBUG(DEBUG_HIGH, "[IOT] ERROR: Port send data failed\r\n");
            return ;
        }
        DEBUG(DEBUG_LOW,"[IOT] send CMD: %s\r\n",command.pCommand);
        //cmdTryCnt = 1;
        cmdExecuteTime = pdMS_TO_TICKS(command.timeOut) + xTaskGetTickCount();
        localFlag.cmdState = 1;
    }
	return ;	
}

/* detect the URC */
static UrcCdoe_t urcDetecter(uint8_t *pData)
{
	uint8_t idx;
	for(idx=0; idx<UrcNum; idx++)
	{
		if(!memcmp(pData, urcTable[idx].pCode, strlen((char *)urcTable[idx].pCode)))
		{
			urcTable[idx].handler(pData);
			break;
		}
	}
	return (UrcCdoe_t)idx;
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
	else
	{
		return  ret_fail;
	}
}

/* wait for SIM card to ready */
static FnRet_t isSimCardReady(uint8_t *pResp)
{
	if(!memcmp(pResp, "OK", 2))
	{
		if(localFlag.isSimReady == 1)
		{
			return ret_sucess;
		}
	}
	DEBUG(DEBUG_HIGH,"[IOT] SIM card not ready");
	return ret_fail;
}

static FnRet_t iccidDeocder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return  ret_fail;
    }
    if(memcmp(pResp, "OK", 2))
    {
        return ret_sucess;
    }
    if(strlen((char *)&pResp[strlen("+QCCID: ")]) == HEX_ICCID_LEN*2)
    {
        uint8_t idx;
        for(idx=0; idx<HEX_ICCID_LEN; idx++)
        {
            moduleInfo.iccid[idx] = StrtoHex(&pResp[2*idx]);
        }
    }
    return ret_fail;
}

static FnRet_t imeiDeocder(uint8_t *pResp)
{
    if(pResp == NULL)
    {
        return  ret_fail;
    }
    if(memcmp(pResp, "OK", 2))
    {
        return ret_sucess;
    }
    if(strlen((char *)pResp) == HEX_IMEI_LEN * 2 - 1)
    {
        uint8_t idx;
        pResp[HEX_IMEI_LEN * 2 - 1] = 'f'; //
        for(idx=0; idx<HEX_IMEI_LEN; idx++)
        {
            moduleInfo.imei[idx] = StrtoHex(&pResp[2*idx]);
        }
        return ret_pass;
    }
    return ret_fail;
}

static FnRet_t imsiDecoder(uint8_t *pResp)
{
	if(pResp == NULL)
	{
		return  ret_parm_err;
	}
	if(memcmp(pResp, "OK", 2))
	{
		return ret_sucess;
	}
	if(strlen((char *)pResp) == HEX_IMSI_LEN * 2 - 1)
	{
		uint8_t idx;
		pResp[HEX_IMSI_LEN * 2 - 1] = 'f'; //
		for(idx=0; idx<HEX_IMSI_LEN; idx++)
		{
			moduleInfo.imsi[idx] = StrtoHex(&pResp[2*idx]);
		}
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

/* if module has registered to CS network, return sucess; */
static FnRet_t checkCreg(uint8_t *pResp)
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

/* if module has registered to PS service, return sucess; */
static FnRet_t checkPSRegest(uint8_t *pResp)
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
    else if(cmdTryCnt >= CMD_QUERY_PS_TIMEOUT/CMD_COMMON_INTERVAL)
    {
        /*when timeout, return sucess no matter register to PS domain or not  */
        return ret_sucess;
    }
	return ret_fail;
}

/* Query the status of the context profile */
static FnRet_t QueryContextOneProfile(uint8_t *pResp)
{
	uint8_t commaPos = 0;
	uint8_t contextId = 0;
	if(pResp == NULL)
	{
		return ret_parm_err;
	}
	if(!memcmp(pResp, "OK", 2))
	{
        if(context[0].contextState == CONTEXT_STATE_ACTIVE)
        {
            return ret_sucess;
        }
        return ret_fail;
	}
	if(!memcmp(pResp, "+QIACT:", strlen("+QIACT:")))
	{
        pResp+=8;
        contextId = atoi((char *)pResp);
        if(contextId > CONTEXT_NUM)
        {
            /*??? how to hanle it?*/
            DEBUG(DEBUG_HIGH, "[IOT] context id error\r\n");
            return ret_sucess;
        }
        for(;;)
        {
            if(*pResp == 0)
            {
                return ret_pass;
            }
            if(*pResp == ',')
            {
                commaPos++;
                pResp++;
            }
            else
            {
                pResp++;
                continue;
            }

            switch(commaPos)
            {
                case 0: //context ID
                    break;
                case 1: //context state
                    if(*pResp == '1')
                    {
                        context[contextId-1].contextState = CONTEXT_STATE_ACTIVE;
                    }
                    else
                    {
                        context[contextId-1].contextState = CONTEXT_STATE_DEACTIVE;
                    }
                    break;
                case 2: //context type
                    break;
                case 3: //IP address
                    {
                        uint32_t ip[4];
                        sscanf((char *)pResp, "%d.%d.%d.%d", &ip[0],
                                                        &ip[1],
                                                        &ip[2],
                                                        &ip[3]);
                        context[contextId-1].ip[0] = ip[0];
                        context[contextId-1].ip[1] = ip[1];
                        context[contextId-1].ip[2] = ip[2];
                        context[contextId-1].ip[3] = ip[3];

                    }
                    break;
                default:
                    break;
            }
        }
	}
	return ret_fail;
}

static FnRet_t ipOpenDecoder(uint8_t *pData)
{
	uint8_t HeadLen = strlen("+QIOPEN: ");
	if(!memcmp(pData, "OK", 2))
	{
		return ret_pass;
	}
	else if(!memcmp(pData,"+QIOPEN: ", HeadLen))
	{
		pData += HeadLen;
		uint8_t channel = atoi((char *)(pData));
		if(channel >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[IOT] net session error\r\n");
			return ret_fail;
		}
		while(*pData)
		{
			if(*pData == ',')
			{
                pData++;
				if(*(pData) == '0')
				{
					netSesionTbl[channel].state = ipSesionOpened;
					return ret_sucess;
				}
				else
				{
                    uint16_t errCode = atoi((char *)pData);
                    switch(errCode)
                    {
                        case 563:
                            reconnectSesion(channel);
                            //ipclose(channel);
                            break;
                        default:
                            break;
                            
                    }
					DEBUG(DEBUG_HIGH, "[IOT] Open connection error, code:%s\r\n", (pData));
					return ret_fail;
				}
			}
			pData++;
		}
	}
	return ret_fail;
}

/* Decode IP send command. wait for '>' */
static FnRet_t ipSendRespDecoder(uint8_t *pData)
{
	FnRet_t ret = ret_fail;
	TxData_t Data;
	if(pData == NULL)
	{
		return ret_fail;
	}
	memset(&Data, 0, sizeof(TxData_t));
	if(!memcmp(pData, "> ", 2))
	{
		atCmd_tbl_t command;
		uint8_t sesionNum;
        uint16_t dataLen;
		if(pdPASS == xQueuePeek(AtCmdQueue, &command, 0))
		{
			sesionNum = atoi((char *)&command.pCommand[10]);
			if(sesionNum >= netSesionNum)
			{
				DEBUG(DEBUG_HIGH, "[IOT] ERROR: session index error\r\n");
			}
			if(sesionNum >= 10)
			{
				dataLen =  atoi((char *)&command.pCommand[13]);
			}
			else
			{
				dataLen = atoi((char *)&command.pCommand[12]);
			}
            if(pdPASS == xQueuePeek(TxDataQueue, &Data, 0))
            {
                if(Data.pData == NULL)
                {
                    DEBUG(DEBUG_HIGH, "[IOT] ERROR: Receive >, but data in queue error!\r\n");
                }
                if(dataLen != Data.dataLen)
                {
                    DEBUG(DEBUG_HIGH, "[IOT] ERROR: data length error\r\n");
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
                    ret = ret_pass;
                }
			}
			else
			{
				DEBUG(DEBUG_HIGH, "[IOT] ERROR: Receive >, but have no data to sent!\r\n");
			} 
		}
	}
	else if(!memcmp(pData,"SEND OK", strlen("SEND OK")))
	{
		if(pdPASS == xQueuePeek(TxDataQueue, &Data, 0))
		{
			ipQuerySendState(Data.channel);
		}
		ret = ret_sucess;
	}
	else
	{
		DEBUG(DEBUG_LOW, "[IOT] \"> \"decode error\r\n")
	}
	return ret;
}

static FnRet_t ftpOpenDecoder(uint8_t *pData)
{
    if(pData == NULL)
    {
        return  ret_fail;
    }
    if(!memcmp(pData, "+QFTPOPEN: ", 11))
    {
        uint16_t errno = atoi((char *)pData+11);
        if(errno != 0)
        {
            DEBUG(DEBUG_HIGH, "[IOT] FTP open error:%d\r\n",errno);
            return ret_fail;
        }
        return ret_sucess;
    }
    else if(!memcmp(pData, "OK", 2))
    {
        return ret_pass;
    }
    else if(!memcmp(pData, "+CME ERROR: ", 12))
    {
        DEBUG(DEBUG_HIGH, "[IOT] FTP open error: +CME ERROR:%d\r\n", atoi((char *)pData+12));
        return ret_fail;
    }
    return ret_fail;
}

/* Decode CSQ value */
static FnRet_t csqCheckDecoder(uint8_t *pData)
{
    if(pData == NULL)
    {
        return ret_fail;
    }
    if(!memcmp(pData, "OK", 2))
    {
        return ret_sucess;
    }
    else if(!memcmp(pData, "+CSQ: ",5))
    {
        localFlag.lastCsq = atoi((char *)pData+5);
        return ret_sucess;
    }
	return ret_fail;
}

static FnRet_t querySendState(uint8_t *pResp)
{
	uint32_t len = 0, ackedbytes = 0, unackedbytes = 0;
	TxData_t data;
	if(pResp == NULL)
	{
		return ret_fail;
	}
	memset(&data, 0, sizeof(TxData_t));
	if(!memcmp(pResp, "OK", 2))
	{
		if(localFlag.isTransmitterBusy == 0)
		{
			//linkDetecter[sesionNum].heartbeat = 0;
            if(xQueueReceive(TxDataQueue, &data, pdMS_TO_TICKS(100)) == pdPASS)
            {

                if(data.cb)
                {
                    data.cb(true, data.channel);
                }
                vPortFree(data.pData);
                data.pData = NULL;
                data.cb = NULL;
            }
			return ret_sucess;
		}
		return  ret_fail;
	}
	if(!memcmp(pResp, "+QISEND:", strlen("+QISEND:")))
	{
		sscanf((char *)(pResp+strlen("+QISEND:")), "%d,%d,%d", &len, &ackedbytes, &unackedbytes);
		if(unackedbytes == 0)
		{
			localFlag.isTransmitterBusy = 0;
		}
		return ret_pass;
	}
	return ret_pass;
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
				localFlag.isNetReceving = 0;
			}
		}
		else if(localFlag.isMsgReceving)
		{

		}
		else
		{
			//Receive module response
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
				//parse URC first
				if(UrcNum != urcDetecter(cmdBuff))
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
#if 0 //how to hanle error message????
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
#endif
					else
					{
						//decode response
						if(command.decoder)
						{
							ret = command.decoder(cmdBuff);
							memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
							if(ret == ret_sucess)
							{
								cmdTryCnt = 1;
								localFlag.cmdState = 0;
								// Receive expected response
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
							else if(ret == ret_pass)
							{
								continue;
							}
							else
							{
#if 0
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
#endif
							}

						}
						else
						{
                            //should not entry here!!!!!
							memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
							// Receive expected response
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
						//localFlag.cmdState = 0;
						continue ;
					}					
				}
			}
		}
	}
	//if UART buffer has no data, check the AT command has been timeout.
	if(pdPASS == hasCmd && localFlag.cmdState == 1)
	{
#if 0
        if(command.decoder)
        {
            if(command.decoder(NULL) == ret_sucess)
            {
                // Receive expected response
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
            if(cmdTryCnt >= command.tryCnt)
            {
                cleanAtCmdQueue();
                localFlag.cmdState = 0;
                cmdTryCnt = 1;
                ModuleRestart();
                ModuleInit();
                ModuleRecoverSesion();
            }
            memset(cmdBuff, 0, MAX_MODLE_CMD_LEN);
			cmdExecuteTime = pdMS_TO_TICKS(command.timeOut) + xTaskGetTickCount();            
			localFlag.cmdState = 0;
			cmdTryCnt++;
		}
	}
}

/****************************** AT command package *****************************/
static FnRet_t ipQuerySendState(uint8_t channel)
{
	uint8_t cmd[32] = "";
	snprintf((char *)cmd, 32, "AT+QISEND=%d,0\r\n",channel);
	return cmdSend(cmd, strlen((char *)cmd),100, 100, querySendState);
}

static FnRet_t ipOpen(uint8_t channel)
{
	uint8_t ipOpenCmd[128] = "";
	uint8_t cmdLen = 0;
	if((cmdLen = snprintf((char *)ipOpenCmd, 128, "AT+QIOPEN=%d,%d,\"%s\",\"%s\",%d,0,1\r\n",
			netSesionTbl[channel].context->contextId,
			channel,
			(char *)netSesionTbl[channel].type,
			(char *)netSesionTbl[channel].pHost,
			netSesionTbl[channel].port)) >= 127)
	{
		DEBUG(DEBUG_HIGH, "[IOT] Error: When encode IP open command");
		return ret_fail;
	}
	if(cmdSend(ipOpenCmd, cmdLen, 7500, MAX_RECONNECT_CNT, ipOpenDecoder) != ret_sucess)
	{
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t gotoSleep(void)
{
	if(cmdSend("AT+WQSCLK=0\r\n", strlen("AT+QSCLK=0\r\n") ,500, 1, commonRespDecoder) != ret_sucess)
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
#if 0
	if(cmdSend("AT+CIPSHUT\r\n", strlen("AT+CIPSHUT\r\n"), 500, 1, commonRespDecoder))
	{
		return ret_fail;
	}
	return ret_sucess;
#endif
	return ret_fail;
}

static FnRet_t ipClose(uint8_t sesion)
{
	uint8_t cmd[64] = "";
	if(sesion >= netSesionNum)
	{
		DEBUG(DEBUG_HIGH, "[IOT] close session number error\r\n");
		return ret_fail;
	}
	sprintf((char *)cmd, "AT+QICLOSE=%d\r\n", sesion);
	if(cmdSend(cmd, strlen((char *)cmd), 1000, 1, commonRespDecoder))
	{
		DEBUG(DEBUG_HIGH, "[IOT] closeSesion add At command error\r\n");
		return ret_fail;
	}
	return ret_sucess;
}

static FnRet_t csqCheck(void)
{
	cmdSend("AT+CSQ\r\n",strlen("AT+CSQ\r\n"),300,1,csqCheckDecoder);
	return ret_sucess;
}

#if 0
static FnRet_t powerDownByCommand(void)
{
	cmdSend("AT+CPOWD=1\r\n",strlen("AT+CPOWD=1\r\n"),5000,1,waitPowerDownOk);
	return ret_sucess;    
}
#endif

static FnRet_t reconnectSesion(uint8_t num)
{
    FnRet_t ret;
    ret = ipClose(num);
    if(ret != ret_sucess)
    {
        DEBUG(DEBUG_HIGH, "[IOT] close failed,when reconnecting");
    }
    ret = ipOpen(num);
    return ret;
}

static FnRet_t netInit(void)
{
#define MAX_CMD_LEN 128
	uint8_t cmd[MAX_CMD_LEN] = "";
    test();
    /* disable echo */
    cmdSend("ATE0\r\n", strlen("AT0\r\n"), 500, 1, echoCloseDecoder);
    /* query SIM card status */
    cmdSend("AT+CPIN?\r\n", strlen("AT+CPIN?\r\n"),CMD_COMMON_INTERVAL,
                                                    CMD_QUERY_SIM_TIMEOUT/CMD_COMMON_INTERVAL, 
                                                     isSimCardReady);
    /* query CS service */
    cmdSend("AT+CREG?\r\n", strlen("AT+CREG?\r\n"), CMD_COMMON_INTERVAL, 
                                                    CMD_QUERY_CS_TIMEOUT/CMD_COMMON_INTERVAL, 
                                                    checkCreg);
    /* query PS service */
    cmdSend("AT+CGREG?\r\n", strlen("AT+CGREG?\r\n"), CMD_COMMON_INTERVAL, 
                                                        CMD_QUERY_PS_TIMEOUT/CMD_COMMON_INTERVAL,    
                                                        checkPSRegest);
    /* get IMEI */
    cmdSend("AT+GSN\r\n", strlen("AT+GSN\r\n"),CMD_COMMON_INTERVAL,1,imeiDeocder);
    /* get IMSI */
    cmdSend("AT+CIMI\r\n", strlen("AT+CIMI\r\n"), CMD_COMMON_INTERVAL, 1, imsiDecoder);
    /* get ICCID */
    cmdSend("AT+QCCID\r\n",strlen("AT+QCCID\r\n"),CMD_COMMON_INTERVAL,1,iccidDeocder);

//	cmdSend("AT+CGREG?\r\n", strlen("AT+CGREG?\r\n"),300, 60000/300);
    /* configure PDP context for net session 0*/
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf((char *)cmd, MAX_CMD_LEN, 
             "AT+QICSGP=%d,1,\"%s\",\"\",\"\",1\r\n",
              netSesionTbl[0].context->contextId,netSesionTbl[0].context->APN);
	cmdSend(cmd, strlen((char *)cmd),CMD_COMMON_INTERVAL,1,commonRespDecoder);
    /* active PDP context */
    memset(cmd, 0, MAX_CMD_LEN);
    snprintf((char *)cmd, MAX_CMD_LEN, "AT+QIACT=%d\r\n",netSesionTbl[0].context->contextId);
    cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    /* query PDP context status */
    switch(netSesionTbl[0].context->contextId)
    {
        case 1:
            cmdSend("AT+QIACT?\r\n", strlen("AT+QIACT?\r\n"),CMD_PDP_ACTIVE_INTERVAL, 3, QueryContextOneProfile);
            break;
        default:
            for(;;)
            {
                DEBUG(DEBUG_HIGH, "[IOT] Context ID error\r\n");
            }
    }
    return ret_sucess;
}

static FnRet_t ftpDownload(void)
{
    uint8_t cmd[MAX_CMD_LEN] = "";
    /* configure the ftp PDP context */
    if(netSesionTbl[FTP_SESION_NUM].context->contextState == CONTEXT_STATE_DEACTIVE)
    {
        memset(cmd, 0, MAX_CMD_LEN);
        snprintf((char *)cmd, MAX_CMD_LEN, "AT+QICSGP=%d,1,\"%s\",\"\",\"\",1\r\n",netSesionTbl[FTP_SESION_NUM].context->contextId,netSesionTbl[FTP_SESION_NUM].context->APN);
        cmdSend(cmd, strlen((char *)cmd),CMD_COMMON_INTERVAL,1,commonRespDecoder);     
        /* active PDP context */
        memset(cmd, 0, MAX_CMD_LEN);
        snprintf((char *)cmd, MAX_CMD_LEN, "AT+QIACT=%d\r\n",netSesionTbl[0].context->contextId);
        cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
        /* query PDP context status */
        switch(netSesionTbl[0].context->contextId)  
        {
            case 1:
                cmdSend("AT+QIACT?\r\n", strlen("AT+QIACT?\r\n"),CMD_PDP_ACTIVE_INTERVAL, 3, QueryContextOneProfile);
                break;
            default:
                for(;;)
                {
                    DEBUG(DEBUG_HIGH, "[IOT] Context ID error\r\n");
                }
        }
    }
    snprintf((char *)cmd, MAX_CMD_LEN, "AT+QFTPCFG=\"contextid\",%d\r\n",netSesionTbl[0].context->contextId);
    cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    snprintf((char *)cmd, MAX_CMD_LEN, "AT+QFTPCFG=\"account\",\"%s\",\"%s\"\r\n",ftpInfo.pUsr,ftpInfo.pPwr);
    cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    cmdSend("AT+QFTPCFG=\"transmode\",1\r\n", strlen("AT+QFTPCFG=\"transmode\",1\r\n"), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    cmdSend("AT+QFTPCFG=\"rsptimeout\",90\r\n", strlen("AT+QFTPCFG=\"rsptimeout\",90\r\n"), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    
    //cmdSend("AT+QFTPCFG=\"ssltype\",1", strlen("AT+QFTPCFG=\"ssltype\",1"),CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    //cmdSend("AT+QFTPCFG=\"sslctxid\",1", strlen("AT+QFTPCFG=\"sslctxid\",1"),CMD_COMMON_INTERVAL, 1, commonRespDecoder);   
    //cmdSend("AT+QSSLCFG=\"ciphersuite\",1, 0xffff", strlen("AT+QSSLCFG=\"ciphersuite\",1, 0xffff"),CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    //cmdSend("AT+QSSLCFG=\"seclevel\",1,0",strlen("AT+QSSLCFG=\"seclevel\",1,0"),CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    ///cmdSend("AT+QSSLCFG=\"sslversion\",1,1", strlen("AT+QSSLCFG=\"sslversion\",1,1"), CMD_COMMON_INTERVAL, 1, commonRespDecoder);
    snprintf((char *)cmd, MAX_CMD_LEN, "AT+QFTPOPEN=\"%s\",%d\r\n", netSesionTbl[FTP_SESION_NUM].pHost,netSesionTbl[FTP_SESION_NUM].port);
    cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, ftpOpenDecoder);
    return ret_sucess;
}

static FnRet_t activateContext(uint8_t num)
{
    uint8_t cmd[MAX_CMD_LEN] = "";
	snprintf((char *)cmd, MAX_CMD_LEN, 
             "AT+QICSGP=%d,1,\"%s\",\"\",\"\",1\r\n",
              netSesionTbl[0].context->contextId,netSesionTbl[0].context->APN);
	cmdSend(cmd, strlen((char *)cmd),CMD_COMMON_INTERVAL,1,commonRespDecoder);
    /* active PDP context */
	memset(cmd, 0, MAX_CMD_LEN);
	snprintf((char *)cmd, MAX_CMD_LEN, "AT+QIACT=%d\r\n",netSesionTbl[0].context->contextId);
	cmdSend(cmd, strlen((char *)cmd), CMD_COMMON_INTERVAL, 1, commonRespDecoder); 
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
	uint8_t cmd[128] = "";
	TxData_t data;
	if(ipTransTimer <= xTaskGetTickCount())
	{
		localFlag.isTransmitterBusy = 0;
	}
	if(localFlag.isTransmitterBusy)
	{
		return;
	}
	if(pdPASS == xQueuePeek(TxDataQueue, &data, 0))
	{

		//sprintf((char *)cmd, "AT+CIPSEND=%d,%d\r\n",data.channel, data.dataLen);
		snprintf((char *)cmd, 128, "AT+QISEND=%d,%d\r\n",data.channel, data.dataLen);
	}
	else
	{
		return ;
	}
#if 0
	if(linkDetecter[data.channel].heartbeat >= MAX_HEARTBEAT_CNT)
	{
		//check current session state
		ModuleCloseSesion(data.channel);
		return ;
	}
#endif
	if(!netSesionTbl[data.channel].isActive)
	{
		if(xQueueReceive(TxDataQueue, &data, pdMS_TO_TICKS(100)) != pdPASS)
		{
			DEBUG(DEBUG_HIGH, "[IOT] Delete TxDataQueue item error\r\n");
		}
		else
		{
				if(data.pData)
				{
					vPortFree(data.pData);
					data.pData = NULL;
				}
				if(data.cb)
				{
					data.cb(false, data.channel);
				}
		}
		return ;

	}
	if(netSesionTbl[data.channel].state == ipSesionOpened)
	{
		if(cmdSend(cmd, strlen((char *)cmd) ,IP_SEND_TIMEOUT, 1, ipSendRespDecoder) != ret_sucess)
		{ 
			DEBUG(DEBUG_HIGH,"[IOT] Error: When send data!\r\n");
			return ;
		}
		localFlag.isTransmitterBusy = 1;
		//linkDetecter[data.channel].heartbeat++;
		interHeartBeat++;
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
	ipOpen(sid);
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
    RxData_t data;
	if(sesionNum >= netSesionNum)
	{
		DEBUG(DEBUG_HIGH, "[IOT] sesion num error\r\n");
	}


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

	return ret_fail;
}

static FnRet_t moduleRecoverSesoinHandle(void *pParm)
{
	uint8_t sesionIdx;
	for(sesionIdx=0; sesionIdx<netSesionNum; sesionIdx++)
	{
		if(netSesionTbl[sesionIdx].isActive)
		{
            if(netSesionTbl[sesionIdx].context->contextState == CONTEXT_STATE_DEACTIVE)
            {
                //if the context bindinged to current session is not active,then activate it
                activateContext(sesionIdx);
            }
            if(netSesionTbl[sesionIdx].state <= ipSesionOpened)
            {
                ModuleIpOpen(sesionIdx);
            }
		}
	}    
    return ret_sucess;
}

static FnRet_t moduleCloseSesionHandle(void *pParm)
{
	ipClose(*(uint8_t *)pParm);
	return ret_sucess;
}

/* URC handle */

static FnRet_t UrcFatalHandler(uint8_t *pData)
{
    if(moduleState <= StatePowerOn)
    {
        cleanAtCmdQueue();
        moduleState = StateRecover;
        ModuleRestart();
        localFlag.cmdState = 0;
        ModuleInit();
        ModuleRecoverSesion(); 
    }
	return ret_sucess;
}

static FnRet_t UrcConnectedHandler(uint8_t *pData)
{
	uint8_t sesionNum = 0;
	//linkDetecter[sesionNum].reconnectCnt = 0;
	netSesionTbl[sesionNum].state = ipSesionOpened;
    localFlag.isTransmitterBusy = 0;
	if(ipOperateCb)
	{
		ipOperateCb(true, sesionNum);
	}
	return ret_sucess;
}
#if 0
static FnRet_t UrcDisConnectedHandler(uint8_t *pData)
{
	uint8_t sesionNum = 0;
	//cleanTxDataQueue();
#if 0
	//reconnect
	if(moduleState != StateRecover)
	{
		if(linkDetecter[sesionNum].reconnectCnt >= MAX_RECONNECT_CNT)
		{
#if 0
			uint8_t idx=0;

			for(idx=0; idx<netSesionNum; idx++)
			{
				linkDetecter[idx].reconnectCnt = 0;
				linkDetecter[idx].heartbeat = 0;
			}
#endif
			memset(linkDetecter, 0, sizeof(linkDetecter));
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
    if(ipOperateCb)
    {
        ipOperateCb(false, sesionNum);
    }
#endif
	return ret_sucess;
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
#endif
static FnRet_t UrcCreghandler(uint8_t *pData)
{
	if(pData[9] == '1' || pData[9] == '5')
	{
		moduleState = StateNetRegest;
	}
	else
	{
        /* do nothing */
	}
	return ret_sucess;
}

static FnRet_t UrcCgregHandler(uint8_t *pData)
{
	if(pData[10] == '1' || pData[10] == '5')
	{
		moduleState = StateGprsRegest;
	}
	else
	{
        /* do nothing */
	}
	return ret_sucess;
}

static FnRet_t UrcCmtiHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcCmtHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcCringHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcRdyHanler(uint8_t *pData)
{
    moduleState = StateReady;
	return ret_fail;
}
static FnRet_t UrcCfunHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcCpinHandler(uint8_t *pData)
{
	//uint8_t headerLen = strlen("+CPIN: ");
	if(!memcmp(&pData[7],"READY", strlen("READY")))
	{
		localFlag.isSimReady = 1;
		return ret_sucess;
	}
	localFlag.isSimReady = 0;
	return ret_fail;
}
static FnRet_t UrcQindHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcPowerDownHander(uint8_t *pData)
{
    if(localFlag.powerDownReq)
    {
        return ret_sucess;
    }
    else
    {
        moduleState = StatePowerOff;
        ModulePowerUp();
        ModuleInit();
        ModuleRecoverSesion();
    }
	return ret_fail;
}
static FnRet_t UrcCgevHandler(uint8_t *pData)
{
    if(!memcmp("REJECT",pData+7, 6))
    {
        
    }
    else if(!memcmp("NW REACT", pData+7, 8))
    {
        
    }
    else if(!memcmp("ME DEACT", pData+7, 8))
    {
    
    }
    else if(!memcmp("NW DETACH", pData+7, 9))
    {
        
    }
    else if(!memcmp("ME DETACH", pData+7, 9))
    {
    
    }
    else if(!memcmp("NW CLASS", pData+7, 8))
    {
    
    }
    else if(!memcmp("ME CLASS", pData+7, 8))
    {
        
    }
	return ret_fail;
}

static FnRet_t UrcCeregHandler(uint8_t *pData)
{
	return ret_fail;
}

static FnRet_t UrcQiurcHandler(uint8_t *pData)
{
	uint8_t headlen = 0;
	if(pData == NULL)
	{
		return  ret_fail;
	}
	headlen = strlen("+QIURC: ");
	if(!memcmp(pData+headlen, "\"closed\"", 8))
	{
		uint8_t connectID = atoi((char *)(pData+headlen+9));
		if(connectID >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[IOT] connection error\r\n");
		}
        else
        {
            netSesionTbl[connectID].state = ipSesionClose;
            ModuleRecoverSesion();
        }
	}
	else if(!memcmp(pData+headlen, "\"recv\"", 6))
	{
		uint8_t commaPos = 0;
		pData += (headlen + 6);
		while(*pData != 0)
		{
			if(*pData == ',')
			{
				commaPos++;
				switch(commaPos)
				{
					case 1:
						RxingSesion = atoi((char *)(pData+1));
						if(RxingSesion >= netSesionNum)
						{
							RxingSesion = 0;
							DEBUG(DEBUG_HIGH, "[IOT] connection ID error\r\n");
							return ret_sucess;
						}
						break;
					case 2:
						RxTmp.dataLen = atoi((char *)(pData+1));
						if(RxTmp.dataLen == 0)
						{
							DEBUG(DEBUG_HIGH, "[IOT] Rx data length error\r\n");
							return ret_sucess;
						}
						//localFlag.isNetReceving = 1;
						break;
					case 3:
						//do nothing
						break;
					case 4:
						//do nothing
						break;
					default:
						break;
				}
			}
			pData++;
		}
		if(commaPos < 2)
		{
			//send AT+QIRD to read the data in memory
		}
		else
		{
			localFlag.isNetReceving = 1;
			RxTmp.pData = pvPortMalloc(RxTmp.dataLen);
			if(RxTmp.pData == NULL)
			{
				DEBUG(DEBUG_HIGH, "[IOT] Malloc error\r\n");
				return ret_sucess;
			}
			ipRxDataIdx = 0;
		}
	}
	else if(!memcmp(pData+headlen, "\"pdpdeact\"", 10))
	{
		uint8_t connectID = atoi((char *)(pData+headlen+11));
		if(connectID >= netSesionNum)
		{
			DEBUG(DEBUG_HIGH, "[IOT] connection ID error\r\n");
			return ret_sucess;
		}
		//reactive the connection

	}
	return ret_sucess;;
}
static FnRet_t UrcQftpOpenHandler(uint8_t *pData)
{
	return ret_fail;
}
static FnRet_t UrcQftpCloseHandler(uint8_t *pData)
{
	return ret_fail;
}

void IOT_Task(void *pParm)
{
	interfaceInit();
	handlerInit();
	ModulePowerDown();
	ModulePowerUp();
	//ModuleSelfTeste();
	ModuleInit();
	ModuleIpOpen(0);
	//ModuleIpOpen(1);
	for(;;)
	{
		Event_t event; 
		if(xQueueReceive(EventQueue, &event, pdMS_TO_TICKS(TASK_WAIT_MAX_TIME)) == pdPASS)
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
		DEBUG(DEBUG_HIGH, "[IOT] ERROR: parameter error:IOT_IpNetSend\r\n");
		return  false;
	}
	if(IOT_GetSessionState(channel) == ipSesionOpened && !localFlag.powerDownReq)
	{
		TxData_t data;
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
			DEBUG(DEBUG_HIGH, "[IOT] ERROR: When Add Data to tx data queue!\r\n");
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
		if(uxQueueMessagesWaiting(TxDataQueue) == 0)
		{
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}while(xTaskGetTickCount() < timeOut);
	//if there has any data still store in data queue, clean it and notify APP
	//cleanTxDataQueue();
	//check all session state, if any session is connected close it
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
				return;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}while(xTaskGetTickCount() < timeOut);
    cleanTxDataQueue();
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
	netSesionTbl[channel].isActive = true;
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
	netSesionTbl[channel].isActive = false;
	return true;
}

bool IOT_IpReconnect(uint8_t channel)
{
	if(channel >= netSesionNum)
	{
		return false;
	}
	ModuleCloseSesion(channel);
	netSesionTbl[channel].isActive = true;
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
		return ipSesionClose;
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
