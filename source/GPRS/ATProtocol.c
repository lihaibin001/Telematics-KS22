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
/*
 * ATProtocol.c
 * AT command handling and UART driver
 */
/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include <standard.h>
#include <stdio.h>
#include "ATProtocol.h"
#include "ATApp.h"
#include "GPRS.h"
#include "uart.h"

#include "spi_flash.h"
#include "TelmProtocol.h"
#include "gps.h"
#include "str_lib.h"

//#define USE_DEBUG
#include "Debug.h"

/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
#define LF			(0x0A)   	//LF char ascii   new line
#define CR			(0x0D)   	//CR char ascii   carriage return
#define	CHAR_QUOTES	(0x22)		//'\"'
#define CHAR_COMMA	(0x2C)		//','
#define CHAR_O		(0x4F)		//'O'
#define CHAR_K		(0x4B)		//'K'
#define CHAR_PLUS	(0x2B)		//'+'
#define CHAR_COLON	(0x3A)  	//':'
#define CHAR_SMCLN	(0x3B)		//';'
#define CHAR_SPACE	(0x20)		//' '
#define CHAR_GREATER    (0x3E)          //'>'
#define CHAR_STOP    (0x2E)          //'.'
#define CHAR_CTRLZ   (0x1A) //ctrl-Z
#define CHAR_ESC     (0x1B)  //ESC

#define TCOM_RX_BUF_SIZE	(1500)
//#define TCOM_RX_BUF_SIZE	(300)
#define TCOM_TX_BUF_SIZE	TCP_TX_MAX_LEN
#define TCOM_RX_BUF_PACKET_NUM	(5)

#define TCP_RX_PARAM_BUFF_SIZE (5)
#define SMS_RX_PARAM_BUFF_SIZE (5)

#define AT_LINK_NUM			(1)		//max link num in command 'IPOPEN,IPSEND'
#define ERROR_RETRY_MAX (20)
#define SEND_RETRY_MAX (10)

#define DEFAULT_APN "cmnet"
#define DEFAULT_DOMAIN_NAME "obu1.deren.com"

#define SMS_RECEIVE_LIMIT (20)
#define MAX_CONNECT_ID (2)

#define GB_DOMAIN_NAME "218.205.176.44"
#define GB_PORT "19006"

//#define TEST_ACK_TIMER
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef enum AT_Resp_Rx_State_tag
{
    RX_IDLE,
    START_RX_PREFIX,
    START_RX_DATA,
    START_RX_POSTFIX,
    RX_STATE_NUM
}AT_Resp_Rx_State_t;

typedef enum AT_Cmd_Tx_State_tag
{
    TX_IDLE,                /* Waiting for message to transmit */
    TX_CS_RETRANSMIT,     /* Waiting to retransmit data frame */
    TX_CS_WAIT_FOR_RESP,   /* Waiting for response*/
    TX_CS_NUM_STATES,
}AT_Cmd_Tx_State_t;

typedef enum AT_Cmd_Type_tag
{
    INVALID = 0,
    BASIC = 1,
    QUERY = 2,
    CHECK = 4,
    EXCUTE = 8,
    FATAL = 16, //fatal command must be send ok, if failed must reset module
}AT_Cmd_Type_t;

typedef enum TCP_Data_Mode_tag
{
    TCP_IDLE = 0,
    TCP_TXING, //Wait for ">"
    TCP_TX_POSTDATA,
    TCP_RXING, //Wait for RX length data.
}TCP_Data_Mode_t;

typedef enum SMS_Data_Mode_tag
{
    SMS_IDLE = 0,
    SMS_RXING, //Wait for data
}SMS_Data_Mode_t;

typedef void (*URC_Resp_Decode_fptr) (char* rx_buffer, uint8_t rx_count);          // encode function
typedef void (*AT_Resp_Decode_fptr) (void);			// encode function

typedef struct GSM_Info_tag
{
    bool	netActive;
    uint8_t rssi; //0~30 is value. 31 is no signal
    uint8_t srv_status; //0: no service, 1 is starting to send data, 2 is sending data, 3 is receiving data
    char swver[32]; 
    char ccid[20];
    char imsi[15];
    char imei[15];
    uint8_t tcp_state[2]; //network state
    uint8_t module_go_sleep;
    bool isModuleOk;
}GSM_Info;

typedef struct AT_Cmd_Info_tag
{
	int8_t AT_Retry_Max;
	char AT_Cmd_Name[16];
	uint32_t AT_Cmd_Type;//AT_Cmd_Type_t
	Tick_Type	timeout;
	bool Supported;
}AT_Cmd_Info_t;

typedef struct AT_Resp_data_tag
{
	uint8_t Cmd_index;
	char	Resp_IT[100];
	uint8_t length;
}AT_Resp_data_t;

/*
typedef struct AT_Rx_Resp_data_tag
{
	char response_char[MAX_RESPONSE_CHAR_LENGTH];
	uint8_t length; // length of  response char
}AT_Rx_Resp_data_t;
*/

typedef struct AT_URC_decode_tag
{
	const	uint8_t	URC_Index;
	const	char	URC[10];
	const	URC_Resp_Decode_fptr decode_routine;
	URC_Notify_fptr		 notify_urc;
}AT_URC_decode_t;

typedef struct AT_Resp_decode_tag
{
	uint8_t	AT_Res_Index;
	AT_Resp_Decode_fptr decode_routine;
}AT_Resp_decode_t;

// AT+QIRD patameters, not used
typedef struct AT_TCP_Rx_Param_tag
{
    uint8_t id;
    uint8_t sc;
    uint8_t sid;
}AT_TCP_Rx_Param_t;

// clock data
typedef struct AT_Clock_tag
{
    uint8_t year[4];
    uint8_t month[2];
    uint8_t day[2];
    uint8_t hour[2];
    uint8_t min[2];
    uint8_t sec[2];
}AT_Clock_t;


/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
/* initialization */

/* recieve process */
static void prvATProt_Rx_Resp_Assemble(uint8_t data);
static int16_t prvATProt_Uart_Get_Char (uint8_t*data);

/* AT recieve decode entrance */
static void prvATProt_Resp_Common_Decode(char* tcom_rx_buffer, uint8_t rx_count);
/* AT URC Notify decode */
static bool prvATProt_URC_Decode		(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_nop	(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_CGreg	(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Signal	(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Recv(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Gprs(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Cmt(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Sms(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Sim(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Sms_Got(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Netreg(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Pdp(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Temp(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Cme_Err(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_SMS_List(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_SMS_Recv(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Net_Recv(char* rx_buffer, uint8_t rx_count);
//static void prvATProt_URC_TCP_Recv(char* rx_buffer, uint8_t rx_count);
static void prvATProt_URC_decode_Clock(char* rx_buffer, uint8_t rx_count);

//static void prvATProt_parse_sms(char* rx_buffer, uint8_t rx_count);

/* AT response decode */
static void prvATProt_Resp_Deep_Decode			(void);
static void prvATProt_Resp_Decode_SW_Ver(void);
static void prvATProt_Resp_Decode_IP_INIT_Execute	(void);
static void prvATProt_Resp_Decode_IP_REG_Execute	(void);
static void prvATProt_Resp_Decode_SET_Echo(void);
static void prvATProt_Resp_Decode_Answer_Call(void);
static void prvATProt_Resp_Decode_CCID_Query(void);
static void prvATProt_Resp_Decode_IMEI_Query(void);
static void prvATProt_Resp_Decode_IP_Query(void);
static void prvATProt_Resp_Decode_QIClose(void);
/* transmit sequence management */
static void prvATProt_Transmit(uint8_t AT_Cmd_index);
static void prvATProt_Retransmit(void);
static void prvATProt_Wait_Resp(void);
static void prvATProt_AT_Transmit_Reset(void);
static void prvATProt_AT_Var_Reset(void);
uint8_t prvATProt_Uart_Transmit(const char* tx_buf, uint16_t bytes);

/* AT command encode */
static void prvATProt_AT_Command_Common_encode	(uint8_t AT_Cmd_index);
static void prvATProt_AT_Command_Deep_encode	(uint8_t AT_Cmd_index);
static void prvATProt_AT_Command_Deep_encode_SendSMS(void);
static void prvATProt_AT_Command_Deep_encode_SendPostdata(void);
static void prvATProt_AT_Command_Deep_encode_Init	(void);
static void prvATProt_AT_Command_Deep_encode_Open	(void);
static void prvATProt_AT_Command_Deep_encode_Send	(void);
/* 2017.6.22 lihaibin modify for ntp*/
static void prvATProt_AT_Command_Deep_encode_SAPBR_phase_1(void);
static void prvATProt_AT_Command_Deep_encode_SAPBR_phase_2(void);
static void prvATProt_AT_Command_Deep_encode_CNTP_phase_1(void);
static void prvATProt_AT_Command_Deep_encode_QICLOSE(void);
/* end 2017.6.22 lihaibin modify */
static void prvATProt_AT_Command_Deep_encode_Sendack	(void);
static void prvATProt_AT_Command_Deep_encode_Multiip	(void);
static void prvATProt_AT_Command_Deep_encode_Cfun	(void);

static void prvATProt_AT_Command_Deep_encode_Rxget      (void);

static void prvATProt_AT_Command_Deep_encode_QIOPEN	(void);
static void prvATProt_AT_Command_Deep_encode_QISEND	(void);
static void prvATProt_AT_Command_Deep_encode_QIRD	(void);
static void prvATProt_AT_Command_Deep_encode_QIREGAPP	(void);
static void prvATProt_AT_Command_Deep_encode_Delete_SMS	(void);
static void prvATProt_AT_Command_Deep_encode_ReadSMS	(void);
/* AT Command queue management */
static void prvATProt_AT_Send_Set(AT_cmd_t send_cmd_index);

//static void prvATProt_Set_Recv_Len(uint16_t len);
static void prvATProt_Set_TCP_Mode(uint8_t mode);
static uint8_t prvATProt_Get_TCP_Mode(void);

static uint8_t prvATProt_Decode_Ipdata(char *data, uint16_t len);
#define USE_DOMAIN_NAME
#ifndef USE_DOMAIN_NAME
static void get_server_ip(void);
#endif
//static void get_server_port(void);


/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
AT_Cmd_Param_t	 	        AT_Cmd_Param;
static uint16_t				tcom_tx_num_bytes;
static GSM_Info			    net_Info;		
static AT_Cmd_Tx_State_t	AT_Cmd_Tx_State;
static AT_Resp_Rx_State_t	AT_Resp_Rx_State;
static char					tcom_tx_buffer[TCOM_TX_BUF_SIZE];
static char 				tcom_rx_buffer[TCOM_RX_BUF_SIZE];
static AT_Resp_data_t		AT_Resp_data;
static char 				IPDATA[TCOM_RX_BUF_SIZE];
static AT_cmd_t			    AT_pending_tx_cmd;
static Tick_Type			AT_Resp_Timeout;
static AT_Cmd_Res			at_cmd_res;
static bool					tcom_receiving_frame;
static uint8_t				tcom_tx_queue[(AT_CMD_NUM+7)/8];
static uint8_t				tcom_frame_tx_attempts;
static char 				Net_Rx_Data[TCOM_RX_BUF_PACKET_NUM][TCOM_RX_BUF_SIZE];
static uint16_t 			Net_Rx_Data_Size[TCOM_RX_BUF_PACKET_NUM];
static uint8_t              Net_Rx_Read_Index=0;
static uint8_t              Net_Rx_Data_Num=0;
static uint8_t              current_sms_index=0;
static uint8_t              module_power_on=0;

//static uint8_t rx_pending_status = 0;

#ifdef TCOM_SUPPORT_SMS
static SMS_glb_info SMS_back_data;
#endif

TCP_data_t TCP_backup_data;
//static uint8_t tcp_mode = TCP_IDLE;
static uint16_t  tcp_rx_len = 0;
static uint8_t tcp_rx_comma_pos=0;
//static Tick_Type tcp_mode_Timeout_timer;//MSec_To_Ticks(5000); //need check.
static uint16_t  rx_char_count = 0;//Modify to file static variable.
static uint8_t   tcp_tx_mode = 0; //0:data, 1:ack

//static SMS_Data_Mode_t sms_rx_mode = SMS_IDLE; //0:not reading; 1:reading
static AT_Clock_t clock_ref;

static uint8_t ack_from_server = 0;
static uint8_t error_count = 0;

static uint16_t  IPDataLen = 0;
static Tick_Type Response_Timeout_timer = MSec_To_Ticks(6000);
static uint8_t   AT_Tx_ATTEMPS = 6;

static uint8_t total_sms_num=0;

#ifdef TCOM_SUPPORT_SMS
#define SMS_SMSC_NO_START   (0)
#define SMS_SMSC_NO_LEN     (18)
#define SMS_MTI_MR_LEN      (4)

#define SMS_RCV_NO_START    (22)
#define SMS_RCV_NO_LEN      (18)

#define SMS_PID_START       (40)
#define SMS_PID_LEN         (6)

#define SMS_DATA_LEN_START  (46)
#define SMS_DATA_BODY_START (48)
#endif

/* telmatics info encode function type */
typedef void (*AT_Cmd_Encode_Fun)(void);

// SIMCOM AT Commands or QUECTEL Commands
static const AT_Cmd_Info_t AT_Cmd_Info_table[AT_CMD_NUM] =
{
    /*AT_Retry_Max,AT_Cmd_Name,          Cmd_Type, timeout,      Supported*/
    { 0,  "None",           INVALID,        0,                      false}, /* AT_CMD_NONE */
    { 10, "ATE0",           EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_NO_ECHO_SETTING */
    { 20, "AT+CLIP=",       EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SET_CLIP */
    { 20, "AT",             EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SET_ECHO */
    { 20, "AT+GMR",         QUERY,          MSec_To_Ticks(1000),    true},   /* AT_CMD_GET_SW_VER */

    { 20, "AT+CCID",        EXCUTE,         MSec_To_Ticks(1000),    true},   /* AT_CMD_GET_CCID_NO */
    { 20, "AT+GSN",         EXCUTE,         MSec_To_Ticks(1000),    true},   /* AT_CMD_GET_IMEI_NO */
    { 20, "AT+CGREG=",      EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SET_CGREG */
    { 20, "AT+CREG=2",      EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SET_CREG */
    { 5,  "",               EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_HANG_UP */

    { 5,  "ATD",            EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_DIAL */
    { 5,  "AT+CMGS=",       EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SEND_SMS */
    { 1,  "",               EXCUTE,         MSec_To_Ticks(60000),   true},  /* AT_CMD_SEND_POST_DATA */
    { 5,  "AT+CGACT=",      EXCUTE,         MSec_To_Ticks(2000),    true},  /* AT_CMD_ACTIVE_PDP_CONTEXT */
    { 1,  "AT+CGATT=1",     EXCUTE,         MSec_To_Ticks(10000),   true},  /* AT_CMD_CGATT */

    
    /* 2017.6.23 lihaibin modify for ntp */
    { 3,  "AT+SAPBR=",      EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SAPBR_PHASE_1 */
    { 3,  "AT+SAPBR=",      EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SAPBR_PHASE_2 */
    { 3,  "AT+SAPBR=1,1",   EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SAPBR_PHASE_3 */
    { 3,  "AT+CNTPCID=1",   EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_CNTPCID */
    { 3,  "AT+CNTP=",       EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_CNTP_PHASE_1 */
    { 3,  "AT+CNTP",        EXCUTE,         MSec_To_Ticks(10000),    true},  /* AT_CMD_CNTP_PHASE_1 */    
    { 3,  "AT+CCLK?",       QUERY,          MSec_To_Ticks(1000),     true},  /* AT_CMD_CCLK */
    /* end 2017.6.23 lihaibin add */    
    
    { 5,  "ATH",            EXCUTE,         MSec_To_Ticks(2000),    true},  /* AT_CMD_ATH */
    { 1,  "AT+CSQ",         BASIC,          MSec_To_Ticks(2000),    true},  /* AT_CMD_CSQ_QUERY */
    { 2,  "AT+CSTT=",       EXCUTE,         MSec_To_Ticks(30000),   true},  /* AT_CMD_IP_INIT_EXCUTE */
    { 3,  "AT+CSTT=?",      CHECK,          MSec_To_Ticks(2000),    true},  /* AT_CMD_IP_INIT_CHECK */
    { 1,  "AT+CIPSTART=",   EXCUTE,         MSec_To_Ticks(50000),   true},  /* AT_CMD_IP_OPEN_EXCUTE */

    { 3,  "AT+CIPSHUT",     EXCUTE|FATAL,   MSec_To_Ticks(2000),    true},  /* AT_CMD_IPCLOSE_EXCUTE */
    { 2,  "AT+CIPRXGET=",   EXCUTE,         MSec_To_Ticks(5000),    true},  /* AT_CMD_IP_RX_MANUALLY_QUERY */
    { 1,  "AT+QIOPEN=",     EXCUTE,         MSec_To_Ticks(75000),   true},  /* AT_CMD_QIOPEN */
    { 1,  "AT+QISEND=",     EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_QISEND */
    { 1,  "AT+CIPCLOSE=",   EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_QICLOSE */

    { 1,  "AT+QINDI=1",     EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_QINDI */
    { 1,  "AT+QIRD=",       EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_QIRD */
    { 1,  "AT+QIHEAD=1",    EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_IPD_HEAD_SET */
    { 1,  "AT+QIREGAPP=",   EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_QIREGAPP */
    { 1,  "AT+CMGD=",       EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_DELETE_SMS */

    { 1,  "AT+CMGF=1",      EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_SMS_FORMAT */
    { 1,  "AT+CNMI=2,2",    EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_NEW_SMS_SETTING */
    { 1,  "AT+CTZU=3",      EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_SYNC_RTC */
    { 3,  "AT+CGACT?",      QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_CGACT_QUERY */
    { 3,  "AT+CSTT?",       QUERY,          MSec_To_Ticks(5000),    true},  /* AT_CMD_IP_INIT_QUERY */
    
    { 3,  "AT+CIPSTATUS",   QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_IP_OPEN_QUERY */
    { 1,  "AT+CIPSEND=",    EXCUTE|FATAL,   MSec_To_Ticks(16000),   true},  /* AT_CMD_IP_SEND_ACK_EXCUTE */
    { 3,  "AT+CPIN?",       QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_DET_SIM_QUERY */
    { 1,  "AT+CIICR",       EXCUTE,         MSec_To_Ticks(5000),    true},  /* AT_CMD_IP_REG_EXCUTE */
    { 3,  "AT+CIFSR",       QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_LOCAL_IP_ADDR */

    { 3,  "AT+CIPSEND=",    EXCUTE|FATAL,   MSec_To_Ticks(16000),   true},  /* AT_CMD_IP_SEND_DATA_EXCUTE */
    { 3,  "AT+CIPMUX=",     EXCUTE,         MSec_To_Ticks(2000),    true},  /* AT_CMD_IP_CONFIG_CONN_EXCUTE */
    { 3,  "AT+CFUN=",       EXCUTE,         MSec_To_Ticks(2000),    true},  /* AT_CMD_FUNCTION */
    { 3,  "AT+CGATT?",      QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_CHECK_GPRS */
    { 3,  "AT+CMGR=",       EXCUTE,         MSec_To_Ticks(2000),    true},  /* AT_CMD_READ_SMS_QUERY */

    { 3,  "AT+CREG?",       QUERY,          MSec_To_Ticks(2000),    true},  /* AT_CMD_CREG_QUERY */
    { 3,  "AT+CIFSR",       QUERY,          MSec_To_Ticks(1000),    true},  /* AT_CMD_IP_QUERY */
    { 3,  "AT+CSCLK=1",     EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_SLEEP */
    { 3,  "AT+CSCLK?",      QUERY,          MSec_To_Ticks(1000),    true},  /* AT_CMD_SLEEP_QUERY */
    { 3,  "AT+CPOWD=0",     EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_POWER_DOWN */

    { 3,  "ATA",            EXCUTE,         MSec_To_Ticks(1000),    true},  /* AT_CMD_ANSWER_CALL */
    { 3,  "AT+CMTE?",       QUERY,          MSec_To_Ticks(1000),    true},  /* AT_CMD_TEMP_READ */

    { 1,  "AT+CIPHEAD=1",   EXCUTE,         MSec_To_Ticks(300),     true},  /* AT_CMD_CIPHEAD */

};

static const AT_Cmd_Encode_Fun ATProt_encode_table[] =
{
    NULL,									/* AT_CMD_NONE					*/
    NULL,	/* AT_CMD_ECHO_SETTING			*/
    NULL,	/* AT_CMD_SET_CLIP				*/
    NULL,  /* AT_CMD_SET_ECHO      */
    NULL, /* AT_CMD_GET_SW_VER */

    NULL,                         /* AT_CMD_GET_IMSI_NO           */
    NULL,									/* AT_CMD_GET_IMEI_NO */
    NULL,									/* AT_CMD_SET_CGREG				*/
    NULL,									/* AT_CMD_SET_CREG				*/
    NULL,									/* AT_CMD_HANG_UP				*/

    NULL,	/* AT_CMD_DIAL					*/
    prvATProt_AT_Command_Deep_encode_SendSMS, /* AT_CMD_SEND_SMS          */
    prvATProt_AT_Command_Deep_encode_SendPostdata, /* AT_CMD_SEND_POST_DATA          */
    NULL,	/* AT_CMD_ACTIVE_PDP_CONTEXT	*/
    NULL,									/* AT_CMD_CGATT				*/

    /* 2017.6.23 lihaibin modify */
    prvATProt_AT_Command_Deep_encode_SAPBR_phase_1,
    prvATProt_AT_Command_Deep_encode_SAPBR_phase_2,
    NULL,
    NULL,
    prvATProt_AT_Command_Deep_encode_CNTP_phase_1,
    NULL,
    NULL,    /* AT_CMD_CCLK */    
    /* end 2017.6.23 lihaibin modify */      
    
    NULL,                         /* AT_CMD_ATH */
    NULL,									/* AT_CMD_CSQ_QUERY				*/
    prvATProt_AT_Command_Deep_encode_Init,	/* AT_CMD_IP_INIT_EXCUTE		*/
    NULL,									/* AT_CMD_IP_INIT_CHECK			*/
    prvATProt_AT_Command_Deep_encode_Open,	/* AT_CMD_IP_OPEN_EXCUTE		*/

    NULL,    /* AT_CMD_IP_CLOSE_EXCUTE */
    prvATProt_AT_Command_Deep_encode_Rxget,    /* AT_CMD_IP_RX_MANUALLY_QUERY */
    prvATProt_AT_Command_Deep_encode_QIOPEN, /* AT_CMD_QIOPEN */
    prvATProt_AT_Command_Deep_encode_QISEND,/* AT_CMD_QISEND */
    prvATProt_AT_Command_Deep_encode_QICLOSE, /* AT_CMD_QICLOSE */

    NULL, /* AT_CMD_QINDI */
    prvATProt_AT_Command_Deep_encode_QIRD,/* AT_CMD_QIRD */
    NULL, /*AT_CMD_IPD_HEAD_SET*/
    prvATProt_AT_Command_Deep_encode_QIREGAPP,/* AT_CMD_QIREGAPP */
    prvATProt_AT_Command_Deep_encode_Delete_SMS,/* AT_CMD_DELETE_SMS */

    NULL,									/* AT_CMD_SMS_FORMAT			*/
    NULL,    /* AT_CMD_NEW_SMS_SETTING */
    NULL,    /* AT_CMD_SYNC_RTC */
    NULL,									/* AT_CMD_CGACT_QUERY			*/
    NULL,									/* AT_CMD_IP_INIT_QUERY			*/

    
    
    NULL,									/* AT_CMD_IP_OPEN_QUERY			*/
    prvATProt_AT_Command_Deep_encode_Sendack,    /* AT_CMD_IP_SEND_ACK_EXCUTE */
    NULL,    /* AT_CMD_DET_SIM_QUERY */
    NULL,    /* AT_CMD_IP_INIT_EXCUTE */
    NULL,    /* AT_CMD_LOCAL_IP_ADDR_QUERY */

    prvATProt_AT_Command_Deep_encode_Send,    /* AT_CMD_IP_SEND_DATA_EXCUTE */
    prvATProt_AT_Command_Deep_encode_Multiip,    /* AT_CMD_IP_CONFIG_CONN_EXCUTE */
    prvATProt_AT_Command_Deep_encode_Cfun,    /* AT_CMD_FUNCTION */
    NULL,    /* AT_CMD_CHECK_GPRS */
    prvATProt_AT_Command_Deep_encode_ReadSMS,    /* AT_CMD_READ_SMS_QUERY */

    NULL,    /* AT_CMD_CREG_QUERY */
    NULL,    /* AT_CMD_IP_QUERY */
    NULL,    /* AT_CMD_SLEEP */
    NULL,    /* AT_CMD_SLEEP */
    NULL,    /* AT_CMD_POWER_DOWN */

    NULL,    /* AT_CMD_ANSWER_CALL */
    NULL,    /* AT_CMD_TEMP_READ */

    NULL,    /* AT_CMD_CIPHEAD */
    

};

static AT_URC_decode_t AT_URC_decode_table[AT_URC_NUM] = 
{
   //
   {AT_URC_NONE, "^NONE", prvATProt_URC_decode_nop, NULL},
   {AT_URC_CGREG, "+CGREG:", prvATProt_URC_decode_CGreg, NULL},
   {AT_URC_SIGNAL, "+CSQ:", prvATProt_URC_decode_Signal, NULL}, //2017.9.20 lihaibin modify
   {AT_URC_RECV, "+RECEIVE", prvATProt_URC_decode_Recv, NULL},
   {AT_URC_CGATT, "+CGATT:", prvATProt_URC_decode_Gprs,	NULL},
   {AT_URC_CMT, "+CMT:", prvATProt_URC_decode_Cmt, NULL},
   {AT_URC_CMGR, "+CMGR:", prvATProt_URC_decode_Sms, NULL},
   {AT_URC_CPIN, "+CPIN:", prvATProt_URC_decode_Sim, NULL},
   {AT_URC_CMTI, "+CMTI:", prvATProt_URC_decode_Sms_Got, NULL},
   {AT_URC_CREG, "+CREG:", prvATProt_URC_decode_Netreg, NULL},
   {AT_URC_PDP, "+PDP:", prvATProt_URC_decode_Pdp, NULL},
   {AT_URC_CMTE, "+CMTE:", prvATProt_URC_decode_Temp, NULL},
   {AT_URC_CME_ERR, "+CME", prvATProt_URC_decode_Cme_Err, NULL},
   {AT_URC_CMGL, "+CMGL:", prvATProt_URC_decode_SMS_List, NULL},
   {AT_URC_SMS_RECV, "+CIPRXGET:", prvATProt_URC_decode_SMS_Recv, NULL},
   {AT_URC_RX_RECV, "+QIRDI:", prvATProt_URC_decode_Net_Recv, NULL},
   {AT_URC_CCLK, "+CCLK:", prvATProt_URC_decode_Clock, NULL},
//   {AT_URC_CNTP, "+CNTP:", prvATProt_URC_decode_CNTP, NULL},
};

static const AT_Resp_decode_t AT_Resp_decode_table[] = 
{
    {AT_CMD_GET_SW_VER,     prvATProt_Resp_Decode_SW_Ver},  
    {AT_CMD_IP_INIT_EXCUTE,	prvATProt_Resp_Decode_IP_INIT_Execute},
    {AT_CMD_QIREGAPP,	    prvATProt_Resp_Decode_IP_REG_Execute},
    {AT_CMD_SET_ECHO,       prvATProt_Resp_Decode_SET_Echo},
    {AT_CMD_ANSWER_CALL,    prvATProt_Resp_Decode_Answer_Call},
    {AT_CMD_GET_CCID_NO,    prvATProt_Resp_Decode_CCID_Query},
    {AT_CMD_GET_IMEI_NO,    prvATProt_Resp_Decode_IMEI_Query},
    {AT_CMD_IP_QUERY,       prvATProt_Resp_Decode_IP_Query},
    {AT_CMD_QICLOSE,        prvATProt_Resp_Decode_QIClose},
};

static const char* CME_ERROR_String = "+CME ERROR:";
static const char* CME_NW_OPENED_String = "ALREADY CONNECT";
static const char* OK_String = "OK";
static const char* CONNECT_OK_String = "CONNECT OK";
static const char* CONNECT_FAIL_String = "CONNECT FAIL";
static const char* SHUT_OK_String = "SHUT OK";
static const char* REMOTE_CLOSE_String = "CLOSED";
static const char* SEND_OK_String = "SEND OK";
static const char* READY_String = "READY";
static const char* DEACT_String = "+PDP: DEACT";
static const char* Callready_String = "Call Ready";
static const char* POWERDOWN_String = "NORMAL POWER DOWN";
static const char* RECEIVE_String = "+RECEIVE";
static const char* CNTPOK_String = "+CNTP: 1";
#ifdef TCOM_ANSWER_CALL
static const char* RING_String = "RING";
#endif

static const char* ERROR_String = "ERROR";

//For day calibration day
//2000,2004,2008,2012,2016,2020,2024,2028,2032,2036,2040,2044,2048,2052,2056,2060,
//2064,2068,2072,2076,2080,2084,2088,2092,2096,2104,2108,2112,2116,2120,2124,2128,
//2132,2136,2140,2144,2148,2152,2156,2160,2164,2168,2172,2176,2180,2184,2188,2192,
//2196,2204

#define TCOM_SEC_GATE_VALUE 30   //30second gate value


/*******************************************************************************
*    Function:  vATProt_Init
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when start up.
*******************************************************************************/
extern void vATProt_Init(void)
{
    prvATProt_AT_Var_Reset();

    net_Info.module_go_sleep = 0;
    Uart_Initialize(UART_GSM_CHANNEL);
    net_Info.srv_status = TCP_IDLE;
    net_Info.tcp_state[0] = NET_INIT;
    net_Info.tcp_state[1] = NET_INIT;
}

/* Pull down PWRKEY to power on module */
extern void vATProt_Power_On(void)
{
    IO_4V_CTRL_OUT(Bit_SET);
    IO_GSM_PWR_ON_OUT(Bit_RESET);
    rl_delay_without_schedule(600);
    IO_GSM_PWR_ON_OUT(Bit_SET); //LOW for 1.1s, this value is used for SIM800C
    rl_delay_without_schedule(1100);
    IO_GSM_PWR_ON_OUT(Bit_RESET);
//    rl_delay_without_schedule(1000);
    GPRS_SetChannelState(AT_CONNECT_ID_MAIN,SERVER_STATE_OFFLINE );
    GPRS_SetChannelState(AT_CONNECT_ID_GB,SERVER_STATE_OFFLINE );
    module_power_on=1;
}

extern void vATProt_Power_Off(void)
{
    IO_GSM_PWR_ON_OUT(Bit_SET); //LOW for 1.2s, this value is used for SIM800C
    rl_delay_without_schedule(1200);
    IO_GSM_PWR_ON_OUT(Bit_RESET);
    rl_delay_without_schedule(2100); // wait for logout net
    IO_4V_CTRL_OUT(Bit_RESET);
    module_power_on=0;
}

extern uint8_t ATProt_Power_Status(void)
{
    return module_power_on;
}

/*******************************************************************************
*    Function:  vATProt_GoSleep
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when going to sleep.
*******************************************************************************/
extern void vATProt_GoSleep(void)
{
    AT_Cmd_Param.fun_type = 0;
    net_Info.module_go_sleep = 1;

    IO_GSM_PWR_ON_OUT(Bit_SET); //LOW for 0.8S
    rl_delay_without_schedule(800);
    IO_GSM_PWR_ON_OUT(Bit_RESET);
    rl_delay_without_schedule(3000); // wait for logout net
    IO_4V_CTRL_OUT(Bit_RESET);
}

/*******************************************************************************
*    Function:  vATProt_WakeUp
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when wake up.
*******************************************************************************/
extern void vATProt_WakeUp(void)
{
    Uart_Initialize(UART_GSM_CHANNEL);
}
/*******************************************************************************
*    Function:  vATProt_Com_Reset
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when start up.
*******************************************************************************/
extern void vATProt_Com_Reset(void)
{
    vTelmApp_AT_Var_Reset();
    prvATProt_AT_Var_Reset();
}
/*******************************************************************************
*    Function:  vATProt_Upgrade_Reset
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when start up.
*******************************************************************************/
extern void vATProt_Upgrade_Reset(void)
{
}

/*******************************************************************************
*    Function:  prvATProt_Check_Transmit
*
*  Parameters:  None
*     Returns:  None
* Description:  check if data exists in rx ring buf. if true, send the data to Telematics module.
*******************************************************************************/
extern void vATProt_Check_Transmit(void)
{
    int Pending_AT_Cmd;

    //Add TCP mode guard check.
/*    if ((TCP_RXING == prvATProt_Get_TCP_Mode())
        &&(OS_Time() > tcp_mode_Timeout_timer))
    {
        prvATProt_Set_TCP_Mode(TCP_IDLE);
        AT_Resp_Rx_State = RX_IDLE; //IP RX message timer out, restart rx process
        tcom_receiving_frame = false;          
    }
    //end

    if(TCP_RXING == prvATProt_Get_TCP_Mode()) return;
*/

    /*get a new AT commond to transmit if there isn't AT command is processing*/
    if (AT_pending_tx_cmd > AT_CMD_NUM)
    {
        prvATProt_AT_Transmit_Reset();

    }

    if(AT_pending_tx_cmd == AT_CMD_NONE)
    {
        Pending_AT_Cmd = Find_First_Set_Bit(tcom_tx_queue, AT_CMD_NUM, true);
        if(TCP_TX_POSTDATA == prvATProt_Get_TCP_Mode() && (AT_CMD_SEND_POST_DATA != Pending_AT_Cmd))
        {
            DEBUG(DEBUG_HIGH,"[2G] IPSEND,Receive >,but no data is send!\n\r");
            AT_pending_tx_cmd = AT_CMD_SEND_POST_DATA;
        }
        else if ((Pending_AT_Cmd > AT_CMD_NONE) && (Pending_AT_Cmd < AT_CMD_NUM))
        {
            AT_pending_tx_cmd = (AT_cmd_t)Pending_AT_Cmd;
        }
        else
        {
            NOP();
        }
    }
    /*process the AT command*/
    if ((AT_pending_tx_cmd > AT_CMD_NONE) && (AT_pending_tx_cmd < AT_CMD_NUM))
    {
        switch (AT_Cmd_Tx_State)
        {
            case TX_IDLE:
                //lihaibin modify we should not send command when receiving
                 //if(!tcom_receiving_frame)
                 {
                    prvATProt_Transmit(AT_pending_tx_cmd);
                 }
                 break;
            case TX_CS_RETRANSMIT:
                //lihaibin modify we should not send command when receiving
                //if(!tcom_receiving_frame)
                {
                    prvATProt_Retransmit();
                }
                 break;
            case TX_CS_WAIT_FOR_RESP:
                 prvATProt_Wait_Resp();
                 break;
            default:
                 AT_Cmd_Tx_State = TX_IDLE;
                 break;
        }
    }
}

/*******************************************************************************
*    Function:  vATProt_Check_Receive
*
*  Parameters:  None
*     Returns:  None
* Description:  check if data exists in rx ring buf. if true, send the data to Telematics module.
*******************************************************************************/
extern void vATProt_Check_Receive(void)
{
    uint8_t data_byte;
    /* Test UART buffer to see if data received */

    while((-1 != prvATProt_Uart_Get_Char(&data_byte)))
    {
        //Feed_Dog();

//        DEBUG(DEBUG_MEDIUM,"%c",data_byte);
        prvATProt_Rx_Resp_Assemble(data_byte);
    }
}
/*********************************************************
*Function Name: 	bATApp_isNetWorkActive
*Prototype: 		bATApp_isNetWorkActive()
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		return if network is active
**********************************************************/
bool bATProt_isNetWorkActive(void)
{
    return net_Info.netActive;
}

/*********************************************************
*Function Name: 	pcATProt_getImsiData
*Prototype: 		pcATProt_getImsiData()
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		return the pointer to IMSI string
**********************************************************/
char const * pcATProt_getImsiData(void)
{
#ifdef TCOM_CHECK_CARINFO_STANDALONE
    return "461234561231234";//Set as 461234561231234 for test.
#else
    return net_Info.imsi;
#endif
}

char const * pcATProt_getImeiData(void)
{
    return net_Info.imei;
}

uint8_t ATProt_getRSSI(void)
{
    return net_Info.rssi;
}

/*********************************************************
*Function Name: 	ucATProt_getRecivedData
*Prototype: 		ucATProt_getRecivedData()
*Called by: 		app
*Parameters:		uint8_t*:length of recieved data
*Returns:			char * const: buffer to store recieved data
*Description:		return recieved data
**********************************************************/
char const* pcATProt_getRecivedData(uint8_t *len)
{
    *len = IPDataLen;
    return IPDATA;
}

/*********************************************************
*Function Name: 	vATProt_Add_URC_Listener
*Prototype: 		vATProt_Add_URC_Listener()
*Called by: 		app
*Parameters:		AT_URC_E		event Name
					URC_Notify_fptr	EventHandler
*Returns:			void
*Description:		set a function to Handle event 
					when it happen
**********************************************************/
void vATProt_Add_URC_Listener(AT_URC_E event, URC_Notify_fptr handler)
{
    AT_URC_decode_table[event].notify_urc = handler;
}

/*********************************************************
*Function Name: 	vATProt_Remove_URC_Listener
*Prototype: 		vATProt_Remove_URC_Listener()
*Called by: 		app
*Parameters:		AT_URC_E		event Name
*Returns:			void
*Description:		when it is no longer needed to notify
					URC event from protocol level, call
					this function to remove the listener
**********************************************************/
void vATProt_Remove_URC_Listener(AT_URC_E event)
{
    AT_URC_decode_table[event].notify_urc = NULL;
}

/*******************************************************************************
*    Function:  vATProt_sendAT_Command
*
*  Parameters:  
*     Returns:  None
* Description:  send AT command
*******************************************************************************/
void vATProt_sendAT_Command(AT_cmd_t cmd, AT_Cmd_Param_t const* data, callBack cb)
{
    // Callback for transport reset
//    AT_Cmd_Callback[cmd] = cb;
    // Handle data
    // Send command
    prvATProt_AT_Send_Set(cmd);
}

/*********************************************************
*Function Name: 	prvATProt_Rx_Buffer_Filling
*Prototype: 		prvATProt_Rx_Buffer_Filling(uint8_t data)
*Called by: 		app
*Parameters:		filling data
*Returns:		true: buffer full, false: ok
*Description:		Add one byte "data" to rx buffer, with length check.
**********************************************************/
static bool prvATProt_Rx_Buffer_Filling(uint8_t data)
{
    tcom_rx_buffer[rx_char_count++] = data;
    //Important!!!!!! RX buffer overflow check.
    //Current is overwrite the rx buffer, need double check
    if(rx_char_count >= (TCOM_RX_BUF_SIZE - 1 )) 
    {
        rx_char_count = 0; 
        return true;
        //AT_Resp_Rx_State = RX_IDLE;
    }
    return false;
}

/*********************************************************
*Function Name: 	prvATProt_Rx_Resp_Assemble
*Prototype: 		prvATProt_Rx_Resp_Assemble(uint8_t data)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		assmble the response one bytes by one bytes.
**********************************************************/
static void prvATProt_Rx_Resp_Assemble(uint8_t data)
{
    switch(AT_Resp_Rx_State)                   
    {
        case RX_IDLE:
            rx_char_count = 0;
            tcp_rx_comma_pos=0;
            tcp_rx_len=0;
            memset(tcom_rx_buffer, 0x00, TCOM_RX_BUF_SIZE);
            if(CR == data)
            {
                AT_Resp_Rx_State = START_RX_PREFIX;
            }
            else
            {
                /* not every package start with <CR><LF>,so reciving will
                still be started even if <CR><LF> doesn't exist */
                prvATProt_Rx_Buffer_Filling(data);
                AT_Resp_Rx_State = START_RX_DATA;
                tcom_receiving_frame = true;
            }
            break;
        case START_RX_PREFIX:
            if (LF == data)
            {
                AT_Resp_Rx_State = START_RX_DATA;
                tcom_receiving_frame = true;
            }
            break;
        case START_RX_DATA:
            if (TCP_RXING == prvATProt_Get_TCP_Mode())
            {
                prvATProt_Rx_Buffer_Filling(data);
                if (rx_char_count < (tcp_rx_len+tcp_rx_comma_pos+2)) 
                //if (rx_char_count < (tcp_rx_len+tcp_rx_comma_pos+1)) //2017.11.2 lihaibin modify 
                {
                    AT_Resp_Rx_State = START_RX_DATA;
                    tcom_receiving_frame = true;
                }
                else
                {
                    AT_Resp_Rx_State = RX_IDLE;
                    tcom_receiving_frame = false;
                    at_cmd_res = AT_CMD_RES_OK;	//set ok flag
                    //at_cmd_end = true;
                    //TelmApp_Set_Server_Ack_State(true);
                    prvATProt_Decode_Ipdata(tcom_rx_buffer+tcp_rx_comma_pos+2, tcp_rx_len);
                    prvATProt_Set_TCP_Mode(TCP_IDLE);
                }
            }
            else if ((CR == data) && tcom_receiving_frame && TCP_TXING != prvATProt_Get_TCP_Mode())
            {
                if (strncmp(tcom_rx_buffer,RECEIVE_String,strlen(RECEIVE_String))==0)
                {
                    uint8_t i=0;
                    for (i=0;i<15;i++)
                    {
                        if(*(tcom_rx_buffer+i)==':')
                        {
                            tcp_rx_comma_pos=i;
                            break;
                        }
                    }
                    // index 9 to ',', connection id
                    for (i=9;i<tcp_rx_comma_pos;i++)
                    {
                        if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
                            break;
                        AT_Cmd_Param.rx_select=*(tcom_rx_buffer+i)-'0';
                        tcp_rx_len = 0;
                    }
                    
                    for (i=11;i<tcp_rx_comma_pos;i++)
                    {
                        if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
                            break;
                        tcp_rx_len=tcp_rx_len*10+*(tcom_rx_buffer+i)-'0';
                        if(tcp_rx_len > 500)
                        {
                            DEBUG(DEBUG_LOW, "TCP+RX+LEN ERROR");
                        }
                    }
                    prvATProt_Set_TCP_Mode(TCP_RXING);
                }
                else
                {
                    AT_Resp_Rx_State = START_RX_POSTFIX;
                    tcom_receiving_frame = false;
                }
            }
            //2017.11.07 lihaibin modify
            else if (TCP_TXING == prvATProt_Get_TCP_Mode())
            //else
            {
                //Get GREATER and SPACE together ,else SPACE will fill to next rx buffer.
                if ((CHAR_GREATER == tcom_rx_buffer[rx_char_count -1]) 
                    && (CHAR_SPACE == data))
                {
                    AT_Resp_Rx_State = RX_IDLE;
                    tcom_receiving_frame = false;
                    prvATProt_Set_TCP_Mode(TCP_TX_POSTDATA);
                    at_cmd_res = AT_CMD_RES_OK;
                    vATProt_sendAT_Command(AT_CMD_SEND_POST_DATA, NULL, NULL);
                    if(rx_char_count > 2)
                    {
                        if (strncmp(tcom_rx_buffer,RECEIVE_String,strlen(RECEIVE_String))==0)
                        {
                            uint8_t i=0;
                            for (i=0;i<15;i++)
                            {
                                if(*(tcom_rx_buffer+i)==':')
                                {
                                    tcp_rx_comma_pos=i;
                                    break;
                                }
                            }
                            // index 9 to ',', connection id
                            for (i=9;i<tcp_rx_comma_pos;i++)
                            {
                                if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
                                    break;
                                AT_Cmd_Param.rx_select=*(tcom_rx_buffer+i)-'0';
                                tcp_rx_len = 0;
                            }
                            
                            for (i=11;i<tcp_rx_comma_pos;i++)
                            {
                                if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
                                    break;
                                tcp_rx_len=tcp_rx_len*10+*(tcom_rx_buffer+i)-'0';
                                if(tcp_rx_len > 500)
                                {
                                    DEBUG(DEBUG_LOW, "TCP+RX+LEN ERROR");
                                }
                            }
                            if (rx_char_count - 2 < (tcp_rx_len+tcp_rx_comma_pos+2)) 
                            {
                                AT_Resp_Rx_State = START_RX_DATA;
                                tcom_receiving_frame = true;
                            }
                            else
                            {
                                //AT_Resp_Rx_State = RX_IDLE;
                                
                                //at_cmd_res = AT_CMD_RES_OK;	//set ok flag
                                //at_cmd_end = true;
                                //vATApp_Sent_Callback();
                                //TelmApp_Set_Server_Ack_State(true);
                                prvATProt_Decode_Ipdata(tcom_rx_buffer+strlen(RECEIVE_String)+2, tcp_rx_len);
                                prvATProt_Set_TCP_Mode(TCP_IDLE);
                            }
                        }
        
                    }
                }
                else
                {
                    prvATProt_Rx_Buffer_Filling(data);//Receive other messages, like "error"                
                }
            }
            else
            {
                prvATProt_Rx_Buffer_Filling(data);
            }
            break;
        case START_RX_POSTFIX: 
            if (LF == data)
            {
                AT_Resp_Rx_State = RX_IDLE;
                prvATProt_Resp_Common_Decode(tcom_rx_buffer, rx_char_count);
            }
            break; 
        default:  
            AT_Resp_Rx_State = RX_IDLE;
            tcom_receiving_frame = false;
            break;
    }
}

/*******************************************************************************
* Function:  prvATProt_Resp_Common_Decode
*
* Parameters:  None
* Returns:  None
* Description:  validate rx frame data normal response or unsolicited.
*******************************************************************************/
static void prvATProt_Resp_Common_Decode(char* tcom_rx_buffer, uint8_t rx_count)
{
    uint16_t attach_data_len;
    bool at_cmd_end = false;
    uint8_t startPos = 0;
    /* check response is URC or not */
    DEBUG(DEBUG_LOW,"[GPRS RESP] %s\r\n", tcom_rx_buffer);
#if 0
    if (SMS_IDLE != sms_rx_mode)
    {
        prvATProt_parse_sms(tcom_rx_buffer, rx_count);
        sms_rx_mode = SMS_IDLE;
        return;
    }
#endif
    if(!prvATProt_URC_Decode(tcom_rx_buffer, rx_count))
    {
        
        if (AT_Cmd_Param.mux_type==1)
        {
           startPos = 3; 
        }
        
        
        /* 2017.6.27 lihaibin modify for ntp */
        /* wait for ClockSync */
        if(AT_pending_tx_cmd == AT_CMD_CNTP_2)
        {
            if (!strncmp((char*)tcom_rx_buffer, CNTPOK_String, strlen(CNTPOK_String)))
            {
                at_cmd_res = AT_CMD_RES_OK;	//set ok flag
                at_cmd_end = true;
            }
        }
        
        /* if response is NETWORK opened already for IPINIT, it's ok for network */
        else if (AT_pending_tx_cmd == AT_CMD_IP_OPEN_EXCUTE)
        {   //For IP OPEN  error recovery
            if((!strncmp((char*)(&tcom_rx_buffer[startPos]), CME_NW_OPENED_String, strlen(CME_NW_OPENED_String)))
                ||(!strncmp((char*)(&tcom_rx_buffer[startPos]), CONNECT_OK_String, strlen(CONNECT_OK_String))))
            {
                at_cmd_res = AT_CMD_RES_OK; //set ok flag
                at_cmd_end = true;
                // If connect to server 1
                net_Info.isModuleOk = true;
                if (NET_TCP_CONNECTED>vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
                {
                    vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_TCP_CONNECTED);  
                }
                // If connect to server 2
                if (NET_IP_GOT==vATProt_Get_TCP_State(AT_CONNECT_ID_GB))
                {
                    vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_TCP_CONNECTED);
                }
            }
            else if (!strncmp((char*)(&tcom_rx_buffer[startPos]), CONNECT_FAIL_String, strlen(CONNECT_FAIL_String)))
            {
                at_cmd_res = AT_CMD_RES_OK; //set ok flag,Connect failed
                at_cmd_end = true;
                //2017.9.22 lihaibin modify, when connect fail,reset the gprs module instead of connect again
                vATProt_Power_On();
                vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_POWER_ON);
                vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_POWER_ON);
                vATProt_Com_Reset();
            }   
        }
        /* if response is CME+error */
        else if(!strncmp((char*)tcom_rx_buffer, CME_ERROR_String, strlen(CME_ERROR_String)))  //error received
        {
            at_cmd_res = AT_CMD_RES_CME_ERR;	//set error flag
            at_cmd_end = true;
        }
        else if(!strncmp((char*)tcom_rx_buffer, DEACT_String, strlen(DEACT_String)))  //power down
        {
            at_cmd_res = AT_CMD_RES_OK;	//set error flag
            at_cmd_end = true;
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_GPRS_ATTACHED);
            vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
            vATProt_Com_Reset();
            prvATProt_Set_TCP_Mode(TCP_IDLE);
//            sms_rx_mode=SMS_IDLE;
            DEBUG(DEBUG_HIGH,"[2G] PDP DEACT!\n\r");
        }
        else if(!strncmp((char*)tcom_rx_buffer, POWERDOWN_String, strlen(POWERDOWN_String)))  //power down
        {
            at_cmd_res = AT_CMD_RES_OK;	//set error flag
            at_cmd_end = true;
            if (net_Info.module_go_sleep != 1)
            {
//                sms_rx_mode=SMS_IDLE;
                vATProt_Com_Reset();
                vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
                vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
                vATProt_Power_Off();
                rl_delay_without_schedule(1000);
                vATProt_Power_On();

                vATProt_Com_Reset();
                vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
            }
            prvATProt_Set_TCP_Mode(TCP_IDLE);
        }
        else if(!strncmp((char*)tcom_rx_buffer, SHUT_OK_String, strlen(SHUT_OK_String)))
        {
            at_cmd_res = AT_CMD_RES_OK;	//set ok flag
            at_cmd_end = true;
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
            vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
//            sms_rx_mode=SMS_IDLE;
            vATProt_Com_Reset();
            DEBUG(DEBUG_HIGH,"[2G] SHUT OK!\n\r");
        }
	    /* if response is error */
        else if(!strncmp((char*)tcom_rx_buffer, ERROR_String, strlen(ERROR_String)))
        {
            at_cmd_res = AT_CMD_RES_ERR;
            at_cmd_end = true;
            DEBUG(DEBUG_HIGH,"[2G] ERROR!\n\r");

            if ((AT_pending_tx_cmd == AT_CMD_IP_SEND_DATA_EXCUTE)
                ||(AT_pending_tx_cmd == AT_CMD_IP_SEND_ACK_EXCUTE))
            {
                //Drop the IP send error, we will resend the command.
                at_cmd_res = AT_CMD_RES_NONE;
            }
            else
            {
                if (error_count++ > ERROR_RETRY_MAX)
                {
                    vATProt_Com_Reset();
                    vATProt_Power_Off();
                    rl_delay_without_schedule(1000);
                    vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
                    vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
                    vATProt_Power_On();

                    vATProt_Com_Reset();
                    error_count = 0;
                    vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
                }
            }
        }

#ifdef TCOM_ANSWER_CALL
		else if(!strncmp((char*)tcom_rx_buffer, RING_String, strlen(RING_String)))
		{
			vATProt_sendAT_Command(AT_CMD_ANSWER_CALL, NULL, NULL);
		}
#endif
		else if (!strncmp((char*)(tcom_rx_buffer+startPos), SEND_OK_String, strlen(SEND_OK_String)))
		{

			at_cmd_res = AT_CMD_RES_OK;	//2017.11.2 lihaibin modify
			at_cmd_end = true;
			vATApp_Sent_Callback();
//                        DEBUG(DEBUG_MEDIUM,"SendOK\n\r");

		}
        //drop "SEND OK", not each send command has a "send ok "response.
		else if (!strncmp((char*)(tcom_rx_buffer + startPos), REMOTE_CLOSE_String, strlen(REMOTE_CLOSE_String)))
		{
			at_cmd_res = AT_CMD_RES_OK;	//set ok flag
			at_cmd_end = true;
            vATProt_Com_Reset();
            vATProt_Power_Off();
            rl_delay_without_schedule(1000);
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
            vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
            vATProt_Power_On();

            vATProt_Com_Reset();
            
            vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
            OS_Send_Message(OS_RECORD_TASK, 
                Build_Message(RECORD_EVT_RESET, 
                0));
		}
		/* if response is ok */
		else if (!strncmp((char*)tcom_rx_buffer, OK_String, strlen(OK_String)))
		{
			/*Copy ok response to rx buffer*/
			at_cmd_res = AT_CMD_RES_OK;
			at_cmd_end = true;
            if (NET_POWER_ON == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
            {
                vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_ECHO_CLOSED);
            }
		}
		else if (AT_pending_tx_cmd == AT_CMD_IP_QUERY)
		{//AT+CIFSR command has no "OK" response, so must set response to OK.
			/*Copy ok response to rx buffer*/
			at_cmd_res = AT_CMD_RES_OK;
			at_cmd_end = true;
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_IP_GOT);
		}
        else if (!strncmp((char*)tcom_rx_buffer, Callready_String, strlen(Callready_String)))
        {
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_POWER_ON);
        }
        /* if normal response */
		else
		{
		    AT_Resp_data.Cmd_index = AT_pending_tx_cmd;
            attach_data_len = min(strlen(tcom_rx_buffer), Num_Elems(AT_Resp_data.Resp_IT)-AT_Resp_data.length);
            strncpy(&AT_Resp_data.Resp_IT[AT_Resp_data.length],tcom_rx_buffer,attach_data_len);   
            AT_Resp_data.length += attach_data_len;
		}
		/* all of AT command response recieved, decode and notify */
		if (at_cmd_end == true)
		{
			AT_Resp_data.Cmd_index = (uint8_t)AT_pending_tx_cmd;
			AT_Resp_data.length = strlen(tcom_rx_buffer);
			//strncpy(AT_Resp_data.Resp_RC + strlen(AT_Resp_data.Resp_RC), tcom_rx_buffer, AT_Resp_data.length);

			prvATProt_Resp_Deep_Decode();
		}
	}
}

/*******************************************************************************
*    Function:  prvATProt_URC_Decode
*
*  Parameters:  None
*     Returns:  None
* Description:  Decode_Unsolicited_Command
*******************************************************************************/
static bool prvATProt_URC_Decode(char* rx_buffer, uint8_t rx_count)
{
	bool Is_URC = false;
	uint8_t index;
//	uint8_t URC_length;

	for( index = AT_URC_NONE + 1; index < AT_URC_NUM; index++)
	{
		if(!strncmp(tcom_rx_buffer, AT_URC_decode_table[index].URC,strlen(AT_URC_decode_table[index].URC)))
		{
			if((index > AT_URC_NONE) && (index < AT_URC_NUM))
			{
				if(NULL != AT_URC_decode_table[index].decode_routine)
				{
					AT_URC_decode_table[index].decode_routine(rx_buffer, rx_count);
				}
				if(NULL != AT_URC_decode_table[index].notify_urc)
				{
					AT_URC_decode_table[index].notify_urc();
				}
			}
			
			index = AT_URC_NUM;	//jump out the for loop
			Is_URC = true;
		}
	}
	return(Is_URC);
}

/*******************************************************************************
*    Function:  prvATProt_URC_decode_nop
*
*  Parameters:  
*     Returns:  None
* Description:  TCOM_Decode_Unsol_Nop.
*******************************************************************************/
static void prvATProt_URC_decode_nop(char* rx_buffer, uint8_t rx_count)
{
}

/*******************************************************************************
*    Function:  prvATProt_URC_decode_Creg
*
*  Parameters:  
*     Returns:  None
* Description:  TCOM_Decode_Unsol_Orig.
*******************************************************************************/
static void prvATProt_URC_decode_CGreg(char* rx_buffer, uint8_t rx_count)
{
    if ((rx_count > 8) && (rx_buffer[8] == '1'))
    {
        net_Info.netActive = true;
    }
    else
    {
        net_Info.netActive = false;
    }
}

/*******************************************************************************
*    Function:  prvATProt_URC_decode_Signal
*
*  Parameters:  
*     Returns:  None
* Description:  prvATProt_URC_decode_Signal.
*******************************************************************************/
static void prvATProt_URC_decode_Signal(char* rx_buffer, uint8_t rx_count)
{
    uint8_t dBm = 0;

    if ((rx_buffer[7] >=0x30) && (rx_buffer[7] <=0x39))
    {
        dBm = (rx_buffer[6] - '0') * 10 + rx_buffer[7] - '0';
    }
    else
    {
        dBm = rx_buffer[6] - '0';
    }

    if (dBm > 31)
    {
        net_Info.rssi = 31;
    }
    else if (dBm <= 31)
    {
        net_Info.rssi = dBm;
    }
    else
    {
        net_Info.rssi = 0;
    }
    if(net_Info.rssi != 0)
    {
        vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_SIM_CONNECTED);     
        DEBUG(DEBUG_HIGH, "[INFO] CSQ:%d\r\n", net_Info.rssi);
        //at_cmd_res = AT_CMD_RES_OK;
    }
}

#ifdef TCOM_GET_CCLK_EN
/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_CCLK_Query
*Prototype: 		prvATProt_Resp_Decode_CCLK_Query(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_Resp_Decode_CCLK_Query
**********************************************************/
static void prvATProt_decode_CCLK_Query(char* rx_buffer, uint8_t rx_count)
{
    uint8_t i = 0;
    uint8_t len = 0;

    /* find the data part(after 1 colon), the value is start from space(0x20) */
    while((rx_buffer[i++] != CHAR_COLON) &&(i < rx_count))
    {
    }
    i++;
    len = min(rx_count - i,20);
    memcpy(net_Info.cclk_time, &rx_buffer[i], len);
}
#endif

uint16_t ATProt_decode_rx(char *buffer)
{
    uint16_t len = 0;
    char *tmp_ptr = &buffer[13];
    while ((*tmp_ptr != CR) && (*tmp_ptr != CHAR_COMMA))
    {
        if ((*tmp_ptr >= '0') || (*tmp_ptr <= '9'))
        {
            len = len*10 + (*tmp_ptr - '0');
        }
        else
        {
            return 0;
        }
        tmp_ptr++;
    }
    while(*tmp_ptr != LF)
        tmp_ptr++;
    tmp_ptr++;
    return len;
}

static void prvATProt_URC_decode_Recv(char* rx_buffer, uint8_t rx_count)
{

//    uint8_t mode = 0;
    uint8_t tcp_rx_comma_pos=0;
    uint8_t i=0;
    for (i=0;i<rx_count;i++)
    {
        if(*(tcom_rx_buffer+i)==':')
        {
            tcp_rx_comma_pos=i;
            break;
        }
    }
    // index 9 to ',', connection id
    for (i=9;i<tcp_rx_comma_pos;i++)
    {
        if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
            break;
        AT_Cmd_Param.rx_select=*(tcom_rx_buffer+i)-'0';
    }
    for (i=11;i<tcp_rx_comma_pos;i++)
    {
        if ((*(tcom_rx_buffer+i)>'9') || (*(tcom_rx_buffer+i)<'0'))
            break;
        tcp_rx_len=tcp_rx_len*10+*(tcom_rx_buffer+i)-'0';
    }
    prvATProt_Set_TCP_Mode(TCP_RXING);

}

static void prvATProt_URC_decode_Gprs(char* rx_buffer, uint8_t rx_count)
{
    uint8_t state = 0;

    state = (rx_buffer[8] - '0');

    if (state == 1)
    {
        /* GPRS OK, go to attach network */
        if (NET_GPRS_ATTACHED == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_APN_REGISTERED);
        AT_Cmd_Param.gprs_ready=1;
    }
    else if (state == 0)
    {
        /* GPRS not working, loop detect */
        AT_Cmd_Param.gprs_ready=0;
    }
    else
    {
        /* Nothing to do */
    }
}

static void prvATProt_URC_decode_Cmt(char* rx_buffer, uint8_t rx_count)
{
  /* Nothing to do */
}

static void prvATProt_URC_decode_Sms(char* rx_buffer, uint8_t rx_count)
{
    /* Parse data from rx_buffer[7] */
//    sms_rx_mode=SMS_RXING;
}

static void prvATProt_URC_decode_Sim(char* rx_buffer, uint8_t rx_count)
{
    /* Parse data from rx_buffer[7] */
    if (strncmp(READY_String, &rx_buffer[7],sizeof(READY_String)-1) == 0)
    {
        /* SIM OK */
        if (NET_ECHO_CLOSED == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_SIM_CONNECTED);
        AT_Cmd_Param.sim_ready=1;
    }
    else
    {
        DEBUG(DEBUG_HIGH,"[2G]:SIM Not Ready!\n\r");
        /* SIM not OK */
        AT_Cmd_Param.sim_ready=0;
    }
}

static void prvATProt_URC_decode_Sms_Got(char* rx_buffer, uint8_t rx_count)
{
    uint8_t sms_num = 0;

    if ((rx_buffer[13]>='0') && (rx_buffer[13]<='9'))
    {
        sms_num = ((rx_buffer[12] - '0')*10)+(rx_buffer[13] - '0');
    }
    else
    {
        sms_num = (rx_buffer[12] - '0');
    }

    if (sms_num >= 1)
    {
        /* Read SMS */
        vATProt_sendAT_Command(AT_CMD_READ_SMS_QUERY, NULL, NULL);
        current_sms_index=sms_num;
        total_sms_num++;
    } 
    else
    {
        /* error */
    }
}

static void prvATProt_URC_decode_Netreg(char* rx_buffer, uint8_t rx_count)
{
    uint8_t netsts = 0;
    char *buf_ptr=rx_buffer;
    uint8_t *p_temp;

    if ((rx_buffer[9]<'0') || (rx_buffer[9]>'9'))
    {
        netsts=0;
        AT_Cmd_Param.netreg=netsts;
        return;
    }
    netsts = (rx_buffer[9] - '0');

    if ((netsts == 1) || (netsts == 5))
    {
//        if (NET_IMSI_GOT == vATProt_Get_TCP_State())
        uint8_t len=0;
        vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_REGISTERED);
        p_temp=Get_Filed_Data_Pointer((uint8_t *)buf_ptr,2);
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len>0)
        {
//            uint32_t lac=StrtoHex(p_temp+1,len-2);
            AT_Cmd_Param.lac[0]=StrtoHex(p_temp+1);
            AT_Cmd_Param.lac[1]=StrtoHex(p_temp+3);
        }
        p_temp=Get_Filed_Data_Pointer((uint8_t *)buf_ptr,3);
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len>0)
        {
            AT_Cmd_Param.cell_id[0]=StrtoHex(p_temp+1);
            AT_Cmd_Param.cell_id[1]=StrtoHex(p_temp+3);
        }
    }
    else
    {
        /* error */
        DEBUG(DEBUG_HIGH,"[2G]:Wrong CREG [%d]!\n\r",netsts);
    }
    AT_Cmd_Param.netreg=netsts;
}

static void prvATProt_URC_decode_Pdp(char* rx_buffer, uint8_t rx_count)
{
}

static void prvATProt_URC_decode_Temp(char* rx_buffer, uint8_t rx_count)
{
}

static void prvATProt_URC_decode_Cme_Err(char* rx_buffer, uint8_t rx_count)
{
    // handle CME error
}

static void prvATProt_URC_decode_SMS_List(char* rx_buffer, uint8_t rx_count)
{
}

static void prvATProt_URC_decode_SMS_Recv(char* rx_buffer, uint8_t rx_count)
{
    // Handle SMS data
    
}

static void prvATProt_URC_decode_Net_Recv(char* rx_buffer, uint8_t rx_count)
{
}

/*
static void prvATProt_URC_TCP_Recv(char* rx_buffer, uint8_t rx_count)
{
    // Handle TCP data
    uint8_t i=0;
    uint8_t data_split_pos=0;
    uint16_t data_len=0;
    // With "IPD" header, shall be set with "AT+QIHEAD=1\n\r"
    if (strncmp(rx_buffer,"IPD",3)!=0)
    {
        return;
    }
    for (i=0;i<(rx_count-1);i++)
    {
        if(*(rx_buffer+i)==':')
        {
            data_split_pos=i;
            break;
        }
    }
    // index 3 to ':', data length
    for (i=3;i<data_split_pos;i++)
    {
        if ((*(rx_buffer+i)>'9') || (*(rx_buffer+i)<'0'))
            break;
        data_len=data_len*10+*(rx_buffer+i)-'0';
    }
    // invalid length, exit
    if (data_len==0)
    {
        return;
    }
    // Put data into parser
    prvATProt_Decode_Ipdata(rx_buffer+data_split_pos+1, data_len);
}
*/

static void prvATProt_URC_decode_Clock(char* rx_buffer, uint8_t rx_count)
{
    if (rx_count<=25)
        return;
    rtc_datetime_t datetime;
    
    datetime.year = 2000 + (*(rx_buffer+8) - '0')*10 + (*(rx_buffer+9) - '0');
    datetime.month = (*(rx_buffer+11) - '0')*10 + (*(rx_buffer+12) - '0');
    datetime.day = (*(rx_buffer+14) - '0')*10 + (*(rx_buffer+15) - '0');
    datetime.hour = (*(rx_buffer+17) - '0')*10 + (*(rx_buffer+18) - '0');
    datetime.minute = (*(rx_buffer+20) - '0')*10 + (*(rx_buffer+21) - '0');
    datetime.second = (*(rx_buffer+23) - '0')*10 + (*(rx_buffer+24) - '0');
    if(datetime.year > 2099 || datetime.year < 1970  || datetime.month > 12 || datetime.day > 31 || datetime.minute > 60 || datetime.second > 60 )
    {
        DEBUG(DEBUG_HIGH,"[2G] Data fromat error!\r\n");
    }
    
    RTC_StopTimer(RTC);
    RTC_SetDatetime(RTC, &datetime);
    RTC_StartTimer(RTC);
   /* 2017.9.4 lihaibin modify */  
    //vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_GPRS_ACTIVATED);
}

#if 0
static void prvATProt_parse_sms(char* rx_buffer, uint8_t rx_count)
{
    char tmp_data[100];
    uint8_t data_len=0;
    uint8_t i=0;
    for (i=0;(i*2)<rx_count;i++)
    {
        tmp_data[i]=StrtoHex((uint8_t *)rx_buffer+(i*2));
        data_len++;
    }
    prvATProt_Decode_Ipdata(tmp_data, data_len);
}
#endif

/*********************************************************
*Function Name: 	prvATProt_Resp_Deep_Decode
*Prototype: 		prvATProt_Resp_Deep_Decode(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Deep_Decode(void)
{
    uint8_t cnt;
    for( cnt = 0; cnt < Num_Elems(AT_Resp_decode_table); cnt++)
    {// Deep decode table is not necessary for every command, so need check the table length, not the command length.
        if(AT_Resp_data.Cmd_index == AT_Resp_decode_table[cnt].AT_Res_Index)
        {
            if(NULL != AT_Resp_decode_table[cnt].decode_routine)
            {
                AT_Resp_decode_table[cnt].decode_routine();
            }
            break;
        }
    }
}

/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_IP_INIT_Execute
*Prototype: 		prvATProt_Resp_Decode_IP_INIT_Execute(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_IP_INIT_Execute(void)
{
    if (NET_GPRS_ATTACHED == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
        vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_APN_REGISTERED);
}
/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_IP_REG_Execute
*Prototype: 		prvATProt_Resp_Decode_IP_REG_Execute(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_IP_REG_Execute(void)
{
//    if (NET_APN_REGISTERED == vATProt_Get_TCP_State())
    vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_APN_REGISTERED);
}
/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_SET_Echo
*Prototype: 		prvATProt_Resp_Decode_SET_Echo(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_SET_Echo(void)
{
    if (NET_WAIT_AT == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
        vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_POWER_ON);
}

/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_Answer_Call
*Prototype: 		prvATProt_Resp_Decode_Answer_Call(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_Answer_Call(void)
{
    DEBUG(DEBUG_LOW,"[2G]:Call coming, answered");
}
/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_IMSI_Query
*Prototype: 		prvATProt_Resp_Decode_IMSI_Query(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_CCID_Query(void)
{
    uint8_t i;
    DEBUG(DEBUG_MEDIUM,"iccid:%s\n\r",AT_Resp_data.Resp_IT);
    memcpy(net_Info.ccid, AT_Resp_data.Resp_IT, sizeof(net_Info.ccid));  
    for(i=0;i<10;i++)
    {
        AT_Cmd_Param.iccid[i]=StrtoHex((uint8_t *)&net_Info.ccid[i*2]);
    }
}
/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_IMEI_Query
*Prototype: 		prvATProt_Resp_Decode_IMEI_Query(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_IMEI_Query(void)
{
    uint8_t imei_tmp[16];
    uint8_t i=0;
    memcpy(net_Info.imei, AT_Resp_data.Resp_IT, sizeof(net_Info.imei));
    memcpy(imei_tmp,net_Info.imei,sizeof(net_Info.imei));
    imei_tmp[15]=0;
    DEBUG(DEBUG_MEDIUM,"IMEI:%s\n\r",net_Info.imei);
    imei_tmp[15]='f';
    for(i=0;i<8;i++)
    {
        AT_Cmd_Param.imei[i]=StrtoHex(imei_tmp+(i*2));
    }
}

/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_IMEI_Query
*Prototype: 		prvATProt_Resp_Decode_IMEI_Query(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_AT_Resp_Deep_Decode
**********************************************************/
static void prvATProt_Resp_Decode_IP_Query(void)
{
    if (NET_GPRS_ACTIVATED == vATProt_Get_TCP_State(AT_CONNECT_ID_MAIN))
    {
        vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_IP_GOT);
        //at_cmd_res = AT_CMD_RES_OK; 
        DEBUG(DEBUG_MEDIUM,"%s\n\r",(char *)AT_Resp_data.Resp_IT);
    }
}

static void prvATProt_Resp_Decode_QIClose(void)
{
    uint8_t channel = AT_Resp_data.Resp_IT[0] - '0';
    if(!strncmp(&AT_Resp_data.Resp_IT[2],REMOTE_CLOSE_String,strlen(REMOTE_CLOSE_String)))
    {
        if(AT_CONNECT_ID_MAIN == channel)
        {
			at_cmd_res = AT_CMD_RES_OK;	//set ok flag
//			at_cmd_end = true;
            vATProt_Com_Reset();
            vATProt_Power_Off();
            rl_delay_without_schedule(1000);
            vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
            vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
            vATProt_Power_On();

            vATProt_Com_Reset();
            vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);            
        }
        else if(AT_CONNECT_ID_GB == channel)
        {
            vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);            
        }
    }
}

/*********************************************************
*Function Name: 	prvATProt_Resp_Decode_SW_Ver
*Prototype: 		prvATProt_Resp_Decode_SW_Ver(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_Resp_Decode_SW_Ver
**********************************************************/
static void prvATProt_Resp_Decode_SW_Ver(void)
{
    memcpy(net_Info.swver, AT_Resp_data.Resp_IT, min(sizeof(net_Info.swver),sizeof(AT_Resp_data.Resp_IT)));
}

/*******************************************************************************
*    Function:  prvATProt_Transmit
*
*  Parameters:  None
*     Returns:  None
* Description:  prvATProt_Transmit
*******************************************************************************/
static void prvATProt_Transmit(uint8_t AT_Cmd_index)
{
    AT_Tx_ATTEMPS = AT_Cmd_Info_table[AT_Cmd_index].AT_Retry_Max;
    AT_Resp_data.Cmd_index = 0x00;
    AT_Resp_data.length = 0x00;
    memset(AT_Resp_data.Resp_IT, 0x00, 100);

    if((AT_Cmd_Info_table[AT_Cmd_index].AT_Cmd_Type != INVALID)&&(AT_Cmd_Info_table[AT_Cmd_index].Supported))
    {
        prvATProt_AT_Command_Common_encode(AT_Cmd_index);//firstly encode AT command

        if((AT_Cmd_Info_table[AT_Cmd_index].AT_Cmd_Type & EXCUTE) == EXCUTE)
        {
            prvATProt_AT_Command_Deep_encode(AT_Cmd_index);//application data to be sent
        }
    }

    if(tcom_tx_num_bytes > TCOM_TX_BUF_SIZE - 2)
    {
        //Reserved 2bytes for LF,CR
        tcom_tx_num_bytes = TCOM_TX_BUF_SIZE - 2;
    }
    /*fix suffix CR and LF*/
    tcom_tx_buffer[tcom_tx_num_bytes ++] = CR;
    tcom_tx_buffer[tcom_tx_num_bytes ++] = LF;

//    DEBUG(DEBUG_MEDIUM,"[2F TX]%x",tcom_tx_buffer[tcom_tx_num_bytes-4]);
    
    prvATProt_Uart_Transmit(tcom_tx_buffer, tcom_tx_num_bytes);

    Response_Timeout_timer = AT_Cmd_Info_table[AT_Cmd_index].timeout;
    AT_Resp_Timeout = OS_Time() + Response_Timeout_timer;   //set reception timer
    tcom_frame_tx_attempts ++;

    AT_Cmd_Tx_State = TX_CS_WAIT_FOR_RESP;
}
/*******************************************************************************
*    Function:  prvATProt_Retransmit
*
*  Parameters:  None
*     Returns:  None
* Description:  prvATProt_Retransmit
*******************************************************************************/
static void prvATProt_Retransmit(void)
{
    at_cmd_res = AT_CMD_RES_NONE;
    AT_Resp_Timeout = OS_Time() + Response_Timeout_timer;   //set reception timer
    AT_Cmd_Tx_State = TX_CS_WAIT_FOR_RESP;
    tcom_frame_tx_attempts++;
    prvATProt_Uart_Transmit(tcom_tx_buffer, tcom_tx_num_bytes);
}

/*******************************************************************************
*    Function:  prvATProt_Wait_Resp
*
*  Parameters:  None
*     Returns:  None
* Description:  prvATProt_Wait_resp
*******************************************************************************/
static void prvATProt_Wait_Resp(void)
{
    /* response timeout, response error/CME error: retry */
    /* check response for each command, we need to know which command sent. If get '>'
       for data send, post data after the response. */

    if( (OS_Time() > AT_Resp_Timeout) ||(at_cmd_res == AT_CMD_RES_ERR))//||(at_cmd_res == AT_CMD_RES_CME_ERR) )
    {
        /* 2017.6.20 lihaibin modify.  */
        if(AT_pending_tx_cmd == AT_CMD_SEND_POST_DATA)
        //if(AT_pending_tx_cmd >= AT_CMD_NUM)
         /* end 2017.6.20 lihaibin modify.  */   
        {
            prvATProt_AT_Transmit_Reset();

        }
        else if(tcom_frame_tx_attempts < AT_Tx_ATTEMPS)
        {
            AT_Cmd_Tx_State = TX_CS_RETRANSMIT; //set tx to retransmit state.
        }
        else
        {
            if((at_cmd_res == AT_CMD_RES_NONE) 
                && ((AT_Cmd_Info_table[AT_pending_tx_cmd].AT_Cmd_Type & FATAL) == FATAL))
            {
                //Fatal command, reset module if failed.
                prvATProt_AT_Transmit_Reset();

                vATProt_Com_Reset();
                vATProt_Power_Off();
                rl_delay_without_schedule(1000);
                vATProt_Set_TCP_State(AT_CONNECT_ID_MAIN, NET_INIT);
                vATProt_Set_TCP_State(AT_CONNECT_ID_GB, NET_INIT);
                vATProt_Power_On();
                vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
            }
            else if((at_cmd_res == AT_CMD_RES_NONE) 
                && ((AT_Cmd_Info_table[AT_pending_tx_cmd].AT_Cmd_Type & BASIC) != BASIC))
            {
                /* if no responsed from modem,reset it */
                prvATProt_AT_Transmit_Reset();

            }
            else
            {
                prvATProt_AT_Transmit_Reset();

            }
        }
    }
//    else if(AT_pending_tx_cmd == AT_CMD_SEND_POST_DATA)
//    {
//        if(TCP_TX_POSTDATA == prvATProt_Get_TCP_Mode())
//        {
//            prvATProt_AT_Transmit_Reset();
//        }
//    }
    else if (at_cmd_res == AT_CMD_RES_OK)
    {
        /* 2017.9.30 lihaibin modify */
        at_cmd_res = AT_CMD_RES_NONE;
        AT_Cmd_Tx_State = TX_IDLE;
        //AT_Resp_Rx_State = RX_IDLE;
        AT_pending_tx_cmd = AT_CMD_NONE;
        tcom_frame_tx_attempts = 0;
        //tcom_receiving_frame = false;
        //prvATProt_AT_Transmit_Reset();
    }
}
/*********************************************************
*Function Name: 	prvATProt_AT_Transmit_Reset
*Prototype: 		prvATProt_AT_Transmit_Reset(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		prvATProt_Transmit_Reset
**********************************************************/
static void prvATProt_AT_Transmit_Reset(void)
{
    /* notify app */
/*    if (AT_Cmd_Callback[AT_pending_tx_cmd] != NULL)
    {
        AT_Cmd_Callback[AT_pending_tx_cmd](at_cmd_res);
    }*/
    at_cmd_res = AT_CMD_RES_NONE;
    AT_Cmd_Tx_State = TX_IDLE;
    AT_Resp_Rx_State = RX_IDLE;
    AT_pending_tx_cmd = AT_CMD_NONE;
    tcom_frame_tx_attempts = 0;
    tcom_receiving_frame = false;
}

static void prvATProt_AT_Var_Reset(void)
{
    at_cmd_res = AT_CMD_RES_NONE;
    AT_Cmd_Tx_State = TX_IDLE;
    AT_pending_tx_cmd = AT_CMD_NONE;
    tcom_frame_tx_attempts = 0;
    tcom_receiving_frame = false;
    AT_Resp_Rx_State = RX_IDLE;
    
    memset(tcom_tx_queue,0,sizeof(tcom_tx_queue));
    memset(tcom_tx_buffer, 0x00, TCOM_TX_BUF_SIZE);
    AT_Cmd_Param.sim_ready=0;
    AT_Cmd_Param.gprs_ready=0;
    AT_Cmd_Param.connect_select=AT_CONNECT_ID_MAIN;
    AT_Cmd_Param.tx_select=AT_CONNECT_ID_MAIN;
    AT_Cmd_Param.rx_select=AT_CONNECT_ID_MAIN;
    memcpy(AT_Cmd_Param.APN,DEFAULT_APN,strlen(DEFAULT_APN));

#ifdef TCOM_SUPPORT_SMS
    SMS_back_data.SMS_Len = 0;
    SMS_back_data.receive_no = 0;
    memset(SMS_back_data.SMS_data, 0x00, SMS_DATA_MAX_LEN);
#endif

    net_Info.netActive = false;
//   memset(net_Info.linkSts,0x00,sizeof(net_Info.linkSts));

    prvATProt_Set_TCP_Mode(TCP_IDLE);//Clear TCP mode after module reset.
}

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Common_encode
*Prototype: 		prvATProt_AT_Command_Common_encode(uint8_t AT_Cmd_index)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Common_encode(uint8_t AT_Cmd_index)
{
    uint8_t Cmd_Name_length;

    /*Get the command name length*/
    Cmd_Name_length = strlen(AT_Cmd_Info_table[AT_Cmd_index].AT_Cmd_Name);
    /*Copy command name to tx buffer*/
    strncpy(tcom_tx_buffer, AT_Cmd_Info_table[AT_Cmd_index].AT_Cmd_Name, Cmd_Name_length);
    tcom_tx_num_bytes = Cmd_Name_length;
}

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_SendSMS
*Prototype: 		prvATProt_AT_Command_Deep_encode_SendSMS(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_SendSMS(void)
{
#ifdef TCOM_SUPPORT_SMS
   uint8_t Cmd_Data_length;

   Cmd_Data_length = sprintf(&tcom_tx_buffer[tcom_tx_num_bytes],"%d",SMS_back_data.SMS_Len);
   tcom_tx_num_bytes = tcom_tx_num_bytes + Cmd_Data_length;
   
   tcom_tx_buffer[tcom_tx_num_bytes] = CR;
   tcom_tx_num_bytes++;
#endif
}

#ifdef TEST_ACK_TIMER
static void prvATProt_Reconnect(void)
{
    //prvATProt_AT_Transmit_Reset();
    vATProt_sendAT_Command(AT_CMD_IPCLOSE_EXCUTE, NULL, NULL);
}
#endif

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_SendPostdata
*Prototype: 		prvATProt_AT_Command_Deep_encode_SendPostdata(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_SendPostdata(void)
{
    uint16_t tcp_data_len;

    if( 1 == tcp_tx_mode)//Send ACK
    {
        tcp_data_len = TCP_backup_data.ACK_Len;
        if((tcom_tx_num_bytes + tcp_data_len) < TCOM_TX_BUF_SIZE - 3 )//keep 3 bytes: 0x1A,0x0D,0x0A
        {
            memcpy(&tcom_tx_buffer[tcom_tx_num_bytes],TCP_backup_data.Ack_data,tcp_data_len);
            tcom_tx_num_bytes = tcom_tx_num_bytes+tcp_data_len;
            tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_CTRLZ;
            tcom_tx_num_bytes++;
        }
    }
    else//Send Command
    {
        tcp_data_len = TCP_backup_data.TCP_Len;
        if((tcom_tx_num_bytes + tcp_data_len) < TCOM_TX_BUF_SIZE - 3 )//keep 3 bytes: 0x1A,0x0D,0x0A
        {
            memcpy(&tcom_tx_buffer[tcom_tx_num_bytes],TCP_backup_data.TCP_data,tcp_data_len);
            tcom_tx_num_bytes = tcom_tx_num_bytes+tcp_data_len;
            tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_CTRLZ;
            tcom_tx_num_bytes++;

#ifdef USE_DEBUG
            {
                uint16_t idx;
                DEBUG(DEBUG_LOW,"[2G tx]"); 
                for(idx = 0; idx < tcp_data_len; idx++)
                {
                    DEBUG(DEBUG_LOW,"%02x", TCP_backup_data.TCP_data[idx]);
                }
            }
#endif
            
        }
        else
        {
            if(tcom_tx_num_bytes > TCOM_TX_BUF_SIZE -3)//keep 3 bytes: 0x1B,0x0D,0x0A
            {
                //Reserved 2bytes for LF,CR
                tcom_tx_num_bytes = TCOM_TX_BUF_SIZE -3;
            }
            tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_ESC;
            tcom_tx_num_bytes++;
            DEBUG(DEBUG_HIGH,"[2G]:ERROR:Tx Buffer Overflow!!\n\r");        
        }
#ifdef TEST_ACK_TIMER
        if (TMR_Is_Timer_Stop(GPRS_CONN_TIMER))
            TMR_Start_Timer(GPRS_CONN_TIMER, 60000 , prvATProt_Reconnect);
#endif
    }
    prvATProt_Set_TCP_Mode(TCP_IDLE);//Clear TCP mode, after send out post data.
    //at_cmd_res = AT_CMD_RES_OK; //Delete it, do not set response ok manually. Send post data will wait 100ms timer out.
}

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_Init
*Prototype: 		prvATProt_AT_Command_Deep_encode_Init(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_Init(void)
{
    //uint8_t Cmd_Data_length;
    //Cmd_Data_length = strlen(AT_Cmd_Data.APN);
    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    /*Copy command data to tx buffer*/
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, DEFAULT_APN,sizeof(DEFAULT_APN)-1);
    tcom_tx_num_bytes = tcom_tx_num_bytes+sizeof(DEFAULT_APN)-1;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;
}
#ifdef TCOM_GET_CCLK_EN
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_SetCCLk
*Prototype: 		prvATProt_AT_Command_Deep_encode_SetCCLk(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_SetCCLk(void)
{
    uint8_t Cmd_Data_length;

    Cmd_Data_length = strlen(Tcom_cclk_cal_time);

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;
    /*Copy command data to tx buffer*/
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, Tcom_cclk_cal_time,Cmd_Data_length);
    tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;
}
#endif
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_Open
*Prototype: 		prvATProt_AT_Command_Deep_encode_Open(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_Open(void)
{
    uint8_t Cmd_Data_length;
    IP_Link_Setting_t connect_param;
    if (AT_Cmd_Param.connect_select==AT_CONNECT_ID_MAIN)
    {
        strncpy(AT_Cmd_Param.IP_Open_Setting[0].IP_LinkType,"TCP",4);
        Get_IP_config((uint8_t *)AT_Cmd_Param.IP_Open_Setting[0].Dest_IP, (uint8_t *)AT_Cmd_Param.IP_Open_Setting[0].Dest_port);
        memcpy(&connect_param, &AT_Cmd_Param.IP_Open_Setting[0], sizeof(IP_Link_Setting_t));
    }
    if (AT_Cmd_Param.connect_select==AT_CONNECT_ID_GB)
    {
        strncpy(connect_param.IP_LinkType,"TCP",4);
        memcpy(connect_param.Dest_IP, GB_DOMAIN_NAME, 16);
        memcpy(connect_param.Dest_port, GB_PORT, 6);
    }

    // multiple link, < 10
#if 1
    if (AT_Cmd_Param.mux_type==1)
    {
        // connection
        tcom_tx_buffer[tcom_tx_num_bytes++] = '0' + AT_Cmd_Param.connect_select;
        // ---add "," ---
        tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;
    }
#endif
    // Copy link type
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;
    Cmd_Data_length = strlen(connect_param.IP_LinkType);
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, connect_param.IP_LinkType,Cmd_Data_length);
    tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;

    // ---add "," ---
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;

    // Copy dest IP
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;
    Cmd_Data_length = strlen(connect_param.Dest_IP);
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, connect_param.Dest_IP,Cmd_Data_length);
    tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;

    // ---add "," ---
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;

    // Copy dest port
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;
    Cmd_Data_length = strlen(connect_param.Dest_port);
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, connect_param.Dest_port,Cmd_Data_length);
    tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_QUOTES;
}

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_Send
*Prototype: 		prvATProt_AT_Command_Deep_encode_Send(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_Send(void)
{
    uint16_t Cmd_Data_length = 0;

    if (AT_Cmd_Param.mux_type==1)
    {
        // connection id
        tcom_tx_buffer[tcom_tx_num_bytes++] = '0' + AT_Cmd_Param.tx_select;
        // ---add "," ---
        tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;
  
        Cmd_Data_length = DECtoStr((uint8_t *)(tcom_tx_buffer+tcom_tx_num_bytes),(uint8_t *)(&(TCP_backup_data.TCP_Len)),2);
        tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
        tcp_tx_mode = 0;//send command
        prvATProt_Set_TCP_Mode(TCP_TXING);
    }
    else
    {
        Cmd_Data_length = DECtoStr((uint8_t *)(tcom_tx_buffer+tcom_tx_num_bytes),(uint8_t *)(&(TCP_backup_data.TCP_Len)),2);
        tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
        tcp_tx_mode = 0;//send command
        prvATProt_Set_TCP_Mode(TCP_TXING);
    }
}

/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_SAPBR_phase_1
*Prototype: 		prvATProt_AT_Command_Deep_encode_SAPBR_phase_1(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_SAPBR_phase_1(void)
{
    memcpy(tcom_tx_buffer+tcom_tx_num_bytes, "3,1,\"Contype\",\"GPRS\"", strlen("3,1,\"Contype\",\"GPRS\""));
    tcom_tx_num_bytes += strlen("3,1,\"Contype\",\"GPRS\"");
}
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_SAPBR_phase_2
*Prototype: 		prvATProt_AT_Command_Deep_encode_SAPBR_phase_2(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_SAPBR_phase_2(void)
{
    sprintf(tcom_tx_buffer+tcom_tx_num_bytes, "3,1,\"APN\",\"%s\"",AT_Cmd_Param.APN);   
    tcom_tx_num_bytes += (strlen("3,1,\"APN\",\"\"") + strlen(AT_Cmd_Param.APN));
}
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_CNTP_phase_1
*Prototype: 		prvATProt_AT_Command_Deep_encode_CNTP_phase_1(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_CNTP_phase_1(void)
{
//    memcpy(tcom_tx_buffer+tcom_tx_num_bytes, "\"202.120.2.101\",32", strlen("\"202.120.2.101\",32"));
//    tcom_tx_num_bytes += strlen("\"202.120.2.101\",32");
    memcpy(tcom_tx_buffer+tcom_tx_num_bytes, "\"1.cn.pool.ntp.org\",32", strlen("\"1.cn.pool.ntp.org\",32"));
    tcom_tx_num_bytes += strlen("\"1.cn.pool.ntp.org\",32");
        
}
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode_QICLOSE
*Prototype: 		prvATProt_AT_Command_Deep_encode_QICLOSE(void)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode_QICLOSE(void)
{
    char conn_num = 0;
    switch(AT_Cmd_Param.connect_select)
    {
        case AT_CONNECT_ID_MAIN:
            conn_num = '0';
            break;
        case AT_CONNECT_ID_GB:
            conn_num = '1';
            break;
        default:
            return;
    }  
    strncat(tcom_tx_buffer+tcom_tx_num_bytes,&conn_num,1);
    tcom_tx_num_bytes += 1;
}
/***********************************************************************************
*Function Name: 	prvATProt_AT_Command_Deep_encode
*Prototype: 		prvATProt_AT_Command_Deep_encode(uint8_t AT_Cmd_index)
*Called by: 		app
*Parameters:		void
*Returns:			void
*Description:		common encode
************************************************************************************/
static void prvATProt_AT_Command_Deep_encode(uint8_t AT_Cmd_index)
{
    if (ATProt_encode_table[AT_Cmd_index] != NULL)
    {
        ATProt_encode_table[AT_Cmd_index]();
    }
}

static void prvATProt_AT_Command_Deep_encode_Sendack(void)
{
}

static void prvATProt_AT_Command_Deep_encode_Multiip(void)
{
    tcom_tx_buffer[tcom_tx_num_bytes++] = '1';
    AT_Cmd_Param.mux_type=1;
}

static void prvATProt_AT_Command_Deep_encode_Cfun(void)
{
}

static void prvATProt_AT_Command_Deep_encode_Rxget(void)
{
}

// AT command for server connection
static void prvATProt_AT_Command_Deep_encode_QIOPEN	(void)
{
}

// AT command for data send
static void prvATProt_AT_Command_Deep_encode_QISEND	(void)
{
    uint16_t Cmd_Data_length = 0;

    Cmd_Data_length = DECtoStr((uint8_t *)(tcom_tx_buffer+tcom_tx_num_bytes),(uint8_t *)(&(TCP_backup_data.TCP_Len)),2);
    tcom_tx_num_bytes = tcom_tx_num_bytes+Cmd_Data_length;
    tcp_tx_mode = 0;//send command
    prvATProt_Set_TCP_Mode(TCP_TXING);
}

// AT command for data read, not used
static void prvATProt_AT_Command_Deep_encode_QIRD	(void)
{
}

// APN setting
static void prvATProt_AT_Command_Deep_encode_QIREGAPP	(void)
{
    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    /*Copy command data to tx buffer*/
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, AT_Cmd_Param.APN,strlen(AT_Cmd_Param.APN));
    tcom_tx_num_bytes = tcom_tx_num_bytes+strlen(AT_Cmd_Param.APN);

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    // ---add "," ---
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    /*Copy command data to tx buffer*/
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, AT_Cmd_Param.APN_user,strlen(AT_Cmd_Param.APN_user)-1);
    tcom_tx_num_bytes = tcom_tx_num_bytes+strlen(AT_Cmd_Param.APN_user)-1;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    // ---add "," ---
    tcom_tx_buffer[tcom_tx_num_bytes++] = CHAR_COMMA;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;

    /*Copy command data to tx buffer*/
    strncpy(tcom_tx_buffer+tcom_tx_num_bytes, AT_Cmd_Param.APN_pwd,strlen(AT_Cmd_Param.APN_pwd)-1);
    tcom_tx_num_bytes = tcom_tx_num_bytes+strlen(AT_Cmd_Param.APN_pwd)-1;

    tcom_tx_buffer[tcom_tx_num_bytes] = CHAR_QUOTES;
    tcom_tx_num_bytes++;
}

// Delete all SMS
static void prvATProt_AT_Command_Deep_encode_Delete_SMS	(void)
{
    // Delete all SMS in SIM card
    tcom_tx_buffer[tcom_tx_num_bytes++] = '1';
    tcom_tx_buffer[tcom_tx_num_bytes++] = ',';
    tcom_tx_buffer[tcom_tx_num_bytes++] = '4';
}

static void prvATProt_AT_Command_Deep_encode_ReadSMS	(void)
{
    if(current_sms_index<=9)
    {
        tcom_tx_buffer[tcom_tx_num_bytes++] = current_sms_index+'0';
    }
    else
    {
        tcom_tx_buffer[tcom_tx_num_bytes++] = (current_sms_index/10)+'0';
        tcom_tx_buffer[tcom_tx_num_bytes++] = (current_sms_index%10)+'0';
    }
//    tcom_tx_num_bytes++;
}

/*******************************************************************************
* Function:  prvATProt_AT_Send
*
* Parameters:  None
* Returns:  None
* Description:  send cmd to Telematics module
*******************************************************************************/
static void prvATProt_AT_Send_Set(AT_cmd_t send_cmd_index)
{
    Set_Bit_RNGE_Check(tcom_tx_queue, send_cmd_index, AT_CMD_NUM);
}

/*******************************************************************************
* Function: prvATProt_Uart_Get_Char
*
* Parameters: &data
* Returns: 8-bit data or -1 if buffer is empty
* Description: The SIPPROT specific UART receive routine which complements
*              Com_Put_Char. See that function for more information.
*******************************************************************************/
static int16_t prvATProt_Uart_Get_Char (uint8_t*data)
{
    int16_t ret_value;

    if (Uart_Get_Char(UART_GSM_CHANNEL, data)) 
    {
        ret_value = (int16_t) (*data);
    }
    else
    {
        ret_value = -1;
    }

    return ret_value;
}

/*******************************************************************************
* Function:  prvATProt_Uart_Transmit
*
* Parameters:  None
* Returns:  None
* Description:  
*******************************************************************************/
uint8_t prvATProt_Uart_Transmit(const char* tx_buf, uint16_t bytes)
{
    uint8_t ret = 0;
    uint16_t i;

    for(i = 0; i < bytes; i++)
    {
        Uart_Put_Char(UART_GSM_CHANNEL,*(tx_buf + i));
    }

    return ret;
}

void vATProt_Set_TCP_State(uint8_t connect_id, uint8_t state)
{
    if (connect_id>MAX_CONNECT_ID)
        return;
    if(net_Info.tcp_state[connect_id] !=state )
    {
        /*if(( state == NET_APN_REGISTERED)
            ||( state == NET_GPRS_ACTIVATED)
            ||( state == NET_IP_GOT))*/
        {
//            vATApp_Retry_Clear();            
        }
        if (state < NET_STATE_NUM)
        {
            net_Info.tcp_state[connect_id] = state;    
        }
        if (state >= NET_TCP_CONNECTED)
        {
            net_Info.netActive = true;
        }
        else
        {
            net_Info.netActive = false;
        }
    }
}

uint8_t vATProt_Get_TCP_State(uint8_t connect_id)
{
    if (connect_id>MAX_CONNECT_ID)
        return 0;
    return net_Info.tcp_state[connect_id];
}

void vATProt_Set_Rxtype(uint8_t type)
{
    AT_Cmd_Param.rx_type = type;
}

#if 0
static void prvATProt_Set_Recv_Len(uint16_t len)
{
    if (len > TCOM_RX_BUF_SIZE)
        tcp_rx_len = TCOM_RX_BUF_SIZE;
    else
        tcp_rx_len = len;
}
#endif 

static void prvATProt_Set_TCP_Mode(uint8_t mode)
{
    net_Info.srv_status = mode;
#if 0
    if(TCP_IDLE != net_Info.srv_status)
    {
        tcp_mode_Timeout_timer = OS_Time()+MSec_To_Ticks(5000);
    }
#endif
}

static uint8_t prvATProt_Get_TCP_Mode(void)
{
    return net_Info.srv_status;
}

static uint8_t prvATProt_Decode_Ipdata(char *data, uint16_t len)
{
    if (len > 0)
    {
        vTelmProt_informData((uint8_t *)data, len);
#ifdef TEST_ACK_TIMER
        TMR_Stop_Timer(GPRS_CONN_TIMER);
#endif
    }
    return 0;
}

uint8_t ATProt_Sim_OK(void)
{
    if (net_Info.tcp_state[0] >= NET_SIM_CONNECTED)
        return 1;
    else
        return 0;
}

uint8_t ATProt_GPRS_Connect(void)
{
/*    if (net_Info.tcp_state >= NET_GPRS_ACTIVATED)
        return 1;
    else
        return 0;*/
    return 1;
}

uint8_t ATProt_ACK_Got(void)
{
    return ack_from_server;
}

// Get IP string

#ifndef USE_DOMAIN_NAME
static void get_server_ip()
{

}
#endif

#if 0
// Get port string
static void get_server_port()
{

}
#endif

void accept_call(void)
{
    vATProt_sendAT_Command(AT_CMD_ANSWER_CALL, NULL, NULL);
}

uint8_t module_alive(void)
{
    if (net_Info.module_go_sleep == 1)
    {
/*        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0)
        {
            DEBUG(DEBUG_MEDIUM, "[2G]:2G module Off\n\r");
            return 0;
        }
        else*/
            return 1;
    }
    else
    {
        return 1;
    }
}

/* Report 2G module temperature */
int8_t ATProt_Get_Temp(void)
{
    return 0;
}

uint8_t ATProt_Rx_Buf_Empty(void)
{
    if ((Net_Rx_Data_Num==0) || (Net_Rx_Data_Num==Net_Rx_Read_Index))
        return 1;
    else
        return 0;
}

uint8_t ATProt_Rx_Buf_Full(void)
{
    if ((Net_Rx_Data_Num==(TCOM_RX_BUF_PACKET_NUM-1)) && 
        (Net_Rx_Data_Num>Net_Rx_Read_Index))
        return 1;
    else
        return 0;
}

void ATProt_Rx_Buf_Clear(void)
{
    uint8_t i=0;
    Net_Rx_Data_Num=0;
    Net_Rx_Read_Index=0;
    for (i=0;i<TCOM_RX_BUF_PACKET_NUM;i++)
    {
        memset(Net_Rx_Data[i],0,TCOM_RX_BUF_SIZE);
        Net_Rx_Data_Size[i]=0;
    }
}

uint8_t ATProt_Push_Rx_Buf(uint8_t *rx_buf, uint16_t length)
{
//    uint8_t rt=0;
    if (ATProt_Rx_Buf_Full())
        return 0;
    if (Net_Rx_Data_Num<TCOM_RX_BUF_PACKET_NUM)
    {
        memcpy(Net_Rx_Data[Net_Rx_Data_Num],rx_buf,TCOM_RX_BUF_SIZE);
        Net_Rx_Data_Size[Net_Rx_Data_Num]=length;
        Net_Rx_Data_Num++;
    }
    return 1;
}

uint16_t ATProt_Get_Rx_Buf(uint8_t *rx_buf)
{
    if (Net_Rx_Data_Num>Net_Rx_Read_Index)
    {
        uint16_t rt=Net_Rx_Data_Size[Net_Rx_Read_Index];
        memcpy(rx_buf,Net_Rx_Data[Net_Rx_Read_Index],TCOM_RX_BUF_SIZE);
        Net_Rx_Read_Index++;
        return rt;
    }
    else
    {
        ATProt_Rx_Buf_Clear();
        return 0;
    }
}

uint8_t ATProt_Sim_Ready(void)
{
    return AT_Cmd_Param.sim_ready;
}

uint8_t ATProt_GPRS_Ready(void)
{
    return AT_Cmd_Param.gprs_ready;
}

void ATProt_Get_Loc(uint8_t *lac,uint8_t *cell_id)
{
    memcpy(lac,AT_Cmd_Param.lac,2);
    memcpy(cell_id,AT_Cmd_Param.cell_id,2);
}

uint8_t ATProt_Get_Netreg(void)
{
    return AT_Cmd_Param.netreg;
}

void ATProt_Get_Imei(uint8_t *imei)
{
    memcpy(imei, AT_Cmd_Param.imei, 8);
}

void ATProt_Get_Imsi(uint8_t *imsi)
{
    memcpy(imsi, AT_Cmd_Param.imsi, 8);
}

void ATProt_Get_ICCID(uint8_t *iccid)
{
    if(iccid == NULL)
    {
        return ;
    }
    memcpy(iccid, AT_Cmd_Param.iccid, sizeof(AT_Cmd_Param.iccid));
}

uint8_t ATProt_Get_Clock(uint8_t *clk)
{
    if (clock_ref.year[0]=='2')
    {
        memcpy(clk, &clock_ref, sizeof(AT_Clock_t));
        return 1;
    }
    else
    {
        return 0;
    }
}

void ATProt_Select_Connection(uint8_t connect_id)
{
    AT_Cmd_Param.connect_select=connect_id; 
}

void ATProt_Select_Tx(uint8_t connect_id)
{
    AT_Cmd_Param.tx_select=connect_id;
}

uint8_t ATProt_Get_Rx(void)
{
    return AT_Cmd_Param.rx_select;
}

//2017.9.22 lihaibin modify
bool ATProt_Is_Module_Ok(void)
{
    return net_Info.isModuleOk;
}

uint8_t ATProt_GetCurrentDataSession(void)
{
    return AT_Cmd_Param.connect_select;
}

/*
void ATProt_test_decode(void)
{
    uint8_t test_data[]={0x55,
    };
    prvATProt_Decode_Ipdata(test_data,1);
}*/

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
