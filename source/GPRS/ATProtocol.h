#ifndef _AT_PROTOCOL_H_
#define _AT_PROTOCOL_H_
#include "GPRS.h"
/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
#define AT_MAX_CID			(3)		//max cid number in command 'CGACT'

#ifdef TCOM_SUPPORT_SMS
#define SMS_DATA_MAX_LEN   300
#endif

#define TCP_TX_MAX_LEN      512
#define TCP_TX_ACK_LEN      3



/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/
typedef struct IP_Link_Setting_tag
{
    char IP_LinkType[4];
    char Dest_IP[30];
    char Dest_port[6];
}IP_Link_Setting_t;

#ifdef TCOM_SUPPORT_SMS
typedef struct SMS_glb_info_tag
{
    uint8_t SMS_Len;
    uint8_t receive_no;
    uint8_t SMS_data[SMS_DATA_MAX_LEN];
}SMS_glb_info;
#endif

typedef enum AT_cmd_tag
{
    AT_CMD_NONE,
    AT_CMD_NO_ECHO_SETTING, // End of use
    AT_CMD_SET_CLIP,  // End of use
    AT_CMD_SET_ECHO, //Add for A1050D echo canceller setting. End of use
    AT_CMD_GET_SW_VER,

    AT_CMD_GET_CCID_NO,  // iccid
    AT_CMD_GET_IMEI_NO,	// imei cmd
    AT_CMD_SET_CGREG,
    AT_CMD_SET_CREG,	
    AT_CMD_HANG_UP,  // End of use

    AT_CMD_DIAL,  // End of use
    AT_CMD_SEND_SMS,  // End of use
    AT_CMD_SEND_POST_DATA,  // End of use//20
    AT_CMD_ACTIVE_PDP_CONTEXT,
    AT_CMD_CGATT,
/* 2017.6.23 lihaibin modify */
    AT_CMD_SAPBR_PHASE_1,
    AT_CMD_SAPBR_PHASE_2,
    AT_CMD_SAPBR_PHASE_3,
    AT_CMD_CNTPCID,
    AT_CMD_CNTP_1,
    AT_CMD_CNTP_2,
    AT_CMD_CCLK,    
/* end 2017.6.23 lihaibin modify */    
    AT_CMD_ATH,  // End of use
    AT_CMD_CSQ_QUERY,
    AT_CMD_IP_INIT_EXCUTE,
    AT_CMD_IP_INIT_CHECK,
    AT_CMD_IP_OPEN_EXCUTE,

    AT_CMD_IPCLOSE_EXCUTE,
    AT_CMD_IP_RX_MANUALLY_QUERY,
    AT_CMD_QIOPEN,
    AT_CMD_QISEND,
    AT_CMD_QICLOSE,

    AT_CMD_QINDI,
    AT_CMD_QIRD,
    AT_CMD_IPD_HEAD_SET,
    AT_CMD_QIREGAPP,
    AT_CMD_DELETE_SMS,

    AT_CMD_SMS_FORMAT,
    AT_CMD_NEW_SMS_SETTING,
    AT_CMD_SYNC_RTC,
    AT_CMD_CGACT_QUERY,
    AT_CMD_IP_INIT_QUERY,


    
    AT_CMD_IP_OPEN_QUERY,
    AT_CMD_IP_SEND_ACK_EXCUTE,
    AT_CMD_DET_SIM_QUERY,
    AT_CMD_IP_REG_EXCUTE,
    AT_CMD_LOCAL_IP_ADDR_QUERY,

    AT_CMD_IP_SEND_DATA_EXCUTE,//40
    AT_CMD_IP_CONFIG_CONN_EXCUTE,
    AT_CMD_FUNCTION,
    AT_CMD_CHECK_GPRS,
    AT_CMD_READ_SMS_QUERY,

    AT_CMD_CREG_QUERY,
    AT_CMD_IP_QUERY,
    AT_CMD_SLEEP,
    AT_CMD_SLEEP_QUERY,
    AT_CMD_POWER_DOWN,

    AT_CMD_ANSWER_CALL,
    AT_CMD_TEMP_READ,

    AT_CMD_CIPHEAD,
    

    AT_CMD_NUM
}AT_cmd_t;
//Size = 336
typedef struct AT_Cmd_Param_tag                           
{
    uint8_t callID;
    char APN[20];
    char APN_user[20];
    char APN_pwd[20];
    IP_Link_Setting_t IP_Open_Setting[2];
    uint8_t send_len;
    uint8_t mux_type;
    uint8_t fun_type;
    uint8_t sms_read_index;
    uint8_t sms_delete_index;
    uint8_t rx_type;
    uint8_t sim_ready;
    uint8_t gprs_ready;
    uint8_t imei[8];
    uint8_t imsi[8];
    uint8_t iccid[10];
    uint8_t lac[2];
    uint8_t cell_id[2];
    uint8_t csq;
    uint8_t netreg;
    uint8_t connect_select;
    uint8_t tx_select;
    uint8_t rx_select;
}AT_Cmd_Param_t;

typedef enum TCP_Conn_State_Tag
{
    NET_INIT=0,
    NET_WAIT_AT, // 1
    NET_POWER_ON,// 2
    NET_ECHO_CLOSED, // 3
    NET_SIM_CONNECTED,// 4
    NET_WATI_CSQ_OK,   //5 2017.9.20 lihaibin modiify
    NET_REGISTERED,// 6
    NET_GPRS_ATTACHED,// 7
    NET_APN_REGISTERED,// 8
    NET_GPRS_ACTIVATED, // 9
    NET_IP_GOT, // 10
    NET_TCP_CONNECTED,// 11
    NET_DATA_TRANS,// 12
    NET_STATE_NUM
}TCP_Conn_State_T;

typedef enum AT_URC_tag
{
    AT_URC_NONE,
    AT_URC_CGREG,
    AT_URC_SIGNAL, //2017.9.20 lihaibin modify
    AT_URC_RECV,
    AT_URC_CGATT,
    AT_URC_CMT,
    AT_URC_CMGR,
    AT_URC_CPIN,
    AT_URC_CMTI,
    AT_URC_CREG,
    AT_URC_PDP,
    AT_URC_CMTE,
    AT_URC_CME_ERR,
    AT_URC_CMGL,
    AT_URC_SMS_RECV,
    AT_URC_RX_RECV,
    AT_URC_CCLK,
//    AT_URC_CNTP,
    AT_URC_NUM
}AT_URC_E;

typedef void (*URC_Notify_fptr)(void);

typedef struct TCP_data_tag
{
    uint16_t TCP_Len;
    uint16_t ACK_Len;
    uint8_t TCP_data[TCP_TX_MAX_LEN];
    uint8_t Ack_data[TCP_TX_ACK_LEN];
}TCP_data_t;



/**********************************************************************
 * Global Function Prototypes
 *********************************************************************/
extern AT_Cmd_Param_t AT_Cmd_Param;
extern TCP_data_t TCP_backup_data;

extern void vATProt_Init(void);
extern void vATProt_Power_On(void);
extern void vATProt_Power_Off(void);
extern void vATProt_Com_Reset(void);
extern void vATProt_Upgrade_Reset(void);
extern void vATProt_Check_Transmit(void);
extern void vATProt_Check_Receive(void);
extern void vATProt_sendAT_Command(AT_cmd_t cmd, AT_Cmd_Param_t const* data, callBack cb);
extern bool bATProt_isNetWorkActive(void);
extern bool bATProt_isInterNetActive(void);
extern bool bATProt_isIPLINK_Active(char const* type,char const *ip,char const *port);
extern char const * pcATProt_getImsiData(void);
extern char const * pcATProt_getImeiData(void);
extern char const * pcATProt_getRecivedData(uint8_t *len);
extern char const * pcATProt_getIncomingNumber(void);
extern void vATProt_Add_URC_Listener(AT_URC_E event, URC_Notify_fptr handler);
extern void vATProt_Remove_URC_Listener(AT_URC_E event);

extern void vATProt_Set_TCP_State(uint8_t connect_id, uint8_t state);
extern uint8_t vATProt_Get_TCP_State(uint8_t connect_id);
extern void vATProt_Set_Rxtype(uint8_t type);
extern uint8_t pcATProt_getRSSI(void);

extern void vATProt_GoSleep(void);
extern void vATProt_WakeUp(void);

extern uint8_t ATProt_Sim_OK(void);
extern uint8_t ATProt_GPRS_Connect(void);
extern uint8_t ATProt_ACK_Got(void);

extern void accept_call(void);
extern uint16_t ATProt_decode_rx(char *buffer);

//Other application external function
#ifdef TCOM_GET_CCLK_EN
extern void vATProt_time_CalChk(void);
#endif

extern int8_t ATProt_Get_Temp(void);
extern uint8_t ATProt_Rx_Buf_Empty(void);
extern uint8_t ATProt_Rx_Buf_Full(void);
extern void ATProt_Rx_Buf_Clear(void);
extern uint8_t ATProt_Push_Rx_Buf(uint8_t *rx_buf, uint16_t length);
extern uint16_t ATProt_Get_Rx_Buf(uint8_t *rx_buf);

extern uint8_t ATProt_Sim_Ready(void);
extern uint8_t ATProt_GPRS_Ready(void);
extern uint8_t ATProt_Power_Status(void);
extern void ATProt_Get_Loc(uint8_t *lac,uint8_t *cell_id);
extern uint8_t ATProt_Get_Netreg(void);
extern void ATProt_Get_Imei(uint8_t *imei);
extern void ATProt_Get_Imsi(uint8_t *imsi);
extern void ATProt_Get_ICCID(uint8_t *iccid);
extern uint8_t ATProt_Get_Clock(uint8_t *clk);
extern void ATProt_Select_Connection(uint8_t connect_id);
extern void ATProt_Select_Tx(uint8_t connect_id);
extern uint8_t ATProt_Get_Rx(void);
extern uint8_t ATProt_getRSSI(void);
bool ATProt_Is_Module_Ok(void);
uint8_t ATProt_GetCurrentDataSession(void);

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif	/* _AT_PROTOCOL_H_ */
