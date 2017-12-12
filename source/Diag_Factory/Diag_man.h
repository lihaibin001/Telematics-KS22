/**********************************************************************
 *  Title:  Diag_man.h
 *
 *  Module Description: This file contains all standard exports for Diag_Man.
 *
 *********************************************************************/
#ifndef DIAG_MAN_H
#define DIAG_MAN_H

/**********************************************************************
 * Include files                                                       
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define MDG_CAN_FRAME_SIZE    8
#define DIAG_TX_FRAME_SIZE    4

#define  DIAG_TX_ID            0x671    // Transmit ID   
#define  DIAG_RX_ID            0x670    // Receive ID    
#define  DIAG_MSG_LEN       8

#define PDSN_NOR_FLASH_ADDR  DEVICE_INFO_ADDR
#define MBSN_NOR_FLASH_ADDR  (DEVICE_INFO_ADDR+TELM_INFO_LEN_DEVID)

#define MFG_PACKET_SIZE 32


#define  FLASH_Sector_size     0x00001000 // 4k
#define  APP_CODE_Start_Addr     0x000C0000
#define  APP_CODE_size     0x00000000
#define  APP_CODE_Sectors   64  // 256K /4k =64
#define ApplicationAddress 0x8002000
#define  ROM_APP_Start_Address	0x8002000
#define  Nor_FLASH_Sector_size	0x00001000 // 4k
#define  NOR_FLASH_OTA_Addr     0x000C0000
/**********************************************************************
  * Enumerations and Structures and Typedefs
  *********************************************************************/
typedef enum diag_ant_status_Tag
{
   ANT_NORMAL,
   ANT_OPEN,
   ANT_SHORT,
} diag_ant_status_T;

typedef enum C003_Response_Code_tag
{
    C003_CMD_STARTED,
    C003_CMD_FINISHED
} C003_Response_Code_Type;

typedef enum MDG_Response_Code_tag
{
    MDG_RESP_NO_ERROR,
    MDG_RESP_CMD_NOT_SUPPORT,
    MDG_RESP_COMMAND_FAILED,
    MDG_RESP_DATA_OUT_RANGE,
    MDG_RESP_DATA_NOT_WRITEABLE,
    MDG_RESP_CANNOT_GET_DATA,
    MDG_RESP_NOT_ENTER_DIAG_MODE
} MDG_Response_Code_Type;

typedef enum SW_UP_Response_Code_tag
{
    SW_UP_RESP_NO_ERROR,
    SW_UP_RESP_FILE_SIZE_ERROR,
    SW_UP_RESP_INDEX_ERROR,
    SW_UP_RESP_PACKET_CHECKSUN_ERROR,
    SW_UP_RESP_SW_CHECKSUM_ERROR,
    SW_UP_RESP_DOWNLOAD_OK,
} SW_UP_Response_Code_Type;

typedef struct MDG_Cmd_Info_tag
{
	bool Multi_Frame;
	uint8_t Command;
	uint8_t Data;
	bool Pending_Flag;
	uint16_t Pending_Time;
	bool MCU_Tx_Flag;
	bool MPU_Tx_Flag;
	MDG_Response_Code_Type Response_Code;
} MDG_Cmd_Info_Type;

typedef enum AP_Factory_Cmd_Tag
{
	AP_FACTORY_CMD_CHECKSUM_GET = 0x01,
	AP_FACTORY_CMD_APP_CHECKSUM_GET = 0x02,

	AP_FACTORY_CMD_SW_VER_GET = 0x03,
	AP_FACTORY_CMD_IMSI_GET = 0x04,
	AP_FACTORY_CMD_PDSN_GET = 0x05,
	AP_FACTORY_CMD_PDSN_SET = 0x06,
	AP_FACTORY_CMD_MBSN_GET = 0x07,
	AP_FACTORY_CMD_MBSN_SET = 0x08,
	AP_FACTORY_CMD_GPSR_SIM_GET = 0x09,
	AP_FACTORY_CMD_GPSR_SIM_NW_GET = 0x0A,
	AP_FACTORY_CMD_GPSR_INFOR_GET = 0x0B,
	AP_FACTORY_CMD_GPS_INFOR_GET = 0x0C,
	AP_FACTORY_CMD_GPS_SAT_INFOR_GET = 0x2C,

	AP_FACTORY_CMD_GSENSOR_INFOR_GET = 0x0D,
	AP_FACTORY_CMD_FASTSLEEP_SET = 0x0E,
	AP_FACTORY_CMD_COLDSTART_SET = 0xF,

	AP_FACTORY_CMD_K_L_LINE_STATUS_GET = 0x10,
	AP_FACTORY_CMD_IMEI_GET = 0x11,
	AP_FACTORY_CMD_TEST_RESULT_SET = 0x12,
	AP_FACTORY_CMD_GPRS_SIGNAL_GET = 0x13,

	AP_FACTORY_CMD_CLEAR_NOR_FLASH_SET = 0x30,
	
	AP_FACTORY_CMD_SW_UPGRADE_DATA = 0x80,
	AP_FACTORY_CMD_SW_UPGRADE_START = 0x81,

	AP_FACTORY_CMD_NUMS
} AP_Factory_Cmd_T;



/**********************************************************************
 * Global Variable extern Declarations
 *********************************************************************/
extern uint8_t Diag_Tx_Message[DIAG_TX_FRAME_SIZE];
extern bool Mdg_Enter_Diag;
extern uint8_t Mdg_SW_Upgrage_mode;
extern uint8_t MDG_Tx_Message[MDG_CAN_FRAME_SIZE];
extern uint8_t MDG_Rx_Message[MDG_CAN_FRAME_SIZE];
extern bool MDG_Get_New_Message;
extern MDG_Cmd_Info_Type MDG_Info;


extern void MDG_Encode_Diag_Message(void);
extern void Diag_periodic_diag_hook(void);
extern void MDG_Parse_Diag_Message(void);

extern bool MDG_In_Mode(void);
extern void MDG_Init(void);
extern void MDG_Clear_Mode(void);
#endif

/**********************************************************************
* REVISION RECORDS
*********************************************************************/
/**********************************************************************
* $Log:
*
*********************************************************************/

