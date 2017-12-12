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
 *  Title:  Diag_man.c
 *
 *  Module Description:  This is the code file for Factory Diagnostic.
 *
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
#include "Standard.h"
#include "TelmProtocol.h"
#include "gps.h"
#include "ATProtocol.h"
#include "crc_ccitt.h"
#include "diag_man.h"
/**********************************************************************
* Static Variables and Const Variables With File Level Scope
*********************************************************************/
static uint8_t diag_tx_multi_frame_num;

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define MDG_ERROR_TRIES           5

#define DIAG_PERIODIC_TIMER       20
#define DIAG_PERIODIC_HOOK_CYCLE  (1000 / DIAG_PERIODIC_TIMER)  //1s

#define MDG_CMD_PENGDING_TIME     200   // 2s

/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/
typedef struct AP_Factory_Data_Tag
{
	uint8_t crc16_checksum[2];
	uint8_t data_addr[2];
	uint8_t data[48];
	uint8_t data_totle_len;
	uint8_t data_cur_len;
} AP_Factory_Data_T;


typedef union app_sw_length_Tag
{
	uint8_t len[4];
	uint32_t length;
} app_sw_length_T;
/**********************************************************************
* Global and Const Variable Defining Definitions / Initializations
*********************************************************************/
uint8_t Diag_Tx_Message[DIAG_TX_FRAME_SIZE];
static bool Mdg_Enter_Diag = false;
__no_init bool mdg_in_mode;
uint8_t Mdg_SW_Upgrage_mode=false;
uint8_t MDG_Tx_Message[MDG_CAN_FRAME_SIZE];
uint8_t MDG_Rx_Message[MDG_CAN_FRAME_SIZE];
bool MDG_Get_New_Message = false;
MDG_Cmd_Info_Type MDG_Info;

AP_Factory_Data_T AP_Factory_Data;
/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
//STATIC_VAR Tick_Type mdg_mfg_diag_timer;                                
app_sw_length_T app_sw_length;
                            
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/

static void mdg_decode_diag_message(const uint8_t *rx_msg);
static void mdg_encode_diag_message(void);
static void mdg_check_enter_daignostic_mode(const uint8_t *rx_msg);
static void mdg_check_exit_daignostic_mode(const uint8_t *rx_msg);
static void mdg_clear_mfg_diag_timer(void);
static bool mdg_mfg_diag_timer_is_expired(void);
static void mdg_reset_mfg_diag_timer(void);
static void mdg_exit_mfg_diag_mode(void);
static void mdg_send_response(void);
static void mdg_info_mcu_report_no_error(void);

/**********************************************************************
* Add User defined functions
*********************************************************************/
#include "Diag_man.cu"

/**********************************************************************
*	File Static Prototype Declare Section
**********************************************************************/

/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/***********************************************************************
*  Function: Diag_periodic_diag_hook 
*  Parameters: void
*  Returns: void
*  Description: periodic diag ,if the state changed ,then Report results 
***********************************************************************/
void Diag_periodic_diag_hook(void)
{
//	diag_evt_main_DAB_ant_status(false);
}

/***********************************************************************
 *  Function: mdg_decode_diag_message 
 *  Parameters: rx_msg--pointer to messsage to decode
 *  Returns: none
 *  Description: Decodes standard 8 byte manufacturing diagnostic message
 ***********************************************************************/
static void mdg_decode_diag_message(const uint8_t *rx_msg)
{
	uint8_t frame_ctl;
	uint8_t command;
	uint16_t crc_result;
	gps_data_t gpsInfo;

	if(rx_msg[0] == mdg_exit_diag_key[0])
	{
		mdg_check_exit_daignostic_mode(rx_msg);
		return;
	}
	else if(rx_msg[0] == mdg_enter_diag_key[0])
	{
		mdg_check_enter_daignostic_mode(rx_msg);
		return;
	}
	else if(Mdg_Enter_Diag == false)
	{
		return;
	}

	frame_ctl = rx_msg[0];
	command = rx_msg[1];
	MDG_Info.Command = command;
//	MDG_Info.Length = rx_msg[2];
	MDG_Info.MCU_Tx_Flag = false;
	MDG_Info.MPU_Tx_Flag = false;
	MDG_Info.Response_Code = MDG_RESP_NO_ERROR;

	Diag_Tx_Message[DIAG_TX_FRAME_SIZE - 1] = C003_CMD_STARTED;

	switch(command)
	{
		case AP_FACTORY_CMD_CHECKSUM_GET:
			crc_result = rom_crc_ccitt();
			AP_Factory_Data.crc16_checksum[0] = crc_result >> 8;
			AP_Factory_Data.crc16_checksum[1] = crc_result & 0xFF;
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_APP_CHECKSUM_GET:
			memcpy(&app_sw_length.len[0], &rx_msg[3], 4);
			crc_result=0xffff;
			crc_result = crc_ccitt(crc_result,(uint8_t *)ROM_APP_Start_Address,app_sw_length.length);
			AP_Factory_Data.crc16_checksum[0] = crc_result >> 8;
			AP_Factory_Data.crc16_checksum[1] = crc_result & 0xFF;
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_SW_VER_GET:
			AP_Factory_Data.data[0]=*(SY_Swid()+13);
			AP_Factory_Data.data[1]=*(SY_Swid()+14);
			AP_Factory_Data.data[2]=*(SY_Swid()+15);
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_IMSI_GET:
			memcpy(AP_Factory_Data.data,pcATProt_getImsiData(),15);
			AP_Factory_Data.data_totle_len =15;  //data length
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_PDSN_GET:
			memcpy(AP_Factory_Data.data, Telm_Get_DEVID(), TELM_INFO_LEN_DEVID);
			AP_Factory_Data.data_totle_len = TELM_INFO_LEN_DEVID;  //data length
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_IMEI_GET:
			memcpy(AP_Factory_Data.data, pcATProt_getImeiData(), 15);
			AP_Factory_Data.data_totle_len =15;  //data length
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_PDSN_SET: //Write PDSN to Nor flash
		case AP_FACTORY_CMD_MBSN_SET: //Write MBSN to Nor flash
		case AP_FACTORY_CMD_TEST_RESULT_SET: //Write test result to Nor flash
		case AP_FACTORY_CMD_CLEAR_NOR_FLASH_SET: //Clear Nor Flash
		case AP_FACTORY_CMD_SW_UPGRADE_DATA: //APP software upgrade
		case AP_FACTORY_CMD_SW_UPGRADE_START:
			if(frame_ctl == 0x11)  //only one frame
			{
				memcpy(&AP_Factory_Data.data[0], &rx_msg[3], rx_msg[2]);
				AP_Factory_Data.data_totle_len = rx_msg[2];
				if(command==AP_FACTORY_CMD_TEST_RESULT_SET)
				{
					set_factory_test(AP_Factory_Data.data[0]);
					mdg_info_mcu_report_no_error();
				}
				else if(command==AP_FACTORY_CMD_CLEAR_NOR_FLASH_SET)
				{
					clear_nv_config();
					mdg_info_mcu_report_no_error();
				}
			}
			else if((frame_ctl & 0x0F) == 0x01)  //fist frame of mutil-frame
			{
				memcpy(&AP_Factory_Data.data[0], &rx_msg[3], 5);
				AP_Factory_Data.data_totle_len = rx_msg[2];   // SCB:bug "-1 " need deleted 
				AP_Factory_Data.data_cur_len = 5;
			}
			else if(AP_Factory_Data.data_totle_len - AP_Factory_Data.data_cur_len > 6)
			{
				memcpy(&AP_Factory_Data.data[AP_Factory_Data.data_cur_len], &rx_msg[2], 6);
				AP_Factory_Data.data_cur_len += 6;
			}
			else
			{
				memcpy(&AP_Factory_Data.data[AP_Factory_Data.data_cur_len], &rx_msg[2], AP_Factory_Data.data_totle_len - AP_Factory_Data.data_cur_len);
				if(command==AP_FACTORY_CMD_PDSN_SET)
				{
					clear_nv_config();//clear all NV
					set_init_flag();//set NV init flag					
					Write_PDSN_of_devinfo(AP_Factory_Data.data);
					write_init_config();//write network config to NV
					load_from_nv();//load new config right now
				}
				else if(command==AP_FACTORY_CMD_MBSN_SET)
				{
					Write_MBSN_of_devinfo(AP_Factory_Data.data);
				}
				else if(command==AP_FACTORY_CMD_SW_UPGRADE_START ||command==AP_FACTORY_CMD_SW_UPGRADE_DATA)
				{
				// Add for CAN bootloader 
					AP_Factory_Data.data[0]=CAN_upgrade_app_sw(AP_Factory_Data.data,command);
					AP_Factory_Data.data[1]=rec_index;
					AP_Factory_Data.data[2]=(uint8_t)(crc_reg&0xff);
					AP_Factory_Data.data[3]=(uint8_t)(crc_reg>>8);
				}
				mdg_info_mcu_report_no_error();
			}
			break;
		case AP_FACTORY_CMD_MBSN_GET:
/*			sFLASH_ReadBuffer(AP_Factory_Data.data, MBSN_NOR_FLASH_ADDR, TELM_INFO_LEN_MBSN);
			AP_Factory_Data.data_totle_len =TELM_INFO_LEN_MBSN;  //data length
			mdg_info_mcu_report_no_error();*/
			break;
		case AP_FACTORY_CMD_GPSR_SIM_GET:
			AP_Factory_Data.data[0]=ATProt_Sim_OK();
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_GPSR_SIM_NW_GET:
			AP_Factory_Data.data[0]=ATProt_GPRS_Connect();
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_GPSR_INFOR_GET:
			AP_Factory_Data.data[0]=ATProt_ACK_Got();
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_GPRS_SIGNAL_GET:
			AP_Factory_Data.data[0]=pcATProt_getRSSI();
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_GPS_INFOR_GET:
			if(GPS_Get_Ready_Flag() == true)
			{
				vGps_Get_Gps_Info(&gpsInfo);
				AP_Factory_Data.data_totle_len =32;  //data length
				AP_Factory_Data.data[0]=gpsInfo.east_or_west;
				memcpy(&AP_Factory_Data.data[1],&gpsInfo.longitude,15);
				AP_Factory_Data.data[17]=gpsInfo.north_or_sourth;
				memcpy(&AP_Factory_Data.data[18],&gpsInfo.latitude,15);
	//			AP_Factory_Data.data[32]=gpsInfo.gps_sat_info.used_sat_num;
	//			AP_Factory_Data.data[33]=gpsInfo.gps_sat_info.sat_info[0].sat_sig;  //get fist star signle
				mdg_info_mcu_report_no_error();
			}
			break;
		case AP_FACTORY_CMD_GPS_SAT_INFOR_GET:
			if(GPS_Get_Ready_Flag() == true)
			{
				vGps_Get_Gps_Info(&gpsInfo);
				AP_Factory_Data.data_totle_len =25;  //data length
				AP_Factory_Data.data[0]=gpsInfo.gps_sat_info.viewed_sat_num;
				memcpy(&AP_Factory_Data.data[1],gpsInfo.gps_sat_info.sat_info,2*AP_Factory_Data.data[0]);
				mdg_info_mcu_report_no_error();
			}
			break;
		case AP_FACTORY_CMD_GSENSOR_INFOR_GET:
//			GSensor_Read_Xyz_Value();
			AP_Factory_Data.data_totle_len =6;  //data length
//			AP_Factory_Data.data[0]=(uint8_t)(xAccel_mg>>8);
//			AP_Factory_Data.data[1]=(uint8_t)(xAccel_mg&0xff);
//			AP_Factory_Data.data[2]=(uint8_t)(yAccel_mg>>8);
//			AP_Factory_Data.data[3]=(uint8_t)(yAccel_mg&0xff);
//			AP_Factory_Data.data[4]=(uint8_t)(zAccel_mg>>8);
//			AP_Factory_Data.data[5]=(uint8_t)(zAccel_mg&0xff);
			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_FASTSLEEP_SET:
			Sys_Clear_Wakeup_Src_Flags();
			Sys_Req_Enter_Deep_Standby();
//			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_COLDSTART_SET:
			SY_Cold_Start();
//			mdg_info_mcu_report_no_error();
			break;
		case AP_FACTORY_CMD_K_L_LINE_STATUS_GET:
//			AP_Factory_Data.data[0] = (Diag_Get_Active_Protocol()==OBD_ISO14230_4_FAST_INIT)? 0x01:0x00;
//			AP_Factory_Data.data[1] = 0x01;//(Diag_Get_Active_Protocol()==OBD_ISO14230_4_FAST_INIT)? 0x00:0x01;
//			mdg_info_mcu_report_no_error();
			break;

		default:
			MDG_Info.MCU_Tx_Flag = true;
			MDG_Info.Response_Code = MDG_RESP_CMD_NOT_SUPPORT;
			MDG_Info.Pending_Flag = true;
			MDG_Info.Pending_Time = 0;
			break;
	}
}

/***********************************************************************
*  Function: mdg_encode_diag_message 
*  Parameters: none
*  Returns: none
*  Description: Encodes standard 8 byte manufacturing diagnostic message
***********************************************************************/
static void mdg_encode_diag_message(void)
{
	uint8_t i;
	static uint8_t multi_frame = 1;

	MDG_Tx_Message[0] = 0x11;
	MDG_Tx_Message[1] = MDG_Info.Command;
//	MDG_Tx_Message[2] = MDG_Info.Length;
	MDG_Tx_Message[3] = MDG_Info.Response_Code;

	for(i = 0; i < 5; i++)
	{
		MDG_Tx_Message[4 + i] = 0;
	}
	
	if(MDG_Info.Response_Code == MDG_RESP_NO_ERROR)
	{
		switch(MDG_Info.Command)
		{
			case AP_FACTORY_CMD_CHECKSUM_GET:
			case AP_FACTORY_CMD_APP_CHECKSUM_GET:
				MDG_Tx_Message[2] = 3;
//				MDG_Tx_Message[3] = AP_Factory_Data.crc16_checksum[0];
//				MDG_Tx_Message[4] = AP_Factory_Data.crc16_checksum[1];
				memcpy(&MDG_Tx_Message[4],&AP_Factory_Data.crc16_checksum[0],2);
				break;
			case AP_FACTORY_CMD_SW_VER_GET:
				MDG_Tx_Message[2] = 4; //data length
				memcpy(&MDG_Tx_Message[4],&AP_Factory_Data.data[0],3);
				break;
			case AP_FACTORY_CMD_SW_UPGRADE_DATA:
			case AP_FACTORY_CMD_SW_UPGRADE_START:
				MDG_Tx_Message[2] = 5; //data length
				memcpy(&MDG_Tx_Message[4],&AP_Factory_Data.data[0],4);
			case AP_FACTORY_CMD_GPSR_SIM_GET:
			case AP_FACTORY_CMD_GPSR_SIM_NW_GET:
			case AP_FACTORY_CMD_GPSR_INFOR_GET:
			case AP_FACTORY_CMD_GPRS_SIGNAL_GET:
				MDG_Tx_Message[2] = 2; //data length
				MDG_Tx_Message[4] =AP_Factory_Data.data[0];
				break;
			case AP_FACTORY_CMD_K_L_LINE_STATUS_GET:
				MDG_Tx_Message[2] = 3; //data length
				memcpy(&MDG_Tx_Message[4],&AP_Factory_Data.data[0],2);
				break;
			case AP_FACTORY_CMD_IMSI_GET:
			case AP_FACTORY_CMD_PDSN_GET:
			case AP_FACTORY_CMD_MBSN_GET:
			case AP_FACTORY_CMD_GPS_INFOR_GET:
			case AP_FACTORY_CMD_GPS_SAT_INFOR_GET:
			case AP_FACTORY_CMD_GSENSOR_INFOR_GET:
			case AP_FACTORY_CMD_IMEI_GET:
				diag_tx_multi_frame_num =( AP_Factory_Data.data_totle_len -4)/ 6 + ((((AP_Factory_Data.data_totle_len-4) % 6) > 0) ? 1 : 0)+1;

				if(multi_frame <= diag_tx_multi_frame_num)
				{
					MDG_Tx_Message[0] = ((diag_tx_multi_frame_num) << 4) + multi_frame;

					if(multi_frame == 1)
					{
						MDG_Tx_Message[2] = AP_Factory_Data.data_totle_len+1;
						memcpy(&MDG_Tx_Message[4],AP_Factory_Data.data,4);
						AP_Factory_Data.data_cur_len = 4;
						multi_frame = 2;
						MDG_Info.Multi_Frame = true;
					}
					else
					{
						if((AP_Factory_Data.data_totle_len - AP_Factory_Data.data_cur_len) > 6)
						{
							memcpy(&MDG_Tx_Message[2], &AP_Factory_Data.data[AP_Factory_Data.data_cur_len], 6);

							AP_Factory_Data.data_cur_len += 6;

							multi_frame++;
							MDG_Info.Multi_Frame = true;
						}
						else
						{
							memcpy(&MDG_Tx_Message[2], &AP_Factory_Data.data[AP_Factory_Data.data_cur_len], AP_Factory_Data.data_totle_len - AP_Factory_Data.data_cur_len);

							multi_frame = 1;
							MDG_Info.Multi_Frame = false;
						}
					}
				}
				break;
			case AP_FACTORY_CMD_PDSN_SET:
			case AP_FACTORY_CMD_MBSN_SET:
			case AP_FACTORY_CMD_FASTSLEEP_SET:
			case AP_FACTORY_CMD_COLDSTART_SET:
			case AP_FACTORY_CMD_TEST_RESULT_SET:
			case AP_FACTORY_CMD_CLEAR_NOR_FLASH_SET:
				MDG_Tx_Message[2] = 1;
				break;
			default:
				break;
		}
	}
	else
	{
		MDG_Tx_Message[2] = 1;
	}

//	MDG_Info.Command = 0;
//	MDG_Info.Length = 0;
//	MDG_Info.Response_Code = MDG_RESP_NO_ERROR;
}

/***********************************************************************
 *  Function: mdg_check_enter_daignostic_mode 
 *  Parameters: rx message
 *  Returns: Manufacturing Daignostics message  test mode state
 *  Description: get the mode state 
 ***********************************************************************/
static void mdg_check_enter_daignostic_mode(const uint8_t *rx_msg)
{
	if(0 == memcmp(rx_msg, mdg_enter_diag_key, 8))         // Enter diagnostic mode msg
	{
		Mdg_Enter_Diag = true;
		mdg_in_mode = true;
/*		if(!PS_Full_System())
		{
		       SY_Cold_Start();
		}*/

		memcpy((uint8_t *)MDG_Tx_Message, (uint8_t *)rx_msg, 8);
		mdg_send_response();
	}
	else
	{
		MDG_Info.Command = rx_msg[1];
//		MDG_Info.Length = rx_msg[2];
		MDG_Info.MCU_Tx_Flag = true;
		MDG_Info.Response_Code = MDG_RESP_NOT_ENTER_DIAG_MODE;
	}
}

/***********************************************************************
 *  Function: mdg_check_exit_daignostic_mode 
 *  Parameters: rx message
 *  Returns: Manufacturing Daignostics message  test mode state
 *  Description: get the mode state 
 ***********************************************************************/
static void mdg_check_exit_daignostic_mode(const uint8_t *rx_msg)
{
	if(0 == memcmp(rx_msg, mdg_exit_diag_key, 8))   // Exit diagnostic mode msg
	{
		Mdg_Enter_Diag = false;

		memcpy((uint8_t *)MDG_Tx_Message, (uint8_t *)rx_msg, 8);
		mdg_send_response();
	}
}

/**********************************************************************      
 *   Function: mdg_clear_mfg_diag_timer
 *   Parameters: None
 *   Returns: None
 *   Description: Clears mdg_mfg_diag_timer
 **********************************************************************/
static void mdg_clear_mfg_diag_timer(void)
{
	mdg_mfg_diag_timer = OS_Time();   //Clear the 5 Minute Timer
}

/**********************************************************************      
*   Function: mdg_mfg_diag_timer_is_expired
*   Parameters: None
*   Returns: true if mdg_mfg_diag_timer has expired
*   Description: function for checking if mfg_mode_timer is expired
**********************************************************************/
static bool mdg_mfg_diag_timer_is_expired(void)
{
	return (mdg_mfg_diag_timer <= OS_Time());
}

/**********************************************************************      
 *   Function: mdg_reset_mfg_diag_timer
 *   Parameters: None
 *   Returns: None
 *   Description: Resets mdg_mfg_diag_timer
 **********************************************************************/
static void mdg_reset_mfg_diag_timer(void)
{
	mdg_mfg_diag_timer = (OS_Time() + (5 * ONE_MINUTE_IN_TICKS));      //Start 5 minute timer
}

/**********************************************************************
*   Function: mdg_exit_mfg_diag_mode 
*   Parameters: void
*   Returns: void
*   Description: Exit out of Manufacturing Diagnostics Mode
 *********************************************************************/
static void mdg_exit_mfg_diag_mode(void)
{
	mdg_clear_mfg_diag_timer();

	Mdg_Enter_Diag = false;

//	MDG_Fan_Ctl_Override = false;
}

/**********************************************************************
*   Function: mdg_send_response 
*   Parameters: void
*   Returns: void
*   Description: 
 *********************************************************************/
static void mdg_send_response(void)
{
#if 0
  PduInfoType pduinfo;
	pduinfo.SduDataPtr = MDG_Tx_Message;
	pduinfo.SduLength = 8;
	pduinfo.CanId = 0;//CanId must set as 0 if use configured CAN id
	/* Send SF */
	CanIf_Transmit(CANIF_TX_vEcuC_Pdu6, &pduinfo);
#endif
	Diag_Tx_Message[DIAG_TX_FRAME_SIZE - 1] = C003_CMD_FINISHED;
}

/**********************************************************************
*   Function:    MDG_Parse_Diag_Message
*   Parameters: none
*   Returns:    none
*   Description: 
**********************************************************************/
void MDG_Parse_Diag_Message(void)
{
	if(MDG_Get_New_Message == true)
	{
		MDG_Get_New_Message = false;

		mdg_reset_mfg_diag_timer();

		if(Mdg_Enter_Diag == false)
		{
			mdg_check_enter_daignostic_mode(MDG_Rx_Message);
		}
		else
		{
			mdg_decode_diag_message(MDG_Rx_Message);
		}
	}

	if(MDG_Info.Pending_Flag == true)
	{
		if(MDG_Info.Pending_Time > MDG_CMD_PENGDING_TIME)
		{
			MDG_Info.Response_Code = MDG_RESP_CANNOT_GET_DATA;
			MDG_Info.MPU_Tx_Flag = true;
			MDG_Info.Pending_Flag = false;
		}
	}

	if(MDG_Info.MPU_Tx_Flag == true)
	{
		mdg_send_response();

		MDG_Info.MPU_Tx_Flag = false;
	}
	else if((MDG_Info.MCU_Tx_Flag == true) || (MDG_Info.Multi_Frame == true))
	{
		mdg_encode_diag_message();

		mdg_send_response();

		MDG_Info.MCU_Tx_Flag = false;
	}
	
	if((Mdg_Enter_Diag == true) && mdg_mfg_diag_timer_is_expired())
	{
		mdg_exit_mfg_diag_mode();
	}
}

/**********************************************************************
*   Function:    MDG_info_set_report_value
*   Parameters: none
*   Returns:    none
*   Description: 
**********************************************************************/
static void mdg_info_mcu_report_no_error(void)
{
//	MDG_Info.Pending_Flag = false;
	MDG_Info.MCU_Tx_Flag = true;
	MDG_Info.Response_Code = MDG_RESP_NO_ERROR;
}

/**********************************************************************
*   Function: MDG_In_Mode 
*   Parameters: void
*   Returns: void
*   Description: 
 *********************************************************************/
bool MDG_In_Mode(void)
{
	return mdg_in_mode;
}

/**********************************************************************
*   Function: MDG_Clear_Mode 
*   Parameters: void
*   Returns: void
*   Description: 
 *********************************************************************/
void MDG_Clear_Mode(void)
{
	mdg_in_mode = false;
}

/**********************************************************************
*   Function: MDG_Init 
*   Parameters: void
*   Returns: void
*   Description: 
 *********************************************************************/
void MDG_Init(void)
{
	mdg_in_mode = false;
}

/**********************************************************************
* REVISION RECORDS
*********************************************************************/
/**********************************************************************
* $Log:
*
*********************************************************************/

