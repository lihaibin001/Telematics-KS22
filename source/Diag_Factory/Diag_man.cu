/**********************************************************************
 *  Title:  Diag_man.cu
 *
 *  Module Description:  This file has the applications specific
 *                              code for Diag_Man.  This file is included
 *                              directly by Diag_Man.C.
 *
 *********************************************************************/
 
/**********************************************************************
* Constant and Macro Definitions using #define
*********************************************************************/

/**********************************************************************
* Enumerations and Structures and Typedefs
*********************************************************************/
typedef struct            
{
	uint16_t update_flag;	/*  if update_flag=0x5555,bootloader will upgrade the app*/
	uint32_t sw_size;		/* software size   */
	uint16_t total_packet_number;
	uint32_t sw_verson;
	uint16_t checksum;
}SW_infor_t;
/**********************************************************************
* Global and Const Variable Defining Definitions / Initializations
*********************************************************************/
//uint8_t Tuner_Ant_Status;
//uint8_t Tuner_Ant_Last_Status;

extern AP_Factory_Data_T AP_Factory_Data;
/**********************************************************************
* Static Variables and Const Variables With File Level Scope
*********************************************************************/
static const uint8_t mdg_enter_diag_key[] = {0xFA, 0x45, 0x39, 0x52, 0x72, 0x57, 0x3E, 0x29};
static const uint8_t mdg_exit_diag_key[] = {0xFB, 0x3A, 0x63, 0x7D, 0x35, 0x5A, 0x27, 0x40};


static uint16_t bin_file_size=0;
static uint16_t packets_received=0;
static uint8_t packets_index=0;

static uint16_t packet_checksum=0;
static uint16_t crc_reg;
static uint16_t pre_crc_reg;

static uint8_t  * code_data;
static uint32_t NorFlashAddr;
static SW_infor_t sw_infor;
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static void NOR_Flash_OTASectors_Erase(void);
/**********************************************************************
 * Function Definitions
 *********************************************************************/

/**********************************************************************
 *  Description: CAN_upgrade_app_sw
 *  Parameters: data
 *  Returns: 
 *********************************************************************/
static uint8_t rec_index;
static uint8_t flash_read_buf[32];
static uint8_t CAN_upgrade_app_sw(uint8_t *data,uint8_t cmd)
{
	SW_infor_t * rec_sw_infor;
	switch(cmd)
	{
		case AP_FACTORY_CMD_SW_UPGRADE_START:  
			/* header packet  to prepera downloader,and stop data downloader*/
			Mdg_SW_Upgrage_mode=true;
			rec_sw_infor=(SW_infor_t *)(data-3); //four bytes update flag
			sw_infor.sw_size=rec_sw_infor->sw_size;
			sw_infor.sw_verson=rec_sw_infor->sw_verson;
			sw_infor.checksum=rec_sw_infor->checksum;
			//if (sw_infor.sw_verson == 0 ||sw_infor.sw_size==0)  // 0X0803FFFF-0X08002000
			if (sw_infor.sw_verson == 0 ||sw_infor.sw_size==0)  
			{
				return SW_UP_RESP_FILE_SIZE_ERROR;//repose file size error
			}
//			OS_Suspend_Task(OS_IOT_TASK);
//			OS_Suspend_Task(OS_GPS_TASK);
			/* Image size is greater than Flash size */
			NOR_Flash_OTASectors_Erase();
			NorFlashAddr=NOR_FLASH_OTA_Addr+32;
			crc_reg=0XFFFF;
			packets_index=0;
			packets_received =1;
			break;
		case AP_FACTORY_CMD_SW_UPGRADE_DATA:
			packets_index++;
			rec_index=*data;
			if(packets_index!=*data)
			{
				packets_index--;
				rec_index=*data;
				return SW_UP_RESP_INDEX_ERROR;// packets index error
			}
			packet_checksum=(uint16_t)(data[1]+(uint16_t)(data[2]<<8)); // get packet checksum
			
			code_data =(uint8_t *)(data+3); //get software data pointer
			//packet_checksum
			crc_reg=crc_ccitt(crc_reg, code_data, MFG_PACKET_SIZE);
			if(packet_checksum!=crc_reg)
			{
				//this packet is wrong, prepare to get this packet again
				packets_index--;
				crc_reg=pre_crc_reg; 
				rec_index=*data;
				return SW_UP_RESP_PACKET_CHECKSUN_ERROR;// packets index error
			}
			// program code data into Nor Flash
			sFLASH_WriteBuffer(code_data,NorFlashAddr,32);
			sFLASH_ReadBuffer(flash_read_buf,NorFlashAddr,32);
			if(0 != memcmp(code_data, flash_read_buf, 32))
			{
				//this packet is wrong, prepare to get this packet again
				packets_index--;
				crc_reg=pre_crc_reg; 
				rec_index=*data;
				return SW_UP_RESP_PACKET_CHECKSUN_ERROR;// packets index error
			}
			pre_crc_reg=crc_reg;  //this packet is ok, save the crc result of the packet.
			NorFlashAddr+=32;
			packets_received ++;
			// first data packet is 2
			if((packets_received-1)*MFG_PACKET_SIZE>=sw_infor.sw_size)  // all code data received 
			{
				if(crc_reg!=sw_infor.checksum) // verify checksum 
				{
					return SW_UP_RESP_SW_CHECKSUM_ERROR;
				}
				else
				{     // set update flag,return OK to pc,wait reset cmd to execute bootloader
					sw_infor.update_flag=0x55AA;  
					sFLASH_WriteBuffer((uint8_t *)&sw_infor, APP_CODE_Start_Addr, 8);
					return SW_UP_RESP_DOWNLOAD_OK;
				}
			}
			break;
		default:
			break;
	}
	return SW_UP_RESP_NO_ERROR;
}

/**********************************************************************
 *  Description: Write_PDSN_of_devinfo
 *  Parameters: data
 *  Returns: 
 *********************************************************************/
static void Write_PDSN_of_devinfo(uint8_t *data)
{
/*    DevInfo_Sct_t info;
    sFLASH_ReadBuffer(info.byte, DEVICE_INFO_ADDR, DEVICE_INFO_LENGTH);
    memcpy(info.structData.dev_sn,data,DEV_SN_LENGTH);
    //set_init_flag();
    sFLASH_WriteBuffer(info.byte,DEVICE_INFO_ADDR,DEVICE_INFO_LENGTH);*/
}
/**********************************************************************
 *  Description: Write_MBSN_of_devinfo
 *  Parameters: data
 *  Returns: 
 *********************************************************************/
static void Write_MBSN_of_devinfo(uint8_t *data)
{
    DevInfo_Sct_t info;
    sFLASH_ReadBuffer(info.byte, DEVICE_INFO_ADDR, DEVICE_INFO_LENGTH);
    memcpy(info.structData.mb_ver,data,BOARD_VERSION_LENGTH);  //modfied MBSN data length to 20 bytes
    //sFLASH_EraseSector(INIT_FLAG_ADDR);
    //set_init_flag();
    sFLASH_WriteBuffer(info.byte,DEVICE_INFO_ADDR,DEVICE_INFO_LENGTH);
}
/**********************************************************************
 *  Description: NOR_Flash_OTASectors_Erase
 *  Parameters: data
 *  Returns: 
 *********************************************************************/
static void NOR_Flash_OTASectors_Erase(void)
{
	uint16_t i;
	uint32_t SectorAddr;
	SectorAddr=APP_CODE_Start_Addr;
	for(i=0;i<APP_CODE_Sectors;i++)
	{
		/* Erase SPI FLASH Sector to write on */
		sFLASH_EraseSector(SectorAddr);
		SectorAddr+=FLASH_Sector_size;
	}
}
/**********************************************************************
* REVISION RECORDS
*********************************************************************/
/**********************************************************************
* $Log:
* 
* -Rev 1.0  ; 6 June 2013 ; Yiting Hu
*   Initial Version
*
*********************************************************************/

