/*********************************************************************************************************
 * FileName       : PRJ_CONFIG.H
 * Author         : 
 * Description    : Contains switchs, function list, special define and others which depense on project
 * Version        : 
 * Function List  : 
 * Config ID      : 
**********************************************************************/
#ifndef _PRJ_CONFIG_H_
#define _PRJ_CONFIG_H_

#include "vehicle.h"
#include "telmprotocol.h"
#include "GPRS.h"
#define MSec_To_Ticks(msec)   (msec)//((((msec) * OS_TICKS_SEC)+999) / 1000U)
                                             // converts millseconds to ticks

/** @defgroup GPIO_pins_define 
  * @{
  */
#define GPIO_Pin_0                 (0u)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 (1u)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 (2u)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 (3u)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 (4u)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 (5u)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 (6u)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 (7u)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 (8u)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 (9u)  /*!< Pin 9 selected */
#define GPIO_Pin_10                (10u)  /*!< Pin 10 selected */
#define GPIO_Pin_11                (11u)  /*!< Pin 11 selected */
#define GPIO_Pin_12                (12u)  /*!< Pin 12 selected */
#define GPIO_Pin_13                (13u)  /*!< Pin 13 selected */
#define GPIO_Pin_14                (14u)  /*!< Pin 14 selected */
#define GPIO_Pin_15                (15u)  /*!< Pin 15 selected */
#define GPIO_Pin_16                (16u)  /*!< Pin 16 selected */
#define GPIO_Pin_17                (17u)  /*!< Pin 17 selected */

/**********************************************************************
 * DEBUG Switches
 *********************************************************************/

/**********************************************************************
 * BENCH Switches
 *********************************************************************/
//#define SUPPORT_SENSORS
#define NORMAL_POWER_MODE 0
#define LOW_POWER_MODE 1
#define FULL_POWER_MODE 2

// temporary data setting for environment tests
//#define EV_TEST_DATA
//#define USE_DEFAULT_CONFIG
//#define TELM_TEST_PHONE
#define BOARD_VER (2)
#define DEVICE_DEFAULT_SN "300100200300400f"

/*
								Flash division
	--------------------------------------------------------------------------
				AREA								SIZE
	--------------------------------------------------------------------------
		Manufacture setting		 					4K
	--------------------------------------------------------------------------
				Config					  			4k
	--------------------------------------------------------------------------
				GPS Info							4k
	--------------------------------------------------------------------------
				GPS Log								512k
	--------------------------------------------------------------------------
				Battery Log							4k
	--------------------------------------------------------------------------
				OTA Info							4k
	--------------------------------------------------------------------------
				OTA Data							512k
	--------------------------------------------------------------------------
				Sys log								8k
	--------------------------------------------------------------------------
				Reserved							-
	--------------------------------------------------------------------------

*/

/**********************************************************************
 * FUNCTION Switches
 *********************************************************************/

/**********************************************************************
* DEBUG MACRO
**********************************************************************/

/*********************************************************************
* Project Flash Config
**********************************************************************/
/* Flash definitions */
#define FLASH_MANUFACTURE_SETTING_OFFSET		0
#define FLASH_CONFIG_OFFSET						(4 * 1024)
#define FLASH_RLDT_INFO_OFFSET					(8 * 1024)
#define FLASH_RLDT_LOG_OFFSET					(12 * 1024)
//#define FLASH_BATTERY_LOG_OFFSET				(524 * 1024)
#define FLASH_RLDT_LOG_END                      (3744 * 1024)
//#define FLASH_WARN_INFO_OFFSET                  (3724 * 1024)
//#define FLASH_WARN_LOG_OFFSET                   (3728 * 1024)
//#define FLASH_WARN_LOG_END                      (3744 * 1024)
#define FLASH_OTA_INFO_OFFSET					(3744 * 1024)
#define FLASH_OTA_DATA_OFFSET					(3748 * 1024)
//#define FLASH_SYS_LOG_OFFSET					(1044 * 1024)
//#define FLASH_DEV_START_OFFSET				(1052 * 1024)
//#define FLASH_RESERVED_OFFSET					(1056 * 1024)

#define FLASH_RLDT_CNT                          1000                       

/*
#define FLASH_MANUFACTURE_SETTING_SIZE			(FLASH_CONFIG_OFFSET - FLASH_MANUFACTURE_SETTING_OFFSET)
#define FLASH_CONFIG_SIZE						(FLASH_RLDT_INFO_OFFSET - FLASH_CONFIG_OFFSET)
#define FLASH_GPS_INFO_SIZE						(FLASH_RLDT_LOG_OFFSET - FLASH_RLDT_INFO_OFFSET)
#define FLASH_GPS_LOG_SIZE						(FLASH_BATTERY_LOG_OFFSET - FLASH_RLDT_LOG_OFFSET)
#define FLASH_BATTERY_LOG_SIZE					(FLASH_OTA_INFO_OFFSET - FLASH_BATTERY_LOG_OFFSET)
#define FLASH_OTA_INFO_SIZE						(FLASH_OTA_DATA_OFFSET - FLASH_OTA_INFO_OFFSET)
#define FLASH_OTA_DATA_SIZE						(FLASH_SYS_LOG_OFFSET - FLASH_OTA_DATA_OFFSET)
#define FLASH_SYS_LOG_SIZE						(FLASH_RESERVED_OFFSET - FLASH_SYS_LOG_OFFSET)
#define FLASH_RESERVED_SIZE						(2 * 1024 * 1024 - FLASH_RESERVED_OFFSET)
*/

#define INITIALIZE_DATA							(0xa5a5)
#define START_OTA_FLAG 							(0x55AA)
#define DEV_START_FLAG 							(0x55AA)
#define INIT_FLAG_DATA 							INITIALIZE_DATA

#define GPS_BACKUP_ADDR							FLASH_RLDT_LOG_OFFSET
#define DEVICE_INFO_ADDR 						(FLASH_MANUFACTURE_SETTING_OFFSET)
#define OTA_LIVE_HEADER_ADDR					FLASH_OTA_INFO_OFFSET
#define DEVICE_CONFIG_ADDR						FLASH_CONFIG_OFFSET
#define OTA_DATA_ADDR							FLASH_OTA_DATA_OFFSET
#define OTA_HEADER_ADDR							FLASH_OTA_INFO_OFFSET
#define INIT_FLAG_ADDR							FLASH_MANUFACTURE_SETTING_OFFSET

#define DEVICE_INFO_LENGTH (14)


#define CONFIG_DATA_LEN (72)
#define GPS_INFO_LENGTH (10)
#define GPS_LOG_LENGTH (26)
#define BATTERY_LOG_LENGTH (7)
#define OTA_HEADER_LENGTH (36)

#define OTA_FLAG_START (0x01)
#define OTA_FLAG_GOING (0x02)
#define OTA_FLAG_END (0x03)

#define DEVICE_MODE_NA           (0x00)
#define DEVICE_MODE_LOGISTIC     (0x01)
#define DEVICE_MODE_SLEEP        (0x02)
#define DEVICE_MODE_TRACKING     (0x03)
#define DEVICE_MODE_ACTIVATED    (0x04)

/***************************************************/
#define RL_WAKE_UP_TYPE_GENERAL     (uint8_t)(0)
#define RL_WAKE_UP_TYPE_DAY         (uint8_t)(1)
#define RL_WAKE_UP_TYPE_HOUR        (uint8_t)(2)
#define RL_WAKE_UP_TYPE_HALFHOUR    (uint8_t)(3)
/***************************************************/
#define DEV_SN_LENGTH (8)
#define BOARD_VERSION_LENGTH (4)

typedef union DevInfo_Sct_tag
{
    struct
    {
        uint8_t initialized[2];
        uint8_t dev_sn[DEV_SN_LENGTH];
        uint8_t mb_ver[BOARD_VERSION_LENGTH];
        uint8_t activation_status ;
        uint8_t sw_ver[4];
        uint8_t dev_type;
		uint8_t crc[2] ;
    }structData;
    uint8_t byte[sizeof(structData)];
}DevInfo_Sct_t;


typedef union Config_tag{
    struct
    {
        uint8_t config_ver[2];
        // Voltage alarm parameters
        uint8_t external_low_voltage_threshold;
        uint8_t internal_low_voltage_threshold;
        // Working parameters
        uint8_t wakeup_lasting_time[2];	/* unit: minute */
        uint8_t data_resend_times;
        // Login retry ineval
        uint8_t login_retry_interval; /* unit: minute */
        // Sleep parameters
        uint8_t sleep_time[2];	/* unit: minute */
        uint8_t network_fail_sleep_time;
        // realtime data upload setting
        uint8_t RLTM_upload_interval;
        uint8_t RLTM_record_mileage[2];
        uint8_t RLTM_record_interval;
        uint8_t RLTM_record_turning;
        // tracking mode
        uint8_t log_max_upload_bytes[2];
        // Network config
		uint8_t network_connection_retain_time;
		uint8_t network_connection_port[2];
        uint8_t network_connection_IP[16];
        uint8_t network_connection_APN[20];
        // VIN code
        uint8_t vin[17];
        //secret key
        uint8_t key[16];
		uint8_t crc[2];
    }structData ;    
	uint8_t byte[sizeof(structData)] ; 
}Config_t;
/* vehicle infor head */
typedef union VHCL_Info_tag{
    struct
    {
        uint8_t log_total_number[2] ;
        uint8_t log_sector_read_pointer[2] ;
        uint8_t log_read_sector[2] ;
        uint8_t log_sector_write_pointer[2] ;
        uint8_t log_write_sector[2] ;
    }structData;
    uint8_t byte[sizeof(structData)] ;
}VHCL_Info_t ;
/* vehicle infor body */
typedef enum
{
    vhcl_log_idle,
    vhcl_log_init,
    vhcl_log_in_trans,
}VHCL_log_sate_t;
typedef void(*callBack)(uint8_t res);



typedef union OTAHdr_Sct_tag{
	uint8_t byte[OTA_HEADER_LENGTH] ;
    struct
    {
        uint8_t ota_ctrl;
        uint8_t ota_target_ver[4];
        uint8_t ota_total_bytes[4];
		uint8_t ota_total_package[2];
		uint8_t ota_current_package_index[2];
		uint8_t ota_downloaded_bytes[4];
    }structData;
}OTAHdr_Sct_t ;

/* struct to store last valid GPS data */
typedef struct Last_GPS_Sct_tag
{
    uint8_t longitude[4];
    uint8_t latitude[4];
    uint8_t valid;
}Last_GPS_Sct_t;

/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/
extern void Save_Param(void);
extern bool Load_Param(void);
extern void test_flash(uint32_t address, uint8_t *test_flash_data, uint16_t test_flash_len);

extern bool Get_Config(Config_t *config);
extern bool Set_Config(Config_t config);
extern bool Get_Config_Version(uint8_t *cfv);
extern uint8_t Get_IP_config(uint8_t *ip, uint8_t *port);
extern void Get_config_data(Config_t *config);
//2017.9.16 lihaibin modifys
extern void Get_config_secretKey(uint8_t *pBuff);
extern void Set_config_secretKey(uint8_t *pData);
//end 2017.9.16 lihaibin modifys
extern void Reset_default_config(void);
extern uint8_t Backup_Is_Empty(void);

extern bool Get_Manufacture_Setting(DevInfo_Sct_t *g_devinfo_sct_t);
extern bool Get_Hardware_Version(uint8_t *hv);
extern void Set_Activation_Status(uint8_t status);
extern uint8_t Get_Activation_Status(void);
extern bool Get_Firmware_Version(uint8_t* fv);
extern bool Get_Device_Type(uint8_t *type);

extern bool Set_OTA_Info(OTAHdr_Sct_t g_ota_info_t);
extern bool Get_OTA_Target_Ver(uint16_t *ver);
extern bool Get_OTA_Info(OTAHdr_Sct_t *g_ota_info_t);
extern bool read_ota_data(uint8_t *data);
extern bool WriteOTAData(uint8_t *data , uint16_t len);
extern bool Start_Write_OTA_Data(uint32_t total_bytes);
extern bool Write_OTA_Data(uint8_t *data , uint16_t len);
extern bool End_Write_OTA_Data(uint16_t crc);
extern bool Read_OTA_Data(uint8_t *data , uint16_t len);

//extern bool WriteBatteryData(Battery_log_t g_battery_log_t);
extern uint16_t Get_Battery_Log_Number(void);
extern void Set_Battery_Log_Number(uint16_t);

extern bool Get_GPS_Log_Write_Pointer(uint16_t *write_pointer);
//bool Switch_GPS_Sturcture_data_to_log(gps_data_t gps_data, VHCL_log_t *gps_log);
extern bool Set_GPS_Data_Next_Read_Pointer(uint8_t num);
extern uint8_t Read_GPS_Data(uint8_t *VHCL_log_t, uint8_t num);
//extern bool Write_GPS_Data(VHCL_log_t *g_VHCL_log_t);
extern uint16_t Get_GPS_Data_Total_Number(void);
extern uint8_t Get_Last_GPS_uploaded(void);
void Set_Last_GPS_uploader(uint8_t status);
extern uint32_t sys_get_cur_sec_offset(void);
extern uint32_t sys_get_sec_offset(uint8_t *time_buffer);

extern uint8_t Get_Low_Voltage_Uploaded(void);
extern void Set_Low_Voltage_Uploaded(uint8_t status);
extern uint8_t Read_vhcl_Data(TelmRealtime_t *VHCL_log, uint8_t num);
extern bool Set_vhcl_Data_Next_Read_Pointer(uint8_t num);
//2017.10.30 lihaibin modify
extern void Init_warn_Info(void);
extern uint16_t Get_warn_Data_Total_Number(void);
//static bool Set_warn_Info(VHCL_Info_t *VHCL_Info);

#if 0
extern bool Write_Warn_Data(VHCL_log_t *VHCL_log);
extern uint8_t Read_warn_Data(VHCL_log_t *VHCL_log, uint8_t num);
#endif
bool Set_warn_Data_Next_Read_Pointer(uint8_t num);
extern bool PRM_GetVIN(uint8_t *vin);
extern bool PRM_SetVIN(uint8_t *vin);
void PRM_GetVin(uint8_t *pVin);
bool PRM_GetSn(uint8_t *pSn);
bool PRM_SetSn(uint8_t *pSn);
bool Set_Manufacture_Setting(DevInfo_Sct_t devinfo_sct);
bool Write_vhcl_Data(TelmRealtime_t *VHCL_log);
void Init_vhcl_Info(void);
/********************************************************************
 *
********************************************************************/
#endif //_PRJ_CONFIG_H
