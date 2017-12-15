#ifndef _TELM_PROTOCOL_H_
#define _TELM_PROTOCOL_H_
/**********************************************************************
   Title                                   : TelmApp.h         
                                                                         
   Module Description         : Telematics module 3 file. used for Telematic internal 
                                                function and external module.

   Author                               : 
   
 *********************************************************************/
    
/**********************************************************************
 * Include header files
 *********************************************************************/
#include "prj_config.h"
//#include "TelmApp.h"

 /******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
#define TELM_INFO_LEN_UTC			(4)

#define TELM_INFO_LEN_ODO			(3)
#define TELM_INFO_LEN_TRIP			(3)
#define TELM_INFO_LEN_AIR_FLOW      (5)
#define TELM_INFO_LEN_VIN			(17)

#define TELM_INFO_IN_PACK_MAX_NUM (8)

#define GPS_BACKUP_STATE_IDLE (0x00)
#define GPS_BACKUP_STATE_PUSH (0x01)
#define GPS_BACKUP_STATE_PULL (0x02)

#define TELM_INFO_MAX_LEN 510

#define TELM_BATTERY_NUM        (37)

#define	TELM_INFO_LEN_LOGIN		(30)
#define	TELM_INFO_LEN_LOGOUT	(8)

#define	TELM_INFO_LEN_VEHICLE	(27)
#define	TELM_INFO_LEN_EC_ENGINE	(14)
#define	TELM_INFO_LEN_FUEL_BATT	(19)
#define	TELM_INFO_LEN_ENGINE	(6)
#define	TELM_INFO_LEN_POSITION	(10)
#define	TELM_INFO_LEN_ABS_VAL	(15)
#define	TELM_INFO_LEN_ALARM 	(6)
#define	TELM_INFO_LEN_BATT_VOLT (86)
#define	TELM_INFO_LEN_BATT_TEMP (21)
#define	TELM_INFO_LEN_EXTENDED 	(16)

/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/

typedef struct VHCL_log_tag
{
    uint16_t len;
    uint8_t data[TELM_INFO_MAX_LEN];
}TelmRealtime_t;

typedef enum
{
    TELM_ENTERPRISE_SESION = 0,
    TELM_GB_SESION,
    TELM_SESION_NUM,
}TelmSesionNum_t;

typedef enum
{
    TELM_SESION_INACTIVE,
    TELM_SESION_ACTIVE,
    TELM_SESION_LOGINING,
    TELM_SESION_LOGINED,
    TELM_SESION_LOGOUTING,
    TELM_SESION_LOGOUTED,
}TelmSesionState_t;

typedef enum
{
    TELM_NONE,
	TELM_LOGIN,
	TELM_LOGOUT,
	TELM_REALTIME,
	//TELM_BACKUP,
	TELM_HEARTBEAT,
    TELM_ACK,
    TELM_NUM,
}TELM_DataType_t;

typedef enum
{
    TELM_ACK_OK = 1,
    TELM_ACK_ERR = 2,
    TELM_VIN_REPEAT = 3,
    TELM_CMD = 0xFE,
}TELM_AckType_t;

typedef struct      
{
    uint16_t start;
    uint8_t cmdType;
    uint8_t ackFalg;
    uint8_t VIN[17];
    uint8_t encType;
    uint8_t len[2];
}Telm_data_header_t;     
/* Login info, without charge code */
typedef union Telm_data_Login_tag
{

	struct {
		uint8_t time[6];
		uint8_t daily_index[2];
		uint8_t iccid[20];
		uint8_t charge_sys;
		uint8_t charge_code_len;
	} structData;
	uint8_t byte[TELM_INFO_LEN_LOGIN];    
} Telm_Data_Login;

/* Drive info */
typedef union Telm_data_Logout_tag
{
	struct {
		uint8_t time[6];
		uint8_t daily_index[2];
	} structData;
    uint8_t byte[TELM_INFO_LEN_LOGOUT];
} Telm_Data_Logout;

typedef union Telm_data_Vehicle_tag
{

	struct {
        uint8_t time[6];
        uint8_t msg_id;
		uint8_t state;
		uint8_t charge_state;
        uint8_t run_state;
        uint8_t speed[2];
        uint8_t mileage[4];
        uint8_t voltage[2];
        uint8_t current[2];
        uint8_t soc;
        uint8_t dc_state;
        uint8_t shift;
        uint8_t insulate_res[2];
        uint8_t accel_pedal;
        uint8_t brake_pedal;
	} structData;
    uint8_t byte[TELM_INFO_LEN_VEHICLE];
} Telm_Data_Vehicle;

typedef union Telm_data_Ec_Engine_tag
{

	struct {
        uint8_t msg_id;
		uint8_t engine_num;
		uint8_t index;
        uint8_t state;
        uint8_t ctl_temp;
        uint8_t rpm[2];
        uint8_t torque[2];
        uint8_t temp;
        uint8_t ctl_vol[2];
        uint8_t ctl_current[2];
    } structData;
    uint8_t byte[TELM_INFO_LEN_EC_ENGINE];
} Telm_Data_Ec_Engine;

typedef union Telm_data_Fuel_Batt_tag
{

	struct {
        uint8_t msg_id;
		uint8_t voltage[2];
		uint8_t current[2];
        uint8_t rate[2];
        uint8_t tprobe_num[2];
        uint8_t h_sys_high_temp[2];
        uint8_t h_sys_high_temp_probe;
        uint8_t h_sys_high_concentration[2];
        uint8_t h_sys_high_concentration_probe;
        uint8_t h_sys_high_pressure[2];
        uint8_t h_sys_high_pressure_probe;
        uint8_t dc_state;
    } structData;
	uint8_t byte[TELM_INFO_LEN_FUEL_BATT];
} Telm_Data_Fuel_Batt;

typedef union Telm_data_Engine_tag
{

	struct {
        uint8_t msg_id;
		uint8_t state;
		uint8_t rpm[2];
        uint8_t rate[2];
    } structData;
    uint8_t byte[TELM_INFO_LEN_ENGINE];
} Telm_Data_Engine;

typedef union Telm_data_Position_tag
{
	struct {
        uint8_t msg_id;
		uint8_t state;
		uint8_t longitude[4];
        uint8_t latitude[4];
    } structData;
    uint8_t byte[TELM_INFO_LEN_POSITION];
} Telm_Data_Position;

typedef union Telm_data_Abs_Val_tag
{
	struct {
        uint8_t msg_id;
		uint8_t max_vol_sys_code;
		uint8_t max_vol_code;
        uint8_t max_vol[2];
		uint8_t min_vol_sys_code;
		uint8_t min_vol_code;
        uint8_t min_vol[2];
		uint8_t max_temp_sys_code;
		uint8_t max_temp_code;
        uint8_t max_temp;
		uint8_t min_temp_sys_code;
		uint8_t min_temp_code;
        uint8_t min_temp;
    } structData;
    uint8_t byte[TELM_INFO_LEN_ABS_VAL];
} Telm_Data_Abs_Val;

typedef union Telm_data_Alarm_tag
{
	struct {
        uint8_t msg_id;
		uint8_t alarm_level;
        union
        {
            uint8_t alarm_flag[4];
            struct
            {
                uint8_t temp_diff : 1;
                uint8_t batt_over_temp : 1;
                uint8_t batt_over_vol : 1;
                uint8_t batt_under_vol : 1;
                uint8_t soc_under : 1;
                uint8_t batt_cell_over_vol : 1;
                uint8_t batt_cell_under_vol : 1;
                uint8_t soc_over : 1;
                uint8_t soc_leap : 1;
                uint8_t batt_mismatch : 1;
                uint8_t batt_cell_disaccord : 1;
                uint8_t isolation : 1;
                uint8_t dc_temp : 1;
                uint8_t brake : 1;
                uint8_t dc_sta : 1;
                uint8_t motor_ctrl_temp : 1;
                uint8_t HVIL : 1;
                uint8_t motor_temp : 1;
                uint8_t batt_over_charge : 1;
            };
        };
    } structData;
    uint8_t byte[TELM_INFO_LEN_ALARM];
} Telm_Data_Alarm;


typedef union Telm_data_Batt_Volt_tag
{
	struct {
        uint8_t msg_id;
		uint8_t batt_num;
        uint8_t sub_sys_id;
        uint8_t batt_volt[2];
        uint8_t batt_current[2];
        uint8_t single_batt_num[2];
        uint8_t frame_start_id[2];
        uint8_t frame_batt_total;
        uint8_t frame_batt_volt[BATTERY_CELL_CNT*2];
    } structData;
    uint8_t byte[TELM_INFO_LEN_BATT_VOLT];
} Telm_Data_Batt_Volt;

typedef union Telm_data_Batt_Temp_tag
{
	struct {
        uint8_t msg_id;
		uint8_t batt_num;
        uint8_t sub_sys_id;
        uint8_t temp_probe_num[2];
        uint8_t frame_batt_temp[BATTERY_CELL_TMEMP_CNT];
    } structData;
    uint8_t byte[TELM_INFO_LEN_BATT_TEMP];
} Telm_Data_Batt_Temp;

typedef union Telm_data_Extended_tag
{
	struct {
        uint8_t msg_id;
		uint8_t length[2];
        uint8_t drive_mileage[2];
        uint8_t charge_finish_time[2];
        uint8_t remain_power;
        uint8_t power_percent;
        uint8_t acc_status;
        uint8_t charge_pile_status;
        uint8_t tie_fr;
        uint8_t tie_fl;
        uint8_t tie_rr;
        uint8_t tie_rl;
        uint8_t window_status;
    } structData;
    uint8_t byte[TELM_INFO_LEN_EXTENDED];
} Telm_Data_Extended;


typedef struct InitFlag_Sct_tag
{
    uint8_t init_flag[4];
}InitFlag_Sct_t;

typedef enum Telm_Info_Tag
{
    TELM_INFO_NONE,
	TELM_INFO_LOGIN,
	TELM_INFO_LOGOUT,
	TELM_INFO_VEHICLE_DATA,
	TELM_INFO_EC_ENGINE_DATA,
	TELM_INFO_FUEL_BATT_DATA,
	TELM_INFO_ENGINE_DATA,
	TELM_INFO_GPS_DATA,
	TELM_INFO_ABS_DATA,
	TELM_INFO_ALARM_DATA,
	TELM_INFO_RESP,
	TELM_INFO_BATT_VOLT,
	TELM_INFO_BATT_TEMP,
	TELM_INFO_EXTENDED,
    TELM_INFO_HEARTBEAT,
    TELM_INFO_NUM
}Telm_info;

typedef enum Msg_Encrypt_Type_tag
{
	MSG_ENCRYPT_NONE=1,
	MSG_ENCRYPT_RSA,
	MSG_ENCRYPT_AES,
	MSG_ENCRYPT_ERROR=0xFE,
	MSG_ENCRYPT_INVALID=0xFF,
}Msg_Encrypt_Type_t;
/* struct to store the info to be uploaded to server */
typedef struct Telm_Upload_Info_tag
{
    uint8_t utcTime[TELM_INFO_LEN_UTC];	/* UTC Time. the second offset from 2000/01/01 00:00:00, 	*/
    uint8_t vin[TELM_INFO_LEN_VIN];		/* Device ID 								*/
    uint8_t activated; /* device activated flag */
    uint8_t multiple; // If send to multiple address
    uint8_t cmd_id;
    uint8_t ack_flag;
}Telm_Upload_Info;

extern Telm_Upload_Info lastInfo;

/* telmatics info encode function type */
typedef unsigned char (*Telm_Info_Encode_Fun)(uint8_t *const encoded, uint16_t bufSize);

typedef enum AT_connect_id_tag
{
    AT_CONNECT_ID_MAIN=0,
    AT_CONNECT_ID_GB,
}AT_connect_id_t;

#define TELM_HEARTBEAT_MAX_LEN (sizeof(Telm_data_header_t) + 1)
#define TELM_REALTIME_MAX_LEN (uint16_t)512
#define TELM_LOG_MAX_LEN (uint16_t)256

/*===========================================================================*\
 * Function Prototypes for Functions with File Level Scope
\*===========================================================================*/
extern void vTelmProt_ParseIpData(uint8_t *pData, uint16_t len, uint8_t sesionNum);
uint16_t TelmProt_Encode(uint8_t *pBuffer, 
                                   TELM_DataType_t dataType, 
                                   Msg_Encrypt_Type_t eccryptType,
                                   uint16_t bufferSzie);
uint16_t TelmProt_Encode_Vehicle(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_GPS(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_EcEngine(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_Engine(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_AbsVal(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_Alarm(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_FuelBatt(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_EncKey(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_Batt_Volt(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_Batt_Temp(uint8_t* const encoded, uint16_t buffsize);
uint16_t TelmProt_Encode_Extended(uint8_t* const encoded, uint16_t buffsize);

extern void	vTelmProt_informData(void);
extern uint8_t seach_pid_support_list(uint8_t pid_idx);
//extern uint16_t vTelmProt_Encode_Package(Telm_Upload_Info_Request const * const req,Telm_Upload_Info const * const info, uint8_t *data);

extern void Clear_OTASector(void);
extern void Load_OTAHeader(void);
extern void Telm_Test_Write(void);
extern void speed_add_new(uint8_t speed);
extern uint8_t check_speed_exception(uint8_t *high, uint8_t *low);
extern uint8_t speed_check_func(void);

extern void gps_write_nv(void);
extern uint8_t get_nv_gpsdata(uint8_t *data);
extern void clear_nv_gpsdata(uint8_t sector);
extern void set_gps_upload_backup(uint8_t enable);
extern uint8_t gps_upload_status(void);

extern uint8_t check_init_flag(void);
extern uint8_t set_init_flag(void);
extern uint8_t set_dev_id(uint8_t *data);
extern void write_init_config(void);
extern void clear_nv_config(void);
extern uint8_t get_sys_config(uint8_t *config);

extern uint8_t get_dev_info(uint8_t *info);
extern void set_activated(void);
extern void set_deactivate(void);
extern uint8_t get_activated(void);

extern uint8_t batt_volt_nv_check_write_timer(void);
extern void batt_volt_nv_read_info(void);
extern void batt_volt_nv_check_tx(void);
extern void batt_volt_nv_erase(void);
extern void TelmProt_Send_GTimes(void);

extern uint8_t gps_backup_empty(void);
extern uint8_t get_gps_upload_freq(void);
extern void load_from_nv(void);
extern void load_from_rom(void);
extern uint8_t set_mbsn(uint8_t *data);
extern uint8_t gps_buffer_empty(void);

extern uint8_t set_factory_test(uint8_t pass);
extern void set_car_crash(uint8_t flag);

extern void set_battery_status(uint8_t status);
extern uint8_t get_battery_status(void);
extern void set_check_engine_light(uint8_t status);
extern void set_engine_startup(uint8_t status);
extern void set_go_sleep(uint8_t status);

extern void save_gps_buffer(void);

extern uint8_t TelmProt_Is_New_OTA(void);
extern void TelmProt_Set_New_OTA(uint8_t status);

extern void store_last_pos(uint8_t *data);
extern void write_flash_last_pos(void);

extern void save_current_except(void);
extern uint8_t except_backup_empty(void);

extern uint8_t eng_on_off_nv_empty(void);
extern uint8_t eng_on_off_nv_get_status(void);
extern void eng_on_off_nv_set_status(uint8_t val);

extern void eng_on_off_ram_write(int16_t data);
extern uint8_t eng_on_off_ram_empty(void);
extern uint8_t eng_on_off_ram_get_status(void);
extern void eng_on_off_ram_set_status(uint8_t val);
extern void eng_on_off_ram_copy_to_nv(void);

extern uint8_t TelmProt_Get_Data_MiningEnb(void);
extern uint8_t TelmProt_Get_G_LogEnb(void);
extern void TelmProt_Sleep_NV_Write(void);
extern void Save_RTC_Ref(uint32_t counter);
extern void Read_RTC_Ref(uint32_t *data);

extern void TelmProt_Travel_Summary_Record(void);
extern void TelmProt_Clear_Travel_Summary(void);
TelmSesionState_t TelmProt_getSesionState(TelmSesionNum_t sesionNum);
bool TelmProt_setSesionState(TelmSesionNum_t sesionNum, TelmSesionState_t state);
bool TelmPort_RealtimeUpload(TelmSesionNum_t sesoinNum);
bool TelmPort_LogoutUpload(TelmSesionNum_t sesoinNum);
bool TelmPort_LoginUpload(TelmSesionNum_t sesoinNum);
bool TelmProt_backupData(TelmRealtime_t *pBuffer);
bool TelmProt_getBackupData(TelmRealtime_t *pBuffer);
bool TelmProt_preGetBackupData(void);
void TelmProt_connectHandler(bool state, uint8_t channel);
uint8_t TelmProtGetHeartbeatCnt(void);
void TelmProtHeartbeatsThrob(void);
#endif

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
