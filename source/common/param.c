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

#include "standard.h"
#include "prj_config.h"
#include <time.h>
#include <string.h>
#include "ATProtocol.h"
#include "rtc.h"

#include "Debug.h"

#define FLASH_SPI_SECTORSIZE 			FLASH_SECTOR_SIZE
#define VHCL_LOG_NUMBER_PER_SECOTR      (FLASH_SPI_SECTORSIZE/(sizeof(TelmRealtime_t)))
#define VHCL_LOG_MAX_SECTOR_NUM         ((FLASH_RLDT_LOG_END-FLASH_RLDT_LOG_OFFSET)/FLASH_SPI_SECTORSIZE)
#define MAX_VHCL_LOG_NUMBER             (VHCL_LOG_NUMBER_PER_SECOTR*VHCL_LOG_NUMBER_PER_SECOTR)
#define OTADATAPERPACKAGE				(1 * 1024)
#define FLASHWRITEREPEATTIMES 			5
/* INITIALIZE DATA */
#define INITIALIZE_DATA							(0xa5a5)
#define START_OTA_FLAG 							(0x55AA)
#define INIT_FLAG_DATA 							INITIALIZE_DATA

#define OTA_MAX_SIZE (256 * 1024)

#define RTC_REF_DELTA 600

/************************ Global vatiables *****************************/
typedef struct
{
    uint16_t log_total_number;
    uint16_t log_sector_read_ptr;
    uint16_t log_read_sector;
    uint16_t log_sector_write_ptr;
    uint16_t log_write_sector;
}data_backup_info_t;

static uint8_t default_devinfo[]={0xa5,0xa5,
                                0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x02,
                                0x04,0x00,0x00,0x00,0x01,0x01};
static uint8_t default_config[]={10u, 0u,
                                // Voltage alarm parameters
                                115u, 30u,
                                // Working parameters
                                0x2Cu, 0x01u, 3u,
                                // login retry interval
                                0x1E, //30 min
                                // Sleep parameters
                                0x01u, 0x00u, 0x01u,
                                // realtime data upload setting
                                10u, 0x01u, 0u, 10u, 1u, 
                                //tracking mode
                                0u, 0u,
                                //Network config    
                                30u, //network_connection_retain_time
                                0x75u, 0x50u, //network_connection_port                             
                                 '5', '9', '.', '4', '4', '.', '4', '3', '.', '2', '3', '4', 0x00,0x00,0x00,0x00,//ip
                                //'1', '0', '1', '.', '3', '7', '.', '8', '7', '.', '6', '5', 0x00,0x00,0x00,0x00,//ip
                                //'c','m','n','e','t',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,//apn
                                'c','m','i','o','t',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                //VIN code
                                '3','5','3','8','1','6','0','5','2','7','8','0','2','8','0','N','E',
                                //secret key
                                //'h','e','n','g','e',0,0,0,0,0,0,0,0,0,0,0,
								//'l','i','a','o','n','i','n','g',0,0,0,0,0,0,0,0,
                                '1','2','3','4','5','6','7','8',0,0,0,0,0,0,0,0,
                                //'1','2','3','4','5','6',0,0,0,0,0,0,0,0,0,0,
                                //crc
                                0x00,0x00,
                                };
static DevInfo_Sct_t g_devinfo_sct;
static Config_t g_config_data;
//static VHCL_Info_t 		g_backup_info;
static data_backup_info_t g_vehicle_info;
static data_backup_info_t g_warning_info;
//static OTAHdr_Sct_t g_ota_info;
//static uint8_t last_gps_uploaded=1;
static uint8_t last_gps_uploaded=0;
static uint8_t low_voltage_uploaded=0;

//static uint32_t ref_rtc = 0; // Reference clock counter
static uint8_t rtc_inited = 0;
static const uint16_t tcom_month_tab[2][12]={ 
   {0,31,59,90,120,151,181,212,243,273,304,334},//No leap year
   {0,31,60,91,121,152,182,213,244,274,305,335}, //Leap year
};

/***************** Internal functions ****************/
//static bool Set_Manufacture_Setting(DevInfo_Sct_t devinfo_sct);
static void Init_Param_In_Flash(void);
static void Get_default_config(Config_t *config);

static uint16_t ArrayToUint16(uint8_t *bytes)
{
    uint16_t data ;

    if(bytes == NULL)
    {
        return 0 ;
    }
    data = bytes[0];
    data <<= 8;
    data += bytes[1] ;
    return data ;
}

static bool Uint16ToArray(uint16_t data , uint8_t *array)
{
    if (array == NULL)
        return false ;
    *array = (uint8_t)(data >> 8) & 0xff;
    *(array+1) = data & 0xff;
    return true ;
}


static void Init_Manufacture_Setting(void)
{
    spi_flash_erase_sector(FLASH_MANUFACTURE_SETTING_OFFSET,FLASH_SECTOR_SIZE);
    DEBUG(DEBUG_MEDIUM, "[PARAM]:Manufacture setting Data initialized!\n\r");
}

// Load setting from flash, address 0
bool Get_Manufacture_Setting(DevInfo_Sct_t *devinfo_sct)
{
    uint16_t crc = 0;

    if(devinfo_sct == NULL)
    {
        return false ;
    }
//    spi_flash_erase_all();
    spi_flash_read(FLASH_MANUFACTURE_SETTING_OFFSET, sizeof(DevInfo_Sct_t), devinfo_sct->byte);
    crc = crc_ccitt(crc, devinfo_sct->byte, sizeof(DevInfo_Sct_t) - 2) ;

    if ((((devinfo_sct->structData.initialized[0] << 8) + \
        devinfo_sct->structData.initialized[1]) != INITIALIZE_DATA) ||
        (crc != ((devinfo_sct->structData.crc[0] << 8) + devinfo_sct->structData.crc[1]))
        )    
    {
        DEBUG(DEBUG_LOW, "[PARAM] Set manufacture...\r\n");
        // read flash error, use default setting
        memcpy(g_devinfo_sct.byte,default_devinfo,sizeof(DevInfo_Sct_t) - 2);
        
        //ATProt_Get_Imei(g_devinfo_sct.structData.dev_sn);

        crc = crc_ccitt(crc, g_devinfo_sct.byte, sizeof(DevInfo_Sct_t) - 2) ;
        g_devinfo_sct.structData.crc[0]=(crc>>8) & 0xff;
        g_devinfo_sct.structData.crc[1]=(crc) & 0xff;
        Init_Param_In_Flash();
        Set_Manufacture_Setting(g_devinfo_sct);
        memcpy(devinfo_sct->byte,g_devinfo_sct.byte,sizeof(DevInfo_Sct_t));
        Get_default_config(&g_config_data);

        return false ;
    }
    return true ;
}

bool Set_Manufacture_Setting(DevInfo_Sct_t devinfo_sct)
{
    uint16_t crc = 0;
//    int count = 0;
    DevInfo_Sct_t tmp_g_devinfo_sct;
    crc = crc_ccitt(crc, devinfo_sct.byte, sizeof(DevInfo_Sct_t) - 2);
    devinfo_sct.structData.crc[0] = crc >> 8;
    devinfo_sct.structData.crc[1] = crc >> 0;

//    while(count < FLASHWRITEREPEATTIMES)
    {

        Init_vhcl_Info();
        OS_Sleep(100);
        spi_flash_erase_sector(FLASH_MANUFACTURE_SETTING_OFFSET,FLASH_SECTOR_SIZE);
        OS_Sleep(100);

        spi_flash_write(FLASH_MANUFACTURE_SETTING_OFFSET,
                            sizeof(DevInfo_Sct_t),
                            devinfo_sct.byte);
        OS_Sleep(100);
        spi_flash_read(FLASH_MANUFACTURE_SETTING_OFFSET,
                            sizeof(DevInfo_Sct_t),
                            tmp_g_devinfo_sct.byte);
        if(memcmp(devinfo_sct.byte, tmp_g_devinfo_sct.byte, sizeof(DevInfo_Sct_t))==0)
        {
            return true ;
        }
//        count++;
    }
    
    return false ;
}


// Get hardware version
extern bool Get_Hardware_Version(uint8_t *hv)
{
    if(hv == NULL)
    {
        return false ;
    }

    memcpy(hv, g_devinfo_sct.structData.mb_ver, 4) ;
    return true ;
}

extern void Set_Activation_Status(uint8_t status)
{
    g_devinfo_sct.structData.activation_status = status;
    Set_Manufacture_Setting(g_devinfo_sct);
}

// Activation status. 0x04 = activated
extern uint8_t Get_Activation_Status(void)
{
    return g_devinfo_sct.structData.activation_status;
}

// Get firmware version
extern bool Get_Firmware_Version(uint8_t* fv)
{
    if(fv == NULL)
    {
        return false;
    }

    memcpy(fv, g_devinfo_sct.structData.sw_ver, 4);
    return true;
}

// Get device type
extern bool Get_Device_Type(uint8_t *type)
{
    if(type == NULL)
    {    
        return false ;
    }
    *type = g_devinfo_sct.structData.dev_type ;
    return true ;
}
/************************Manufacture Settings*****************************/

/************************Config Settings**********************************/

static void Init_Config(void)
{
    uint16_t crc = 0 ;
    crc = crc_ccitt(crc , g_config_data.byte , sizeof(Config_t) - 2) ;
    g_config_data.structData.crc[0] = crc >> 8 ;
    g_config_data.structData.crc[1] = crc >> 0 ;
    spi_flash_erase_sector(FLASH_CONFIG_OFFSET,FLASH_SECTOR_SIZE);
    spi_flash_write(FLASH_CONFIG_OFFSET, sizeof(Config_t), g_config_data.byte);

    DEBUG(DEBUG_MEDIUM, "[PARAM]:Config Data initialized!\n\r");
}

// Get default config from RAM
static void Get_default_config(Config_t *config)
{
#if 1
    uint16_t crc = 0 ;
    memcpy(config->byte,default_config,sizeof(Config_t)-2);
    crc = crc_ccitt(crc , config->byte , sizeof(Config_t) - 2) ;
    config->structData.crc[0]=(crc>>8) & 0xff;
    config->structData.crc[1]=(crc) & 0xff;
    Set_Config(*config);
#endif
}
// Read config data from flash
bool Get_Config(Config_t *config)
{
    uint16_t crc = 0;

    if(config == NULL)
    {
        return false;
    }

    spi_flash_read(FLASH_CONFIG_OFFSET, sizeof(Config_t), config->byte);

    crc = crc_ccitt(crc, config->byte, sizeof(Config_t)-2);

    if(crc != ((config->structData.crc[0] << 8) + config->structData.crc[1]))
    {
        DEBUG(DEBUG_HIGH, "Get_Config\r\n");
        Get_default_config(config);
        return false;
    }
#ifdef USE_DEFAULT_CONFIG
    spi_flash_erase_all();
    Get_default_config(config);
#endif
    return true;
}

// Save config data to flash
bool Set_Config(Config_t config)
{
    uint16_t crc = 0;
    Config_t tmp_config;

    crc = crc_ccitt(crc , config.byte , sizeof(Config_t) - 2);

    config.structData.crc[0] = crc >> 8;
    config.structData.crc[1] = crc;

    memcpy(g_config_data.byte, config.byte, sizeof(Config_t) - 2);
    {
        if(0 != spi_flash_erase_sector(FLASH_CONFIG_OFFSET, FLASH_SECTOR_SIZE))
        {
            return false;
        }
        OS_Sleep(100);
        if(0 != spi_flash_write(FLASH_CONFIG_OFFSET, sizeof(Config_t), config.byte))
        {
            return false;
        }
        OS_Sleep(100);
        if(0 != spi_flash_read(FLASH_CONFIG_OFFSET, sizeof(Config_t), tmp_config.byte))
        {
            return false;
        }
        if(memcmp(config.byte, tmp_config.byte, sizeof(Config_t))==0)
        {
            return true;
        }
    }
    return false ;
}

// Get config version
extern bool Get_Config_Version(uint8_t *cfv)
{
    if(cfv == NULL)
    {
        return false ;
    }

    memcpy(cfv, g_config_data.structData.config_ver, 2);
    return true;
}

extern uint8_t Get_IP_config(uint8_t *ip, uint8_t *port)
{
    uint8_t rt=0;
    uint8_t port_tmp[2];
    rt=strlen((const char*)g_config_data.structData.network_connection_IP);
    strcpy((char *)ip,(const char*)g_config_data.structData.network_connection_IP);
    port_tmp[0]=g_config_data.structData.network_connection_port[1];
    port_tmp[1]=g_config_data.structData.network_connection_port[0];
    DECtoStr(port, port_tmp, 2);
    
    return rt;
}

extern void Get_config_data(Config_t *config)
{
    memcpy(config->byte, g_config_data.byte, sizeof(Config_t));
}
extern void Set_config_secretKey(uint8_t *pData)
{
    if(pData)
    {
        memcpy(g_config_data.structData.key, pData, 16);
    }
}
extern void Get_config_secretKey(uint8_t *pBuff)
{
    if(pBuff)
    {
        memcpy(pBuff, g_config_data.structData.key, 16);
    }
/*
    for(;;)
    {
        //should not entry here
    }
*/
}
extern void Reset_default_config(void)
{
    DEBUG(DEBUG_HIGH, "Reset_default_config\r\n");
    Get_default_config(&g_config_data);
}

/************************Config Settings end******************************/

/************************GPS Info*****************************************/

/*******************************************************************************
*    Function:  sys_get_sec_offset
*
*  Parameters:  time_buffer is the string like "20120304,123456"
*     Returns:  None
* Description:  get system time, convert to second.
*******************************************************************************/
uint32_t sys_get_sec_offset(uint8_t *time_buffer)
{
    uint8_t year_offset; // current day offset from 2000 to the start of current year
    uint16_t day_offset; // current day offset from the start of current year

    uint32_t cur_day_offset;//the second offset from 2000/01/01 to  the start of the current  day 2012/03/04
    uint32_t cur_sec_offset; //the second offset from the 00:00:00  to the current time 12:34:56
    uint16_t t;

//    if ((time_buffer == NULL) || (time_buffer[0]!='2') || (time_buffer[1]!='0'))
    if (time_buffer == NULL)
    {
        return 0;
    }

    /* Check the GPS data and  */
    if (time_buffer[0]=='2')
    {
        year_offset = (time_buffer[2] - '0') * 10 + time_buffer[3] - '0';
        if (year_offset%4)
        {
            day_offset = tcom_month_tab[0][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1 ];
        }
        else
        {
            day_offset = tcom_month_tab[1][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1];
        }
        day_offset += (time_buffer[6] - '0') * 10 + time_buffer[7] - '0' - 1;

        cur_day_offset = (year_offset*365ul + ((year_offset-1)/4 + 1) + day_offset)*86400ul;
        cur_sec_offset = ((time_buffer[8] - '0')*10+ time_buffer[9] - '0') * 3600ul + \
              ((time_buffer[10] - '0')*10+ time_buffer[11] - '0')* 60ul + \
              ((time_buffer[12] - '0')*10+ time_buffer[13] - '0');

        cur_sec_offset += cur_day_offset;
        for(t=1970;t<2000;t++) //
        {
            if(Is_Leap_Year(t))
            {
                cur_sec_offset+=31622400; //
            }
            else
            {
                cur_sec_offset+=31536000; //
            }
        }
        // Then plus the RTC adjustment data
    }
    else
    {
        year_offset = ((time_buffer[2] - '0') * 10 + time_buffer[3] - '0')-70;
        if (year_offset%4)
        {
            day_offset = tcom_month_tab[0][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1 ];
        }
        else
        {
            day_offset = tcom_month_tab[1][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1];
        }
        day_offset += (time_buffer[6] - '0') * 10 + time_buffer[7] - '0' - 1;

        if (year_offset>0)
        {
            cur_day_offset = (year_offset*365ul + ((year_offset-1)/4 + 1) + day_offset)*86400ul;
        }
        else
        {
            cur_day_offset = day_offset*86400ul;
        }
        cur_sec_offset = ((time_buffer[8] - '0')*10+ time_buffer[9] - '0') * 3600ul + \
              ((time_buffer[10] - '0')*10+ time_buffer[11] - '0')* 60ul + \
              ((time_buffer[12] - '0')*10+ time_buffer[13] - '0');

        cur_sec_offset += cur_day_offset;
    }
    return cur_sec_offset;
}

/*******************************************************************************
*    Function:  sys_get_cur_sec_offset
*
*  Parameters:  
*     Returns:  None
* Description:  get current second offset from system.
*******************************************************************************/
extern uint32_t sys_get_cur_sec_offset(void)
{
   uint8_t curGpsUtc[15];

   /* Get GPS data from gps modual */
   vGps_Get_Gps_Utc(curGpsUtc);

   if (curGpsUtc[0] == true)
   {
      /* Get GPS data from gps modual */
        if (rtc_inited == 0)
        {
            rtc_datetime_t utc = {0};
            utc.year = (((curGpsUtc[1] - '0') * 1000) + 
                         ((curGpsUtc[2] - '0') * 100) +
                             ((curGpsUtc[3] - '0') * 10) +
                                 (curGpsUtc[4] - '0'));
            utc.month = (((curGpsUtc[5] - '0') * 10) + (curGpsUtc[6] - '0'));
            utc.day = (((curGpsUtc[7] - '0') * 10) + (curGpsUtc[8] - '0'));
            utc.hour = (((curGpsUtc[9] - '0') * 10) + (curGpsUtc[10] - '0'));
            utc.minute = (((curGpsUtc[11] - '0') * 10) + (curGpsUtc[12] - '0'));
            utc.second = (((curGpsUtc[13] - '0') * 10) + (curGpsUtc[14] - '0'));

            // current UTC time plus 8 hours for GMT +8
            RTC_SetTimeCount(RTC_ConvertDatetimeToSeconds(&utc)+28800);
            rtc_inited = 1;
      }
      return sys_get_sec_offset(&curGpsUtc[1]);   
   }
   else //if (0 != rtc_inited)
   {
      /* Get RTC from Local system time */
      uint8_t utc_buff[20];
      if (0 == RTC_GetTime(utc_buff))
      {
         return sys_get_sec_offset(utc_buff);
      }
   }
   return 0;
}

void Init_vhcl_Info(void)
{
    spi_flash_erase_sector(FLASH_RLDT_INFO_OFFSET, FLASH_SECTOR_SIZE);
    memset(&g_vehicle_info, 0 , sizeof(data_backup_info_t));
}

static bool Get_backupl_Info(void)
{
    VHCL_Info_t vhcl_info;

    spi_flash_read(FLASH_RLDT_INFO_OFFSET, sizeof(VHCL_Info_t), vhcl_info.byte);

    if ((vhcl_info.byte[0]==0xff) && (vhcl_info.byte[1]==0xff))
    {
        memset(&g_vehicle_info,0,sizeof(data_backup_info_t));
        return false;
    }
    g_vehicle_info.log_read_sector = ArrayToUint16(vhcl_info.structData.log_read_sector);
    g_vehicle_info.log_sector_read_ptr = ArrayToUint16(vhcl_info.structData.log_sector_read_pointer);
    g_vehicle_info.log_write_sector = ArrayToUint16(vhcl_info.structData.log_write_sector);
    g_vehicle_info.log_sector_write_ptr = ArrayToUint16(vhcl_info.structData.log_sector_write_pointer);
    g_vehicle_info.log_total_number = ArrayToUint16(vhcl_info.structData.log_total_number);

    if(g_vehicle_info.log_read_sector > 128)
    {
        g_vehicle_info.log_read_sector=0;
        g_vehicle_info.log_sector_read_ptr=0;
        return false;		/* 512K , max: 128sector */
    }
    if(g_vehicle_info.log_sector_read_ptr > VHCL_LOG_NUMBER_PER_SECOTR)
    {
        g_vehicle_info.log_sector_read_ptr=0;
        return false;
    }
    if(g_vehicle_info.log_write_sector > 128)
    {
        g_vehicle_info.log_write_sector=0;
        g_vehicle_info.log_sector_write_ptr=0;
        return false;		/* 512K , max: 128sector */
    }
    if(g_vehicle_info.log_sector_write_ptr >= VHCL_LOG_NUMBER_PER_SECOTR)
    {
        g_vehicle_info.log_sector_write_ptr=0;
        return false;
    }
    if(g_vehicle_info.log_total_number > MAX_VHCL_LOG_NUMBER)
    {
        g_vehicle_info.log_total_number=0;
        return false;
    }

    return true;
}

uint16_t Get_vhcl_Data_Total_Number(void)
{
    return g_vehicle_info.log_total_number;
}

/*******************************************************************************
*    Function:  Set_vhcl_Info
*
*  Parameters:  
*     Returns:  None
* Description:  Write record's information to flash memery
*******************************************************************************/
static bool Set_vhcl_Info(VHCL_Info_t *VHCL_Info)
{
//    int count = 0;
    VHCL_Info_t tmp_g_backup_info;
    if (VHCL_Info->structData.log_total_number == 0)
    {
        return false;
    }
//    while(count < FLASHWRITEREPEATTIMES)
    {
        if(0 != spi_flash_erase_sector(FLASH_RLDT_INFO_OFFSET,FLASH_SECTOR_SIZE))
        {
            return false;
        }
        if(0 != spi_flash_write(FLASH_RLDT_INFO_OFFSET, sizeof(VHCL_Info_t), VHCL_Info->byte))
        {
            return false;
        }
        if(0 != spi_flash_read(FLASH_RLDT_INFO_OFFSET, sizeof(VHCL_Info_t), tmp_g_backup_info.byte))
        {
            return false;
        }
        if(0==(memcmp(VHCL_Info->byte, tmp_g_backup_info.byte, sizeof(VHCL_Info_t))))
        {
            return true;
        }
 //       count++;
    }
    return false;
}

// write vehicle data to flash
bool Write_vhcl_Data(TelmRealtime_t *VHCL_log)
{
    /* arrive new sector */
    if(g_vehicle_info.log_sector_write_ptr == 0)
    {
        spi_flash_erase_sector(FLASH_RLDT_LOG_OFFSET + g_vehicle_info.log_write_sector * FLASH_SPI_SECTORSIZE,
                               FLASH_SECTOR_SIZE);
        if (g_vehicle_info.log_total_number==MAX_VHCL_LOG_NUMBER)
        {
            g_vehicle_info.log_total_number-=VHCL_LOG_NUMBER_PER_SECOTR;
            if (g_vehicle_info.log_write_sector>=(VHCL_LOG_MAX_SECTOR_NUM-1))
            {
                g_vehicle_info.log_read_sector = 0;
            }
            else
            {
                g_vehicle_info.log_read_sector = g_vehicle_info.log_write_sector+1;
            }
            g_vehicle_info.log_sector_read_ptr = 0;
        }
    }

    spi_flash_write(FLASH_RLDT_LOG_OFFSET + g_vehicle_info.log_write_sector * FLASH_SPI_SECTORSIZE + \
                    g_vehicle_info.log_sector_write_ptr * sizeof(TelmRealtime_t), \
                    sizeof(TelmRealtime_t),
                    VHCL_log);
    g_vehicle_info.log_sector_write_ptr++;
    /* if write pointer reach end of flash area , reset to 0 */
    if(g_vehicle_info.log_sector_write_ptr >= VHCL_LOG_NUMBER_PER_SECOTR)
    {
        g_vehicle_info.log_sector_write_ptr = 0;
        g_vehicle_info.log_write_sector ++;
    }
    if(g_vehicle_info.log_write_sector >= VHCL_LOG_MAX_SECTOR_NUM)
    {
        g_vehicle_info.log_write_sector = 0;
        g_vehicle_info.log_sector_write_ptr = 0;
    }

    if(g_vehicle_info.log_total_number < MAX_VHCL_LOG_NUMBER)
        g_vehicle_info.log_total_number ++;
    else 
    {
        /* read pointer = write pointer */
        g_vehicle_info.log_total_number = MAX_VHCL_LOG_NUMBER;
        g_vehicle_info.log_read_sector = g_vehicle_info.log_write_sector;
        g_vehicle_info.log_sector_read_ptr = g_vehicle_info.log_sector_write_ptr;
    }
    return true ;
}

// set read pointer to the next N
// num shall not greater than (GPS_LOG_NUMBER_PER_SECOTR-log_sector_read_ptr)
bool Set_vhcl_Data_Next_Read_Pointer(uint8_t num)
{
    if((g_vehicle_info.log_total_number == 0) || (g_vehicle_info.log_total_number <= num))
    {
        memset(&g_vehicle_info, 0, sizeof(data_backup_info_t));
        return false;
    }
    g_vehicle_info.log_sector_read_ptr += num;
    if(g_vehicle_info.log_sector_read_ptr >= VHCL_LOG_NUMBER_PER_SECOTR)
    {
        g_vehicle_info.log_sector_read_ptr = 0;
        g_vehicle_info.log_read_sector++;
    }
    if(g_vehicle_info.log_read_sector >= VHCL_LOG_MAX_SECTOR_NUM)
    {
        g_vehicle_info.log_read_sector = 0;
        g_vehicle_info.log_sector_read_ptr = 0;
        return true;
    }
    if(g_vehicle_info.log_total_number > 0)
    {
        g_vehicle_info.log_total_number -= num;
        if (g_vehicle_info.log_total_number == 0)
        {
            g_vehicle_info.log_read_sector = 0;
            g_vehicle_info.log_write_sector = 0;
            g_vehicle_info.log_sector_read_ptr = 0;
            g_vehicle_info.log_sector_write_ptr = 0;
            Init_vhcl_Info();
            return true;
        }
    }
    if((g_vehicle_info.log_read_sector == g_vehicle_info.log_write_sector) &&
       (g_vehicle_info.log_sector_read_ptr == g_vehicle_info.log_sector_write_ptr))
    {
        g_vehicle_info.log_total_number = 0;
        g_vehicle_info.log_read_sector = 0;
        g_vehicle_info.log_write_sector = 0;
        g_vehicle_info.log_sector_read_ptr = 0;
        g_vehicle_info.log_sector_write_ptr = 0;
        Init_vhcl_Info();
        DEBUG(DEBUG_MEDIUM,"[GPS LOG] reset read pointer ,write pointer and total number!\r\n");
    }

    return true;
}

uint8_t Read_vhcl_Data(TelmRealtime_t *VHCL_log, uint8_t num)
{
//	VHCL_log_t VHCL_log_t;
    uint16_t read_sector;
    uint16_t read_sector_pointer;
    uint8_t i=0;

    read_sector_pointer = g_vehicle_info.log_sector_read_ptr;
    read_sector = g_vehicle_info.log_read_sector;

    if (num > g_vehicle_info.log_total_number)
    {
        num = g_vehicle_info.log_total_number;
    }
    if ((num+g_vehicle_info.log_sector_read_ptr) >= VHCL_LOG_NUMBER_PER_SECOTR)
    {
        num = (VHCL_LOG_NUMBER_PER_SECOTR-read_sector_pointer);
    }
    memset(VHCL_log, 0, sizeof(TelmRealtime_t));
    for (i=0;((i+read_sector_pointer) < VHCL_LOG_NUMBER_PER_SECOTR) && (i<num); i++)
    {
        spi_flash_read(FLASH_RLDT_LOG_OFFSET + read_sector * FLASH_SPI_SECTORSIZE + (read_sector_pointer+i) * sizeof(TelmRealtime_t),
                        sizeof(TelmRealtime_t),
                        VHCL_log+(i*sizeof(TelmRealtime_t)));
    }
    return num;
}

#if 0
//2017.10.30 lihaibin modify
void Init_warn_Info(void)
{
    spi_flash_erase_sector(FLASH_WARN_INFO_OFFSET, FLASH_SECTOR_SIZE);
}
#endif
uint16_t Get_warn_Data_Total_Number(void)
{
    return g_warning_info.log_total_number;
}

#if 0
static bool Set_warn_Info(VHCL_Info_t *VHCL_Info)
{
//    int count = 0;
    VHCL_Info_t tmp_g_backup_info;
    if (VHCL_Info->structData.log_total_number == 0)
    {
        return false;
    }
//    while(count < FLASHWRITEREPEATTIMES)
    {
        if(0 != spi_flash_erase_sector(FLASH_WARN_INFO_OFFSET,FLASH_SECTOR_SIZE))
        {
            return false;
        }
        if(0 != spi_flash_write(FLASH_WARN_INFO_OFFSET, sizeof(VHCL_Info_t), VHCL_Info->byte))
        {
            return false;
        }
        if(0 != spi_flash_read(FLASH_WARN_INFO_OFFSET, sizeof(VHCL_Info_t), tmp_g_backup_info.byte))
        {
            return false;
        }
        if(0==(memcmp(VHCL_Info->byte, tmp_g_backup_info.byte, sizeof(VHCL_Info_t))))
        {
            return true;
        }
 //       count++;
    }
    return false;
}

bool Write_warn_Data(VHCL_log_t *VHCL_log)
{
    /* arrive new sector */
    if(g_vehicle_info.log_sector_write_ptr == 0)
    {
        spi_flash_erase_sector(FLASH_WARN_LOG_OFFSET + g_warning_info.log_write_sector * FLASH_SPI_SECTORSIZE,
                               FLASH_SECTOR_SIZE);
        if (g_warning_info.log_total_number == MAX_WARN_LOG_NUMBER)
        {
            g_warning_info.log_total_number -= WARN_LOG_NUMBER_PER_SECOTR;
            if (g_warning_info.log_write_sector >= (WARN_LOG_MAX_SECTOR_NUM-1))
            {
                g_warning_info.log_read_sector = 0;
            }
            else
            {
                g_warning_info.log_read_sector = g_warning_info.log_write_sector+1;
            }
            g_warning_info.log_sector_read_ptr = 0;
        }
    }

    spi_flash_write(FLASH_WARN_LOG_OFFSET + g_warning_info.log_write_sector * FLASH_SPI_SECTORSIZE + \
                    g_warning_info.log_sector_write_ptr * sizeof(VHCL_log_t), \
                    sizeof(VHCL_log_t),
                    VHCL_log);
    g_warning_info.log_sector_write_ptr++;
    /* if write pointer reach end of flash area , reset to 0 */
    if(g_warning_info.log_sector_write_ptr >= WARN_LOG_NUMBER_PER_SECOTR)
    {
        g_warning_info.log_sector_write_ptr = 0;
        g_warning_info.log_write_sector ++;
    }
    if(g_warning_info.log_write_sector >= VHCL_LOG_MAX_SECTOR_NUM)
    {
        g_warning_info.log_write_sector = 0;
        g_warning_info.log_sector_write_ptr = 0;
    }

    if(g_warning_info.log_total_number < MAX_VHCL_LOG_NUMBER)
        g_warning_info.log_total_number ++;
    else 
    {
        /* read pointer = write pointer */
        g_warning_info.log_total_number = MAX_VHCL_LOG_NUMBER;
        g_warning_info.log_read_sector = g_warning_info.log_write_sector;
        g_warning_info.log_sector_read_ptr = g_warning_info.log_sector_write_ptr;
    }
    return true ;
}
bool Set_warn_Data_Next_Read_Pointer(uint8_t num)
{
    if((g_warning_info.log_total_number == 0) || (g_warning_info.log_total_number <= num))
    {
        memset(&g_warning_info, 0, sizeof(data_backup_info_t));
        return false;
    }
    g_warning_info.log_sector_read_ptr += num;
    if(g_warning_info.log_sector_read_ptr >= VHCL_LOG_NUMBER_PER_SECOTR)
    {
        g_warning_info.log_sector_read_ptr = 0;
        g_warning_info.log_read_sector++;
    }
    if(g_warning_info.log_read_sector >= VHCL_LOG_MAX_SECTOR_NUM)
    {
        g_warning_info.log_read_sector = 0;
        g_warning_info.log_sector_read_ptr = 0;
        return true;
    }
    if(g_warning_info.log_total_number > 0)
    {
        g_warning_info.log_total_number -= num;
        if (g_warning_info.log_total_number == 0)
        {
            g_warning_info.log_read_sector = 0;
            g_warning_info.log_write_sector = 0;
            g_warning_info.log_sector_read_ptr = 0;
            g_warning_info.log_sector_write_ptr = 0;
            Init_vhcl_Info();
            return true;
        }
    }
    if((g_warning_info.log_read_sector == g_warning_info.log_write_sector) &&
       (g_warning_info.log_sector_read_ptr == g_warning_info.log_sector_write_ptr))
    {
        g_warning_info.log_total_number = 0;
        g_warning_info.log_read_sector = 0;
        g_warning_info.log_write_sector = 0;
        g_warning_info.log_sector_read_ptr = 0;
        g_warning_info.log_sector_write_ptr = 0;
        Init_warn_Info();
        DEBUG(DEBUG_MEDIUM,"[GPS LOG] reset read pointer ,write pointer and total number!\r\n");
    }

    return true;
}

uint8_t Read_Warn_Data(VHCL_log_t *VHCL_log, uint8_t num)
{
    uint16_t read_sector;
    uint16_t read_sector_pointer;
    uint8_t i=0;

    read_sector_pointer = g_warning_info.log_sector_read_ptr;
    read_sector = g_warning_info.log_read_sector;

    if (num > g_warning_info.log_total_number)
    {
        num = g_warning_info.log_total_number;
    }
    if ((num+g_warning_info.log_sector_read_ptr) >= WARN_LOG_NUMBER_PER_SECOTR)
    {
        num = (WARN_LOG_NUMBER_PER_SECOTR-read_sector_pointer);
    }
    memset(VHCL_log, 0, sizeof(VHCL_log_t));
    for (i=0;((i+read_sector_pointer) < WARN_LOG_NUMBER_PER_SECOTR) && (i<num); i++)
    {
        spi_flash_read(FLASH_RLDT_LOG_OFFSET + read_sector * FLASH_SPI_SECTORSIZE + (read_sector_pointer+i) * sizeof(VHCL_log_t),
                        sizeof(VHCL_log_t),
                        VHCL_log+(i*sizeof(VHCL_log_t)));
    }
    return num;    
}
#endif
/************************GPS Info end*************************************/

/************************OTA Info*****************************************/
#if 0
static void Init_OTA_Info(void)
{

    spi_flash_erase_sector(FLASH_OTA_INFO_OFFSET, FLASH_SECTOR_SIZE);
    DEBUG(DEBUG_MEDIUM, "[PARAM]:OTA Info Data initialized!\n\r");
}

           
bool Set_OTA_Info(OTAHdr_Sct_t ota_info)
{
    memcpy(g_ota_info.byte, ota_info.byte, sizeof(OTAHdr_Sct_t));

    if(0 != spi_flash_erase_sector(FLASH_OTA_INFO_OFFSET, FLASH_SECTOR_SIZE))
    {
        return false;
    }
    return true ;
}

bool Get_OTA_Info(OTAHdr_Sct_t *ota_info) 
{
    if(ota_info == NULL)
    {
        return false ;
    }
    if(0 != spi_flash_read(FLASH_OTA_INFO_OFFSET, sizeof(OTAHdr_Sct_t), ota_info->byte))
    {
        return false;
    }
    return true ;
}

bool Get_OTA_Target_Ver(uint16_t *ver)
{
    OTAHdr_Sct_t ota_info;

    if(ver == NULL)
        return false ;
    if(!(Get_OTA_Info(&ota_info)))
        return false ;

    *ver = (ota_info.structData.ota_target_ver[0] << 8 ) + ota_info.structData.ota_target_ver[1] ;

    return true ;
}

/*
static bool ota_process = false ;

bool Get_OTA_Process_State(void)
{
    return ota_process ;
}

void Set_OTA_Process_State(bool state)
{
    ota_process = state ;
}
*/

bool Start_Write_OTA_Data(uint32_t total_bytes)
{
    OTAHdr_Sct_t ota_info;
    int i = 0 ;
    uint16_t need_sector = 0 ;

    if(total_bytes > OTA_MAX_SIZE)
        return false ;

    if(!(Get_OTA_Info(&ota_info)))
        return false ;

    ota_info.structData.ota_total_bytes[0] = total_bytes >> 24 ;
    ota_info.structData.ota_total_bytes[1] = total_bytes >> 16 ;
    ota_info.structData.ota_total_bytes[2] = total_bytes >> 8 ;
    ota_info.structData.ota_total_bytes[3] = total_bytes >> 0 ;

    if ((total_bytes % FLASH_SPI_SECTORSIZE)==0)
    {
        need_sector = total_bytes / FLASH_SPI_SECTORSIZE;
    }
    else
    {
        need_sector = total_bytes / FLASH_SPI_SECTORSIZE + 1 ;
    }
    for(i = 0 ; i < need_sector ; i ++)
    {
        spi_flash_erase_sector(FLASH_OTA_DATA_OFFSET + (i * FLASH_SPI_SECTORSIZE),
                               FLASH_SECTOR_SIZE);
    }

    Set_OTA_Info(ota_info) ;
    return true ;
}

bool Write_OTA_Data(uint8_t *data , uint16_t len)
{
    uint16_t ota_total_package;
    uint16_t ota_current_package_index;
    uint32_t ota_total_bytes;
    uint32_t ota_downloaded_bytes;
    OTAHdr_Sct_t ota_info;
//    uint16_t crc;

    if(data == NULL)
        return false ;

    Get_OTA_Info(&ota_info) ;
    ota_current_package_index = (ota_info.structData.ota_current_package_index[0] << 8) + \
                                 ota_info.structData.ota_current_package_index[1] ;
    ota_total_package         = (ota_info.structData.ota_total_package[0] << 8) + \
                                 ota_info.structData.ota_total_package[1];
    ota_total_bytes           = (ota_info.structData.ota_total_bytes[0] << 24) + \
                                (ota_info.structData.ota_total_bytes[1] << 16) + \
                                (ota_info.structData.ota_total_bytes[2] << 8) + \
                                 ota_info.structData.ota_total_bytes[3];
    ota_downloaded_bytes      = (ota_info.structData.ota_downloaded_bytes[0] << 24) + \
                                (ota_info.structData.ota_downloaded_bytes[1] << 16) + \
                                (ota_info.structData.ota_downloaded_bytes[2] << 8) + \
                                 ota_info.structData.ota_downloaded_bytes[3];
    if(ota_current_package_index > ota_total_package)
    {
        return false;
    }
    if(ota_downloaded_bytes > ota_total_bytes)
    {
        return false;
    }
    spi_flash_write(FLASH_OTA_DATA_OFFSET + ota_downloaded_bytes, len, data);

//    crc = crc_ccitt(crc , data , len) ;
    ota_current_package_index ++ ;
    ota_downloaded_bytes += len ;

    ota_info.structData.ota_downloaded_bytes[0] = ota_downloaded_bytes >> 24 ;
    ota_info.structData.ota_downloaded_bytes[1] = ota_downloaded_bytes >> 16 ;
    ota_info.structData.ota_downloaded_bytes[2] = ota_downloaded_bytes >> 8 ;
    ota_info.structData.ota_downloaded_bytes[3] = ota_downloaded_bytes >> 0 ;

    ota_info.structData.ota_current_package_index[0] = ota_current_package_index >> 8 ;
    ota_info.structData.ota_current_package_index[1] = ota_current_package_index >> 0 ;

    Set_OTA_Info(ota_info) ;

    return true ;
}

bool End_Write_OTA_Data(uint16_t crc)
{
    OTAHdr_Sct_t ota_info ;

    if(!(Get_OTA_Info(&ota_info)))
        return false ;

//    if(crc != ((ota_info.structData.ota_crc[0] << 8) + ota_info.structData.ota_crc[1]))
//        return false ;

    return true ;
}

bool Read_OTA_Data(uint8_t *data , uint16_t len)
{
    if(data == NULL)
        return false ;

    //ota_read_bytes_pointer
    return true ;
}
#endif

/************************OTA Info end*************************************/

/************************Battery Info*************************************/
/*
	bettery log
	----------------------------------------------------------------------
	 name	vol1	time1	vol2	time2	...
	----------------------------------------------------------------------
	 size	 1		  4		 1		 4		...
	----------------------------------------------------------------------
	max 819 battery data
	battery_log_numberÔÚconfigÄÚ
*/
#if 0
bool WriteBatteryData(Battery_log_t g_battery_log_t)
{
    uint16_t battery_log_number = 0;

    if(battery_log_number >= 819)
    {
        battery_log_number = 0 ;
        spi_flash_erase_sector(FLASH_BATTERY_LOG_OFFSET, FLASH_SECTOR_SIZE);
    }

    spi_flash_write(FLASH_BATTERY_LOG_OFFSET + battery_log_number * sizeof(Battery_log_t),
                        sizeof(Battery_log_t),
                        g_battery_log_t.byte);
    battery_log_number ++;
    return true ;
}

static Battery_log_t ReadBatteryData(uint16_t start)
{
    Battery_log_t g_bettery_log_t ;
    spi_flash_read(FLASH_BATTERY_LOG_OFFSET + start * sizeof(Battery_log_t) ,
                    sizeof(Battery_log_t),
                    g_bettery_log_t.byte);
    return g_bettery_log_t;
}

uint16_t Get_Battery_Log_Number(void)
{
    return 0 ;
}


void Set_Battery_Log_Number(uint16_t bettery_log_number)
{

}

/************************Battery Info end*********************************/

/******************Device started flag*************************/
static uint8_t Get_Dev_Start(void)
{
    uint8_t rt=0;
    uint8_t data[2];
    spi_flash_read(FLASH_DEV_START_OFFSET,2,data);
    if ((data[0]==0x55) && (data[1]==0xaa))
    {
        rt=1;
    }
    return rt;
}

static void Set_Dev_Start(void)
{
    uint8_t data[2]={0x55,0xaa};
    spi_flash_erase_sector(FLASH_DEV_START_OFFSET,FLASH_SECTOR_SIZE);
    spi_flash_write(FLASH_DEV_START_OFFSET, 2, data);
}
#endif
/******************Device started flag end*********************/

static void Init_Param_In_Flash(void)
{
    Init_Manufacture_Setting();
    Init_Config();
    Init_vhcl_Info();
//    Init_OTA_Info();
}

void Save_Param(void)
{
    VHCL_Info_t vhcl_info;

    Uint16ToArray(g_vehicle_info.log_total_number, vhcl_info.structData.log_total_number);
    Uint16ToArray(g_vehicle_info.log_read_sector, vhcl_info.structData.log_read_sector);
    Uint16ToArray(g_vehicle_info.log_sector_read_ptr, vhcl_info.structData.log_sector_read_pointer);
    Uint16ToArray(g_vehicle_info.log_write_sector, vhcl_info.structData.log_write_sector);
    Uint16ToArray(g_vehicle_info.log_sector_write_ptr, vhcl_info.structData.log_sector_write_pointer);    

    Set_vhcl_Info(&vhcl_info);
}

bool Load_Param(void)
{
    bool res = true;

    if(!(Get_Manufacture_Setting(&g_devinfo_sct)))
    {
        DEBUG(DEBUG_HIGH, "[PARAM] Get_Manufacture_Setting error!\r\n");
        res = false ;
    }
    if(!(Get_Config(&g_config_data)))
    {
        DEBUG(DEBUG_HIGH, "[PARAM] Get_Config error!\r\n");
        res = false ;
    }
    if(!(Get_backupl_Info()))
    {
        DEBUG(DEBUG_HIGH, "[PARAM] Get_backupl_Info error!\r\n");
        res = false;
    }
    
#if 0
    if(!(Get_OTA_Info(&g_ota_info)))
    {
//        Init_OTA_Info() ;
        res = false ;
    }

    if (0==Get_Dev_Start())
        Set_Dev_Start();
#endif 
    
    return res ;
}

uint8_t Get_Last_GPS_uploaded(void)
{
/*    if (g_backup_info.log_total_number>0)
    {
        last_gps_uploaded=0;
    }
    else
    {
        last_gps_uploaded=1;
    }*/
    return last_gps_uploaded;
}

void Set_Last_GPS_uploader(uint8_t status)
{
    last_gps_uploaded=status;
}

// 0=GPS point not uploaded, 1=GPS point uploaded
uint8_t Get_Low_Voltage_Uploaded(void)
{
    return low_voltage_uploaded;
}

void Set_Low_Voltage_Uploaded(uint8_t status)
{
    low_voltage_uploaded = status;
}

uint8_t Backup_Is_Empty(void)
{
    return true;
}
/*******************************************************************************
*    Function:  PRM_GetVIN
*
*  Parameters:  'vin' point to the buffer that used to store the VIN.
*     Returns:  None.
* Description:  Get the VIN 
*******************************************************************************/           
bool PRM_GetVIN(uint8_t *vin)
{
    if(vin == NULL)
    {
        return false;
    }
    else
    {
        memcpy((char *)vin, (const char *)g_config_data.structData.vin, 17);
        return true;
    }
}
/*******************************************************************************
*    Function:  PRM_SetVIN
*
*  Parameters:  'vin' point to the VIN.
*     Returns:  true/false.
* Description:  Set the VIN, and store it to FLASH
*******************************************************************************/ 
bool PRM_SetVIN(uint8_t *vin)
{
    if(vin == NULL)
    {
        return false;
    }
    else
    {
        Config_t config;
        memset(&config, 0, sizeof(Config_t));
        Get_Config(&config);
        memcpy((char *)config.structData.vin, (const char *)vin, 17);
        return Set_Config(config);
    }
}
bool PRM_GetSn(uint8_t *pSn)
{
    if(pSn == NULL)
    {
        return false;
    }
    memcpy(pSn, g_devinfo_sct.structData.dev_sn, DEV_SN_LENGTH);
    return true;
}

bool PRM_SetSn(uint8_t *pSn)
{
    if(pSn == NULL)
    {
        return false;
    }
    memcpy(g_devinfo_sct.structData.dev_sn, pSn, DEV_SN_LENGTH);
    return true;
}
/* end of file */

