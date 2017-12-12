#ifndef _GPS_H_
#define _GPS_H_
/**********************************************************************
   Title                      : gps.h         
                                                                         
   Module Description         : GPS. This file is the communication task
                                        with gps modlue.

   Author                     : 
   
 *********************************************************************/
#include "uart.h"
//Module debug flag
#define GPS_CHECK_SATSIG            true

#define GPS_RX_RING_BUF_SIZE        15
#define GPS_BUF_RING_BUF_SIZE       60

#define GPS_MAX_FRAME_LENGTH        120  //50  //fixme. temp value. exclude sync 
#define GPS_RX_BUF_SIZE             GPS_MAX_FRAME_LENGTH
#define GPS_FRAME_MAX_DATA_LEN      (GPS_MAX_FRAME_LENGTH + 2)  //include checksum
#define GPS_NMEA_ADD_LEN            5
//#define GPS_FIELD_LENGTH          15
#define GPS_FIELD_LENGTH            4
#define GPS_UTC_TIMER_LEN           11
#define GPS_UTC_DATE_LEN            7
#define GPS_COG_DATE_LEN            6
#define GPS_SPEED_DATE_LEN          10
#define GPS_POS_DATE_LEN            16

#define GPS_LON_EAST                ('E')
#define GPS_LON_WEST                ('W')
#define GPS_LAT_NORTH               ('N')
#define GPS_LAT_SOUTH               ('S')

typedef enum gps_rx_state_tag
{
    RX_CS_IDLE,
    RX_CS_DATA,
    //RX_CS_END_SYNC,
    RX_CS_NUM_STATES
}gps_rx_state_t;

typedef enum gps_event_tag
{
    GPS_EVT_NOP,
    
    GPS_NUM_EVENTS
}gps_event_t;


typedef enum nmea_msg_tag
{
   NMEA_MSG_GGA,
   NMEA_MSG_GLL,
   NMEA_MSG_RMC,
//   NMEA_MSG_ZDA,
#if GPS_CHECK_SATSIG
   NMEA_MSG_GSV,
   BD_MSG_GSV,
   NMEA_MSG_GSA,
   BD_MSG_GSA,
#endif
   GNSS_MSG_RMC,
   NMEA_NUM_MSG
}nmea_msg_t;

typedef struct gps_data_frame_tag
{
   uint8_t address[GPS_NMEA_ADD_LEN];
   uint8_t data[GPS_FRAME_MAX_DATA_LEN];
}gps_data_frame_t;

typedef union gps_rx_err_tag
{
   uint8_t  error_flags;
   struct  
   {
      uint8_t   overrun  : 1;
      uint8_t   framing  : 1;
      uint8_t   parity   : 1;
      uint8_t   ab_frame : 1;
      uint8_t   chksum   : 1;
      uint8_t   length   : 1;
      uint8_t   timeout  : 1;     
      uint8_t   unused   : 1;     
   }bit;
}gps_rx_err_t;
typedef union gps_time_tag
{
   uint8_t utc_deal_time[16]; //
   struct
   {
      uint8_t utc_raw_year[4];
      uint8_t utc_raw_month[2];
      uint8_t utc_raw_day[2];
      uint8_t utc_raw_hour[2];
      uint8_t utc_raw_min[2];
      uint8_t utc_raw_sec[2];
      uint8_t utc_raw_ms[2];
   }gps_raw_time;
}gps_time_t;

typedef struct gps_sat_detail_tag
{
   uint8_t sat_num;
   uint8_t sat_sig;
}gps_sat_detail_t;

typedef struct gps_sat_info_tag
{
   uint8_t used_sat_num;  // GPS Satellites in ues
   uint8_t viewed_sat_num;  // GPS Satellites in View
   gps_sat_detail_t sat_info[12];// sat_info[0~11] mark the No 1~12 satellite informations.
}gps_sat_info_t;

typedef struct gps_data_tag // 118 bytes
{
   gps_time_t utc_time; //20 bytes
   gps_sat_info_t gps_sat_info; //26bytes
   gps_sat_info_t gnss_sat_info;
   gps_sat_info_t bd_sat_info;
   uint8_t latitude[GPS_FIELD_LENGTH];
   uint8_t north_or_sourth;
   uint8_t longitude[GPS_FIELD_LENGTH];
   uint8_t east_or_west;
   uint8_t altitude[GPS_FIELD_LENGTH];
   uint8_t cog[2];   // degree, MSB first
   uint8_t speed[2]; // km/h, MSB first
   uint8_t pdop[2];  //0.1, MSB first
   bool valid;
}gps_data_t;

typedef bool (*nmea_decode_fptr) (uint8_t * rx_buf);          // decode function

/*********************************************************************/
/* interface for other moduals to get GPS infomation                 */
/*********************************************************************/
extern void vGps_Get_Gps_Info(gps_data_t* gpsInfo);
extern void vGps_Get_Gps_Utc(uint8_t* utc);
//extern Sys_circuit_Status usGPS_Get_gps_Antenna(void);
extern void GPS_Task(void* pvParameters);
extern void GPS_Send_Data(uint8_t *data, uint32_t len);

extern bool vGps_Get_Gps_Status(void);
extern bool GPS_Get_Ready_Flag(void);
extern uint16_t GPS_Parse_Cog(uint8_t *data);
extern void GPS_Get_Sig(uint8_t *sig);
extern void vGps_Get_Valid_Gps_Info(gps_data_t* gpsInfo);
/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif //_GPS_H_
