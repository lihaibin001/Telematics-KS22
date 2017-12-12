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
   Title                      : gps.c         
                                                                         
   Module Description         : GPS. This file is the communication task
                                        with gps modlue.

   Author                     : 
   
 *********************************************************************/
#include <stdio.h>
//#include "gps.h"
#include "ring_buf.h"
#include "definiti.h"
#include "standard.h"     
#include "rtos.h"
//#define USE_DEBUG
#include "Debug.h"
/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
//#define USE_DUMMY_GPS_DATA

#define GPS_DEBUG_BOARD         false
#define GPS_DEBUG_VALUE         false
#define GPS_DEBUG_MUTEX         false

#if GPS_DEBUG_BOARD
#define GPS_CHANNEL             UART_CONF_CMMB
#define GPS_BAUD_RATE           UART_BAUD_9600
#define GPS_UART_CONFIG         (UART_LSB_FIRST | UART_PARITY_NONE | UART_DATA_BIT_8 | UART_STOP_ONE )
#endif //GPS_DEBUG_BOARD

#define FRAME_START_CHAR        0x24
#define FIRST_END_SYNC_CHAR     0x0D
#define SECOND_END_SYNC_CHAR    0x0A
#define NO_ERROR                0x00

#define NMEA_CHKSUM_LENGTH      3      //include "*" charcter
#define ASCII_TO_HEX_VALUE      0x30
#define COLON_CH                0x3A //":"
#define SLASH_CH                0x2F //"/"

#define ZERO_CH                                        0x30

#define KNOTS_DIV (1943844)

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/

//static bool first_sync_char_received;
//static bool second_sync_char_received;
static Ring_Buf_Type        gps_rx_ring_control;
static gps_data_frame_t     gps_rx_ring_buffer[GPS_RX_RING_BUF_SIZE];
static gps_rx_err_t         gps_rx_err;
static gps_data_t           gps_data;
static gps_data_t           last_gps_data;
static gps_data_t           last_valid_gps_data;
//uint16_t gpsAntennaOld;

//static Ring_Buf_Type gps_buf_ring_control;
//static gps_data_t gps_buf_ring_buffer[GPS_BUF_RING_BUF_SIZE];

#define UBX_Ready_To_Receive_Data()    (!Ring_Buf_Is_Full(&gps_rx_ring_control))

#if GPS_DEBUG_VALUE
static uint8_t gps_debug_flag;
#endif

static bool gps_data_ready_flag = true;
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static void GPS_COM_Initialize(void);
static void Ublox_Module_Initialization(void);
static bool Gps_any_received_frames(void);
static void Gps_check_receive(void);
static void Gps_Rx_Data_Byte(uint8_t data, uint8_t error);
static void GPS_Validate_Rx_Frame_Data(const uint8_t* rx_buffer, uint8_t num_bytes);
static void gps_evt_nop(int16_t data);
static void GPS_update_last(void);

static gps_data_frame_t* Gps_received_frame(void);
//static uint8_t Get_Next_Data_Len_Before_Comma(uint8_t * rx_buf);
//static uint8_t* Get_Filed_Data_Pointer(uint8_t* rx_buf, uint8_t field);

static bool NMEA_GNGGA_Msg_Decode(uint8_t * rx_buf);
static bool NMEA_GPGLL_Msg_Decode(uint8_t * rx_buf);
static bool NMEA_GPRMC_Msg_Decode(uint8_t * rx_buf);
#if GPS_CHECK_SATSIG
static bool NMEA_GPGSV_Msg_Decode(uint8_t * rx_buf);
static bool BD_BDGSV_Msg_Decode(uint8_t * rx_buf);
static bool NMEA_GPGSA_Msg_Decode(uint8_t * rx_buf);
static bool BD_BDGSA_Msg_Decode(uint8_t * rx_buf);
#endif
static bool GNSS_GNRMC_Msg_Decode(uint8_t * rx_buf);
//static void prvGPS_checkAntenna(void);

// Compress latitude, longitude and altitude to 4 bytes
static void convert_gps_lon(uint8_t *gps_data, uint8_t *dst_buf);
static void convert_gps_alt(uint8_t *gps_data, uint8_t *dst_buf);
static void convert_gps_knot(uint8_t *gps_data, uint8_t *dst_buf);
static void convert_gps_cog(uint8_t *gps_data, uint8_t *dst_buf);
static void convert_gps_dop(uint8_t *gps_data, uint8_t *dst_buf);

/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/
static const void_int16_fptr gps_event_handler[GPS_NUM_EVENTS] =
{
    gps_evt_nop,
};

static const char* nmea_msg_add_string[NMEA_NUM_MSG] = 
{
    "GNGGA",
    "GPGLL",
    "GPRMC",
//    "GPZDA",
#if GPS_CHECK_SATSIG
    "GPGSV",
    "BDGSV",
    "GPGSA",
    "BDGSA",
#endif
    "GNRMC",
};

static const nmea_decode_fptr nmea_decode_table[NMEA_NUM_MSG] =
{
    NMEA_GNGGA_Msg_Decode,
    NMEA_GPGLL_Msg_Decode,
    NMEA_GPRMC_Msg_Decode,
#if GPS_CHECK_SATSIG
    NMEA_GPGSV_Msg_Decode,
    BD_BDGSV_Msg_Decode,
    NMEA_GPGSA_Msg_Decode,
    BD_BDGSA_Msg_Decode,
#endif
    GNSS_GNRMC_Msg_Decode
};

/**********************************************************************
 * Function Definitions
 *********************************************************************/
//#if GPS_DEBUG
/*******************************************************************************
*    Function:  GPS_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle communication between V850 and UBLOX GPS module.
*******************************************************************************/
extern void GPS_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint8_t data;

    DEBUG( DEBUG_HIGH, "[GPS]:GPS TASK Started!\r\n");
    
 #if 1   
    GPS_COM_Initialize();

    Ublox_Module_Initialization();
#endif

    while(PS_Running())
    {
        if(OS_E_OK == OS_Wait_Message(OS_GPS_TASK, &msg.all, MSec_To_Ticks(500)))
        {
#if 1
            if(msg.parts.msg < GPS_NUM_EVENTS)
            {
                if(NULL != gps_event_handler[msg.parts.msg])
                {
                     (*gps_event_handler[msg.parts.msg])(msg.parts.data);
                }
            }
#endif 
        }
#if 1
        while (Uart_Get_Char(UART_GPS_CHANNEL,&data) == true)
        {
            Gps_Rx_Data_Byte(data,0x00);
        }
        do
        {
            Gps_check_receive();
        }while(Gps_any_received_frames());
#endif 
    }
    OS_Terminate_Task();
}
//#endif
/*******************************************************************************
*    Function:  vGps_Get_Gps_Status
*
*  Parameters:  GPS Infomation to return
*     Returns:  None
* Description:  GCOM_Initialize.
*******************************************************************************/
extern bool vGps_Get_Gps_Status(void)
{
    return last_gps_data.valid;
//    return gps_data.valid;
}

/*******************************************************************************
*    Function:  vGps_Get_Gps_Info
*
*  Parameters:  GPS Infomation to return
*     Returns:  None
* Description:  GCOM_Initialize.
*******************************************************************************/
extern void vGps_Get_Gps_Info(gps_data_t* gpsInfo)
{
//	static uint16_t s_speed = 0 ;
    /* to prevend gps_data from being accessed while reading gps_data */
//    memcpy(gpsInfo,&gps_data,sizeof(gps_data_t));
    memcpy(gpsInfo,&last_gps_data,sizeof(gps_data_t));

#ifdef USE_DUMMY_GPS_DATA
//        memcpy(gpsInfo->latitude,"3111.79770",11);
//    memcpy(gpsInfo->longitude,"12126.21970",12);
//    memcpy(gpsInfo->latitude,"3958.7222",11);
//    memcpy(gpsInfo->longitude,"11620.9520",12);
	/*
		format : old - 2014 01 01 00 00 00  
				 new - 20140101000000
	*/
    memcpy(gpsInfo->altitude,"6",2);
    gpsInfo->valid = true;
    gpsInfo->north_or_sourth = 'N';
    gpsInfo->east_or_west = 'E';
    gpsInfo->utc_time.utc_deal_time[0] = '2';
    gpsInfo->utc_time.utc_deal_time[1] = '0';
    gpsInfo->utc_time.utc_deal_time[2] = '1';
    gpsInfo->utc_time.utc_deal_time[3] = '4';
    gpsInfo->utc_time.utc_deal_time[4] = '0';
    gpsInfo->utc_time.utc_deal_time[5] = '1';
    gpsInfo->utc_time.utc_deal_time[6] = '0';
    gpsInfo->utc_time.utc_deal_time[7] = '1';
    gpsInfo->utc_time.utc_deal_time[8] = '0';
    gpsInfo->utc_time.utc_deal_time[9] = '0';
    gpsInfo->utc_time.utc_deal_time[10] = '0';
    gpsInfo->utc_time.utc_deal_time[11] = '0';
    gpsInfo->utc_time.utc_deal_time[12] = '0';
    gpsInfo->utc_time.utc_deal_time[13] = '0';
    gpsInfo->speed[0] = s_speed >> 8 ;
    gpsInfo->speed[1] = s_speed >> 0 ;
    s_speed ++ ;
#endif
    /* enable gps_data accessing */
}

/*******************************************************************************
*    Function:  vGps_Get_Gps_Info
*
*  Parameters:  GPS Infomation to return
*     Returns:  None
* Description:  GCOM_Initialize.
*******************************************************************************/
extern void vGps_Get_Valid_Gps_Info(gps_data_t* gpsInfo)
{
    memcpy(gpsInfo, &last_valid_gps_data, sizeof(gps_data_t));    
}

/*******************************************************************************
*    Function:  vGps_Get_Gps_Utc
*
*  Parameters:  GPS UTC to return
*     Returns:  None
* Description:  vGps_Get_Gps_Utc.
*******************************************************************************/
void vGps_Get_Gps_Utc(uint8_t* utc)
{
#ifdef USE_DUMMY_GPS_DATA
    gps_data.valid = true;
    gps_data.utc_time.utc_deal_time[0] = '2';
    gps_data.utc_time.utc_deal_time[1] = '0';
    gps_data.utc_time.utc_deal_time[2] = '1';
    gps_data.utc_time.utc_deal_time[3] = '6';
    gps_data.utc_time.utc_deal_time[4] = '1';
    gps_data.utc_time.utc_deal_time[5] = '1';
    gps_data.utc_time.utc_deal_time[6] = '2';
    gps_data.utc_time.utc_deal_time[7] = '9';
    gps_data.utc_time.utc_deal_time[8] = '0';
    gps_data.utc_time.utc_deal_time[9] = '0';
    gps_data.utc_time.utc_deal_time[10] = '0';
    gps_data.utc_time.utc_deal_time[11] = '0';
    gps_data.utc_time.utc_deal_time[12] = '0';
    gps_data.utc_time.utc_deal_time[13] = '0';
#endif
//    *utc = gps_data.valid;
//    memcpy(utc+1,&gps_data.utc_time.utc_deal_time,16);
    *utc = last_gps_data.valid;
    memcpy(utc+1,&last_gps_data.utc_time.utc_deal_time,16);
}

/*******************************************************************************
*    Function:  GCOM_Initialize
*
*  Parameters:  GPS Infomation to return
*     Returns:  None
* Description:  GCOM_Initialize.
*******************************************************************************/
/*extern gps_data_t* pGps_Get_Gps_Info(void)
{
   return &gps_data;
}
*/

static void GPS_update_last(void)
{
    memcpy(&last_gps_data,&gps_data,sizeof(gps_data_t));
    if(gps_data.valid == true)
    {
        memcpy(&last_valid_gps_data, &gps_data, sizeof(gps_data_t));
    }
}

/*******************************************************************************
*    Function:  GCOM_Initialize
*
*  Parameters:  None
*     Returns:  None
* Description:  GCOM_Initialize.
*******************************************************************************/
static void GPS_COM_Initialize(void)
{
    gps_rx_err.error_flags = NO_ERROR;
    gps_data.valid = false;

    //Add the time fixed value
    gps_data.utc_time.gps_raw_time.utc_raw_year[0] = '2';
    gps_data.utc_time.gps_raw_time.utc_raw_year[1] = '0';
    Uart_Initialize(UART_GPS_CHANNEL);
    Ring_Buf_Reset(&gps_rx_ring_control, GPS_RX_RING_BUF_SIZE);
//    UART_Allocate(UART_GPS_CHANNEL, 9600, (UART_LSB_TRANSFER | UART_NO_PARITY | UART_8_DATA_BITS | UART_1_STOP_BIT), NULL);
}

/*******************************************************************************
*    Function:  Gps_Rx_Data_Byte
*
*  Parameters:  None
*  Returns:  None
* Description:  Gps_Rx_Data_Byte.
*******************************************************************************/
static void Gps_Rx_Data_Byte(uint8_t data, uint8_t error)
{
    static gps_rx_state_t gps_rx_state = RX_CS_IDLE;          
    static uint8_t rx_count = 0;
    static uint8_t gps_rx_buffer[GPS_RX_BUF_SIZE] = {0};
    uint8_t rx_data_byte_error = error & 0x07;   /*Mask Parity, Stop bit and Overrun errors*/

    static bool first_sync_ch_received = false;

    gps_rx_err.error_flags |= (error & 0x07); /*Mask Parity, Stop bit and Overrun errors*/

    switch(gps_rx_state)                   
    {
        case RX_CS_IDLE:
            if ((0x00 == rx_data_byte_error) && (FRAME_START_CHAR == data))/*Jump only if no error in rx byte*/
            {
                gps_rx_state = RX_CS_DATA;
                memset(gps_rx_buffer, 0x00, GPS_RX_BUF_SIZE);
                rx_count = 0;

                //ubx_receiving_frame = FALSE;      /*Initialize rx variables*/
                gps_rx_err.error_flags = 0x00;
            }
            break;

        case RX_CS_DATA:
            if ((0x00 < rx_data_byte_error) )
            {
                gps_rx_state = RX_CS_IDLE; 
            }
            else if (FIRST_END_SYNC_CHAR == data)
            {
                first_sync_ch_received = true;
                gps_rx_buffer[rx_count++] = data;
                //rx_count = 0;
            }
            else if((SECOND_END_SYNC_CHAR == data) && first_sync_ch_received)
            {
                gps_rx_state = RX_CS_IDLE;
                first_sync_ch_received = false;
                GPS_Validate_Rx_Frame_Data(gps_rx_buffer, rx_count -1); //abandon first sync char
                memset(gps_rx_buffer, 0x00, GPS_RX_BUF_SIZE);
                rx_count = 0;
            }
            else
            {
                gps_rx_buffer[rx_count++] = data;
                first_sync_ch_received = false;
            }
            break; 

       default:              
         gps_rx_state = RX_CS_IDLE;
         break;
   }/*Switch*/
}

/*******************************************************************************
*    Function:  GPS_Validate_Rx_Frame_Data
*
*  Parameters:  None
*     Returns:  None
* Description:  GPS_Validate_Rx_Frame_Data.
*******************************************************************************/
static void GPS_Validate_Rx_Frame_Data(const uint8_t* rx_buffer, uint8_t num_bytes)
{
    uint8_t data_buffer[GPS_RX_BUF_SIZE];
    uint8_t cal_checksum;
    uint8_t index;

    if( num_bytes > GPS_RX_BUF_SIZE)
    {
        num_bytes = GPS_RX_BUF_SIZE;  //be
    }

    if( num_bytes >= 10)
    {
        memset(data_buffer, 0x00, GPS_RX_BUF_SIZE);
        memcpy(data_buffer, rx_buffer, num_bytes);

        //checksum compare.
        cal_checksum = 0;
        for(index = 0; index < num_bytes -NMEA_CHKSUM_LENGTH; index++)
        {
            cal_checksum ^= rx_buffer[index];
        }
        if(NO_ERROR == gps_rx_err.error_flags)
        {
#if 1
            if (UBX_Ready_To_Receive_Data())
            {
               memcpy(&gps_rx_ring_buffer[gps_rx_ring_control.in], data_buffer, (num_bytes -NMEA_CHKSUM_LENGTH));   //don't copy checksum
               *((uint8_t *)&gps_rx_ring_buffer[gps_rx_ring_control.in]+ (num_bytes -NMEA_CHKSUM_LENGTH)) = 0;
               Ring_Buf_Add(&gps_rx_ring_control);    
            }
            else
            {
            }
#endif
        }
        else
        {
        }
    }
}

/*******************************************************************************
*    Function:  Ublox_Module_Initialization
*
*  Parameters:  None
*     Returns:  None
* Description:  Ublox_Module_Initialization.
*******************************************************************************/
static void Ublox_Module_Initialization(void)
{
//    const gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0,};
    //hardware Initilization
    IO_3V3_GPS_EN_OUT(Bit_SET);
    IO_GPS_RESET_OUT(Bit_RESET);
}


/*******************************************************************************
*    Function:  Gps_check_receive
*
*  Parameters:  None
*     Returns:  None
* Description:  Gps_check_receive.
*******************************************************************************/
static void Gps_check_receive(void)
{
    gps_data_frame_t * rx_frame;
    uint8_t msg_address[GPS_NMEA_ADD_LEN +1]; //length +1 , in order to compare with nmea_msg_add_string.
    uint8_t index;

    memset(msg_address, 0x00, GPS_NMEA_ADD_LEN + 1);
    if (Gps_any_received_frames())
    {
        rx_frame = Gps_received_frame();                            /* get pointer to received data */
        memcpy(msg_address, rx_frame->address, 5);
        for(index = 0; index < NMEA_NUM_MSG; index++)
        {
            if(!strcmp((const char*)msg_address, nmea_msg_add_string[index]))
            {
                break;
            }
        }
        if( index < NMEA_NUM_MSG)
        {
            if((*nmea_decode_table[index])((uint8_t*)(rx_frame->data)))
            {
                GPS_update_last();
                if(index == NMEA_NUM_MSG-1)
                {   
                    //send refresh event to record task
                    OS_Send_Message(OS_RECORD_TASK,Build_Message(RECORD_EVT_RLTM_DATA_REFRESH, 1));
                }
            }
        }
    }
}

/*******************************************************************************
*    Function:  ucom_any_received_frames
*
*  Parameters:  None
*     Returns:  TRUE if any receive frames are available
* Description:  Notify decoder task that new frame is available
*******************************************************************************/
static bool Gps_any_received_frames(void)
{
   return(!Ring_Buf_Is_Empty(&gps_rx_ring_control));
//  return false;
}

/*******************************************************************************
*    Function:  ubx_received_frame
*
*  Parameters:  None
*     Returns:  Pointer to received frame / NULL if no available frame
* Description:  Returns pointer to oldest received frame
*               Increment receive ring buffer
*******************************************************************************/
static gps_data_frame_t* Gps_received_frame(void)
{
   gps_data_frame_t *rx_frame = NULL;

   if (Gps_any_received_frames())         // ensure frame is available 
   {
      rx_frame =  &gps_rx_ring_buffer[gps_rx_ring_control.out];                 
      Ring_Buf_Remove(&gps_rx_ring_control);
   }
   return(rx_frame);
}

/*******************************************************************************
*    Function:  gps_evt_nop
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  gps_evt_nop
*******************************************************************************/
static void gps_evt_nop(int16_t data)
{

}

static bool GNSS_GNRMC_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t *original_p;
    uint8_t *p ;
    uint8_t *p_temp;
//    uint8_t test;

#if 0
//#if GPS_DEBUG_VALUE    
    test = strlen(",135556.00,A,4717.11437,N,00833.91522,E,0.004,77.52,190712,,,");
    memcpy(rx_buf, ",135556.00,A,4717.11437,N,00833.91522,E,0.004,77.52,190712,,,",test);
#endif

#if GPS_DEBUG_MUTEX
    /* to prevend gps_data from being accessed while updating gps_data */
    //Disable_Interrupts();
    vTaskSuspendAll();
#endif

    gps_data_ready_flag = false;

    original_p = rx_buf;
    //Save the UTC time
    rx_buf++; //elimite first comma.
    p = rx_buf;

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= GPS_UTC_TIMER_LEN))
    {
        gps_data.utc_time.gps_raw_time.utc_raw_hour[0] = rx_buf[0];
        gps_data.utc_time.gps_raw_time.utc_raw_hour[1] = rx_buf[1];

        gps_data.utc_time.gps_raw_time.utc_raw_min[0] = rx_buf[2];
        gps_data.utc_time.gps_raw_time.utc_raw_min[1] = rx_buf[3];

        gps_data.utc_time.gps_raw_time.utc_raw_sec[0] = rx_buf[4];
        gps_data.utc_time.gps_raw_time.utc_raw_sec[1] = rx_buf[5];

       //  memcpy((void*)gps_data.utc_cur_time, rx_buf, len);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(len==1)
    {
//        DEBUG(DEBUG_MEDIUM,"[valid]:%s\n\r",rx_buf);
        if('A' == *rx_buf)
        {
            gps_data.valid = true;
        }
        else
        {
            gps_data.valid = false;
        }
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
    if(len >0)
    {
//        memcpy(gps_data.latitude, rx_buf, len);
//        gps_data.latitude[len] = 0;
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.latitude);
    }
    else
    {
//        gps_data.latitude[0] = 0;
//        memset(gps_data.latitude,0,GPS_FIELD_LENGTH);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= 1))
    {
        gps_data.north_or_sourth = *rx_buf;
    }
    rx_buf+= (len + 1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
    if(len >0)
    {
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.longitude);
//        gps_data.longitude[len] = 0;
    }
    else
    {
//        gps_data.longitude[0] = 0;
//        memset(gps_data.longitude,0,GPS_FIELD_LENGTH);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(1 == len)//((len >0) && (len <= 1))
    {
        gps_data.east_or_west = *rx_buf;
    }
    rx_buf+= (len+1);  //also jump ove comma

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    rx_buf+= (len+1);  //also jump over comma

    p_temp = Get_Filed_Data_Pointer(original_p, 7);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if(len >0)
    {
        //get GPS data
        uint8_t spd_tmp[8];
        memset(spd_tmp,0,8);
        memcpy(spd_tmp, p_temp, len);
        convert_gps_knot(spd_tmp,gps_data.speed);
    }
    else
    {
        memset(gps_data.speed,0,2);
    }

    p_temp = Get_Filed_Data_Pointer(original_p, 8);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if((len >0) && (len <= GPS_COG_DATE_LEN))
    {
        uint8_t cog_tmp[10];
        memset(cog_tmp,0,10);
        memcpy(cog_tmp, p_temp, len);        
        convert_gps_cog(cog_tmp,gps_data.cog);
    }
    else
    {
        memset(gps_data.cog,0,2);
    }

    //Save the UTC data
    p_temp = Get_Filed_Data_Pointer(original_p, 9);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if((len >0) && (len <= GPS_UTC_DATE_LEN))
    {
        gps_data.utc_time.gps_raw_time.utc_raw_day[0] = p_temp[0];
        gps_data.utc_time.gps_raw_time.utc_raw_day[1] = p_temp[1];
        gps_data.utc_time.gps_raw_time.utc_raw_month[0] = p_temp[2];
        gps_data.utc_time.gps_raw_time.utc_raw_month[1] = p_temp[3];
        gps_data.utc_time.gps_raw_time.utc_raw_year[2] = p_temp[4];
        gps_data.utc_time.gps_raw_time.utc_raw_year[3] = p_temp[5];

    //   memcpy(gps_data.utc_cur_date, p_temp, len);
    }

//    DEBUG(DEBUG_LOW,"[GNRMC]:%s\n\r",original_p);
#if GPS_DEBUG_MUTEX
    /* enable gps_data accessing */
    // Enable_Interrupts();
    xTaskResumeAll();
#endif
    gps_data_ready_flag = true;

    return true;
}

/*******************************************************************************
*    Function:  gps_evt_nop
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  gps_evt_nop
*******************************************************************************/
static bool NMEA_GPRMC_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    uint8_t* p ;
    uint8_t* p_temp;
//    uint8_t test;

#if 0
//#if GPS_DEBUG_VALUE    
    test = strlen(",135556.00,A,4717.11437,N,00833.91522,E,0.004,77.52,190712,,,");
    memcpy(rx_buf, ",135556.00,A,4717.11437,N,00833.91522,E,0.004,77.52,190712,,,",test);
#endif

#if GPS_DEBUG_MUTEX
    /* to prevend gps_data from being accessed while updating gps_data */
    //Disable_Interrupts();
    vTaskSuspendAll();
#endif

    gps_data_ready_flag = false;

    original_p = rx_buf;
    //Save the UTC time
    rx_buf++; //elimite first comma.
    p = rx_buf;

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= GPS_UTC_TIMER_LEN))
    {
        gps_data.utc_time.gps_raw_time.utc_raw_hour[0] = rx_buf[0];
        gps_data.utc_time.gps_raw_time.utc_raw_hour[1] = rx_buf[1];

        gps_data.utc_time.gps_raw_time.utc_raw_min[0] = rx_buf[2];
        gps_data.utc_time.gps_raw_time.utc_raw_min[1] = rx_buf[3];

        gps_data.utc_time.gps_raw_time.utc_raw_sec[0] = rx_buf[4];
        gps_data.utc_time.gps_raw_time.utc_raw_sec[1] = rx_buf[5];

       //  memcpy((void*)gps_data.utc_cur_time, rx_buf, len);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(len==1)
    {
//        DEBUG(DEBUG_MEDIUM,"[valid]:%s\n\r",rx_buf);
        if('A' == *rx_buf)
        {
            gps_data.valid = true;
        }
        else
        {
            gps_data.valid = false;
        }
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
    if(len >0)
    {
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.latitude);
//        gps_data.latitude[len] = 0;
    }
    else
    {
//        gps_data.latitude[0] = 0;
//        memset(gps_data.latitude,0,GPS_FIELD_LENGTH);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= 1))
    {
        gps_data.north_or_sourth = *rx_buf;
    }
    rx_buf+= (len + 1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
    if(len >0)
    {
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.longitude);
//        gps_data.longitude[len] = 0;
    }
    else
    {
//        gps_data.longitude[0] = 0;
//        memset(gps_data.longitude,0,GPS_FIELD_LENGTH);
    }
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(1 == len)//((len >0) && (len <= 1))
    {
        gps_data.east_or_west = *rx_buf;
    }
    rx_buf+= (len+1);  //also jump ove comma

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    rx_buf+= (len+1);  //also jump over comma

    p_temp = Get_Filed_Data_Pointer(original_p, 7);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if(len >0)
    {
        //get GPS data
        uint8_t spd_tmp[8];
        memset(spd_tmp,0,8);
        memcpy(spd_tmp, p_temp, len);
        convert_gps_knot(spd_tmp,gps_data.speed);
    }
    else
    {
        memset(gps_data.speed,0,2);
    }

    p_temp = Get_Filed_Data_Pointer(original_p, 8);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if((len >0) && (len <= GPS_COG_DATE_LEN))
    {
        uint8_t cog_tmp[10];
        memset(cog_tmp,0,10);
        memcpy(cog_tmp, p_temp, len);
        convert_gps_cog(cog_tmp,gps_data.cog);
    }
    else
    {
        memset(gps_data.cog,0,2);
    }
    //Save the UTC data
    p_temp = Get_Filed_Data_Pointer(original_p, 9);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
    if((len >0) && (len <= GPS_UTC_DATE_LEN))
    {
        gps_data.utc_time.gps_raw_time.utc_raw_day[0] = p_temp[0];
        gps_data.utc_time.gps_raw_time.utc_raw_day[1] = p_temp[1];
        gps_data.utc_time.gps_raw_time.utc_raw_month[0] = p_temp[2];
        gps_data.utc_time.gps_raw_time.utc_raw_month[1] = p_temp[3];
        gps_data.utc_time.gps_raw_time.utc_raw_year[2] = p_temp[4];
        gps_data.utc_time.gps_raw_time.utc_raw_year[3] = p_temp[5];

    //   memcpy(gps_data.utc_cur_date, p_temp, len);
    }

//   DEBUG(DEBUG_LOW,"[GPRMC]:%s\n\r",original_p);
#if GPS_DEBUG_MUTEX
    /* enable gps_data accessing */
    // Enable_Interrupts();
    xTaskResumeAll();
#endif
    gps_data_ready_flag = true;

    return true;
}

/*******************************************************************************
*    Function:  NMEA_GPGGA_Msg_Decode
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  NMEA_GPGGA_Msg_Decode
*******************************************************************************/
static bool NMEA_GNGGA_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    //uint8_t* original_p1;
    uint8_t* p ;
    uint8_t* p_temp;

#if GPS_DEBUG_VALUE    
    uint8_t test; //Move test here to define it along with GPS_DEBUG_VALUE.
    test = strlen(",135556.00,3044.39172,N,12049.59415,E,1,09,2.39,15.5,M,8.6,M,,");
    memcpy(rx_buf, ",135556.00,3044.39172,N,12049.59415,E,1,09,2.39,15.5,M,8.6,M,,",test);
#endif
//    DEBUG(DEBUG_LOW,"[GNGGA]%s\n\r",rx_buf);
#if GPS_DEBUG_MUTEX
	/* to prevend gps_data from being accessed while updating gps_data */
    //Disable_Interrupts();
    vTaskSuspendAll();
#endif
    gps_data_ready_flag = false;

    original_p = rx_buf;
    //original_p1 = rx_buf;
    rx_buf++; //elimite first comma.
    p = rx_buf;

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
#if 1 // UTC time change to GPGGA command
    if((len >0) && (len <= GPS_UTC_TIMER_LEN))
    {
/*       gps_data.utc_time.gps_raw_time.utc_raw_hour[0] = rx_buf[0];
       gps_data.utc_time.gps_raw_time.utc_raw_hour[1] = rx_buf[1];
    
       gps_data.utc_time.gps_raw_time.utc_raw_min[0] = rx_buf[2];
       gps_data.utc_time.gps_raw_time.utc_raw_min[1] = rx_buf[3];
    
       gps_data.utc_time.gps_raw_time.utc_raw_sec[0] = rx_buf[4];
       gps_data.utc_time.gps_raw_time.utc_raw_sec[1] = rx_buf[5];
*/
      //  memcpy((void*)gps_data.utc_cur_time, rx_buf, len);
    }
#endif
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
/*    if(len >0)
    {
//        memcpy(gps_data.latitude, rx_buf, len);
//        gps_data.latitude[len] = 0;
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.latitude);
    }
    else
    {
//        gps_data.latitude[0] = 0;
    }*/
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
/*    if((len >0) && (len <= 1))
    {
        gps_data.north_or_sourth = *rx_buf;
    }*/
    rx_buf+= (len + 1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
/*    if(len >0)
    {
        uint8_t lon_tmp[16];
        memset(lon_tmp,0,16);
        memcpy(lon_tmp, rx_buf, len);
        convert_gps_lon(lon_tmp,gps_data.longitude);
//        gps_data.longitude[len] = 0;
    }
    else
    {
//        gps_data.longitude[0] = 0;
//        memset(gps_data.longitude,0,GPS_FIELD_LENGTH);
    }
*/
    rx_buf+= (len+1);

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
/*    if(1 == len)//((len >0) && (len <= 1))
    {
        gps_data.east_or_west = *rx_buf;
    }*/
    rx_buf+= (len+1);  //also jump ove comma

    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
/*    if(1 == len)//((len >0) && (len <= 1))
    {
        //gps_data.valid = (*rx_buf == )
        if(ZERO_CH != *rx_buf)
        {
            gps_data.valid = true;
        }
        else
        {
            gps_data.valid = false;
        }
    }
    else
    {
            gps_data.valid = false;
    }*/
    rx_buf+= (len+1);  //also jump over comma

   //Get the viewed statelite number for gps_sat_info structure.
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(len == 2)
    {
       gps_data.gnss_sat_info.used_sat_num = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) -0x30;
    }
    else if (len==1)
    {
       gps_data.gnss_sat_info.used_sat_num = *(rx_buf) -0x30;
    }
    rx_buf+= (len+1);  //also jump over comma

    p_temp = Get_Filed_Data_Pointer(original_p, 8);
    len = Get_Next_Data_Len_Before_Comma(p_temp);
//    if((len >0) && (len <= GPS_FIELD_LENGTH))
    if(len >0)
    {
//        memcpy(gps_data.altitude, p_temp, len);
//        gps_data.altitude[len] = 0;
        uint8_t alt_tmp[10];
        memset(alt_tmp,0,10);
        memcpy(alt_tmp, p_temp, len);
        convert_gps_alt(alt_tmp,gps_data.altitude);
//        gps_data.longitude[len] = 0;
    }
    else
    {
//        gps_data.longitude[0] = 0;
//        memset(gps_data.altitude,0,GPS_FIELD_LENGTH);
    }
   // rx_buf+= (len+1);  //also jump ove comma

#if GPS_DEBUG_MUTEX
	/* enable gps_data accessing */
   // Enable_Interrupts();
   xTaskResumeAll();
#endif   
    gps_data_ready_flag = true;

    return true;
}

/*******************************************************************************
*    Function:  NMEA_GPGLL_Msg_Decode
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  NMEA_GPGLL_Msg_Decode
*******************************************************************************/
static bool NMEA_GPGLL_Msg_Decode(uint8_t *rx_buf)
{
//    DEBUG(DEBUG_MEDIUM,"[GPGLL]%s\n\r",rx_buf);
    return true;
}

#if GPS_CHECK_SATSIG
/*******************************************************************************
*    Function:  NMEA_GPGSV_Msg_Decode
*
*  Parameters:  Tx Message
*     Returns:  Status
* Description:  NMEA_GPGSV_Msg_Decode
*******************************************************************************/
static bool NMEA_GPGSV_Msg_Decode(uint8_t *rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    uint8_t* p ;
    uint8_t* p_temp;
    //uint8_t pack_sum;
    uint8_t pack_num;
    uint8_t cur_sat_num;
//    DEBUG(DEBUG_MEDIUM,"[GPGSV]%s\n\r",rx_buf);

#if GPS_DEBUG_VALUE
    uint8_t test; //Move test here to define it along with GPS_DEBUG_VALUE.
    gps_debug_flag ++;

    if (gps_debug_flag == 1)
    {
        test = strlen(",3,1,09,41,51,61,71,42,52,62,72,43,53,63,73,44,54,64,74*AA");
        memcpy(rx_buf, ",3,1,09,41,51,61,71,42,52,62,72,43,53,63,73,44,54,64,74*AA",test);
    }
    else if (gps_debug_flag == 2)
    {
        test = strlen(",3,2,09,41,51,61,75,42,52,62,76,43,53,63,77,44,54,64,78*AA");
        memcpy(rx_buf, ",3,2,09,41,51,61,75,42,52,62,76,43,53,63,77,44,54,64,78*AA",test);
    }
    else if (gps_debug_flag == 3)
    {
        gps_debug_flag = 0;
        test = strlen(",3,3,09,41,51,61,79,42,52,62,01,43,53,63,02,44,54,64,03*AA");
        memcpy(rx_buf, ",3,3,09,41,51,61,79,42,52,62,01,43,53,63,02,44,54,64,03*AA",test);
    }
      
#endif

    original_p = rx_buf;
    rx_buf++; //elimite first comma.
    p = rx_buf;

    //First parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1);
    if((len >0) && (len <= 2))
    {
        if (len == 2)
        {
            //pack_sum = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        }
        else
        {      
            //pack_sum = *rx_buf - 0x30;
        }
    }
    rx_buf+= (len+1);

    //Second parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= 2))
    {
        if (len == 2)
        {
            pack_num = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        }
        else
        {      
            pack_num = *rx_buf - 0x30;
        }
    }
    rx_buf+= (len+1);
   
    //Third parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(len == 2)
    {
        gps_data.gps_sat_info.viewed_sat_num = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        if (gps_data.gps_sat_info.viewed_sat_num > 12) 
        {
            gps_data.gps_sat_info.viewed_sat_num = 12;
        }
        else if (gps_data.gps_sat_info.viewed_sat_num == 0)
        {
            memset(gps_data.gps_sat_info.sat_info,0,24);
        }
    }
    rx_buf+= (len+1);

//Get signal strength parameter
//0,1,2,3
    for(cur_sat_num = 0; cur_sat_num < 4 ;cur_sat_num++) 
    {
        if ((cur_sat_num + 4 * (pack_num -1)) >=  gps_data.gps_sat_info.viewed_sat_num)
        {
            break;
        }

        p_temp = Get_Filed_Data_Pointer(original_p, 4 + 4 *cur_sat_num );
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len == 2)
        {
            gps_data.gps_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_num = (*p_temp- 0x30) * 10 +  *(p_temp+1)- 0x30;
        }
        else
        {
            gps_data.gps_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_num = 0;
        }

        p_temp = Get_Filed_Data_Pointer(original_p, 7 + 4 *cur_sat_num );
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len == 2)
        {
            gps_data.gps_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_sig = (*p_temp- 0x30) * 10 +  *(p_temp+1)- 0x30;      
        }
        else
        {
            gps_data.gps_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_sig = 0;      
        }
    }

    return true;   
}

static bool BD_BDGSV_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    uint8_t* p ;
    uint8_t* p_temp;
    uint8_t pack_num;
    uint8_t cur_sat_num;
//    DEBUG(DEBUG_MEDIUM,"[BDGSV]%s\n\r",rx_buf);

#if GPS_DEBUG_VALUE
    uint8_t test; //Move test here to define it along with GPS_DEBUG_VALUE.
    gps_debug_flag ++;

    if (gps_debug_flag == 1)
    {
        test = strlen(",3,1,09,41,51,61,71,42,52,62,72,43,53,63,73,44,54,64,74*AA");
        memcpy(rx_buf, ",3,1,09,41,51,61,71,42,52,62,72,43,53,63,73,44,54,64,74*AA",test);
    }
    else if (gps_debug_flag == 2)
    {
        test = strlen(",3,2,09,41,51,61,75,42,52,62,76,43,53,63,77,44,54,64,78*AA");
        memcpy(rx_buf, ",3,2,09,41,51,61,75,42,52,62,76,43,53,63,77,44,54,64,78*AA",test);
    }
    else if (gps_debug_flag == 3)
    {
        gps_debug_flag = 0;
        test = strlen(",3,3,09,41,51,61,79,42,52,62,01,43,53,63,02,44,54,64,03*AA");
        memcpy(rx_buf, ",3,3,09,41,51,61,79,42,52,62,01,43,53,63,02,44,54,64,03*AA",test);
    }
      
#endif

    original_p = rx_buf;
    rx_buf++; //elimite first comma.
    p = rx_buf;

    //First parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1);
    if((len >0) && (len <= 2))
    {
        if (len == 2)
        {
            //pack_sum = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        }
        else
        {      
            //pack_sum = *rx_buf - 0x30;
        }
    }
    rx_buf+= (len+1);

    //Second parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if((len >0) && (len <= 2))
    {
        if (len == 2)
        {
            pack_num = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        }
        else
        {
            pack_num = *rx_buf - 0x30;
        }
    }
    rx_buf+= (len+1);
   
    //Third parameter
    len = Get_Next_Data_Len_Before_Comma(p);
    p+=(len+1); //elimite first comma.
    if(len == 2)
    {
        gps_data.bd_sat_info.viewed_sat_num = (*rx_buf - 0x30) * 10 +  *(rx_buf+1) - 0x30;      
        if (gps_data.bd_sat_info.viewed_sat_num > 12) 
        {
            gps_data.bd_sat_info.viewed_sat_num = 12;
        }
        else if (gps_data.bd_sat_info.viewed_sat_num == 0)
        {
            memset(gps_data.bd_sat_info.sat_info,0,24);
        }
    }
    rx_buf+= (len+1);

    //Get signal strength parameter
    //0,1,2,3
    for(cur_sat_num = 0; cur_sat_num < 4 ;cur_sat_num++) 
    {
        if ((cur_sat_num + 4 * (pack_num -1)) >=  gps_data.bd_sat_info.viewed_sat_num)
        {
            break;
        }

        p_temp = Get_Filed_Data_Pointer(original_p, 4 + 4 *cur_sat_num );
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len == 2)
        {
            gps_data.bd_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_num = (*p_temp- 0x30) * 10 +  *(p_temp+1)- 0x30;
        }
        else
        {
            gps_data.bd_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_num = 0;
        }

        p_temp = Get_Filed_Data_Pointer(original_p, 7 + 4 *cur_sat_num );
        len = Get_Next_Data_Len_Before_Comma(p_temp);
        if (len == 2)
        {
            gps_data.bd_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_sig = (*p_temp- 0x30) * 10 +  *(p_temp+1)- 0x30;      
        }
        else
        {
            gps_data.bd_sat_info.sat_info[cur_sat_num + 4 * (pack_num -1)].sat_sig = 0;      
        }
    }

    return true;
}

static bool NMEA_GPGSA_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    uint8_t* p ;
//   DEBUG(DEBUG_MEDIUM,"[GPGSA]%s\n\r",rx_buf);

    original_p = rx_buf;
    rx_buf++; //elimite first comma.
    p = rx_buf;

    //First parameter
    p = Get_Filed_Data_Pointer(original_p, 2);
    len = Get_Next_Data_Len_Before_Comma(p);
    if((len >0) && (len <= 1))
    {
//        uint8_t gps_fixed=*p-'0';
        // reserved for 3d fix mode
    }

    p = Get_Filed_Data_Pointer(original_p, 15);
    len = Get_Next_Data_Len_Before_Comma(p);
    if(len >0)
    {
        convert_gps_dop(p,gps_data.pdop);
    }
    p+=(len+1);
    return true;
}

static bool BD_BDGSA_Msg_Decode(uint8_t * rx_buf)
{
    uint8_t len;
    uint8_t* original_p;
    uint8_t* p ;
//   DEBUG(DEBUG_MEDIUM,"[BDGSA]%s\n\r",rx_buf);

    original_p = rx_buf;
    rx_buf++; //elimite first comma.
    p = rx_buf;

    //First parameter
    p = Get_Filed_Data_Pointer(original_p, 2);
    len = Get_Next_Data_Len_Before_Comma(p);
    if((len >0) && (len <= 1))
    {
//        uint8_t gps_fixed=*p-'0';
        // reserved for 3d fix mode
    }

    p = Get_Filed_Data_Pointer(original_p, 15);
    len = Get_Next_Data_Len_Before_Comma(p);
    if((len >0) && (len <= 8))
    {
        gps_data.pdop[0]=*p-'0';
        gps_data.pdop[1]=(*(p+2)-'0')*10+(*(p+3)-'0');
    }
    p+=(len+1);
    return true;
}


#endif

bool GPS_Get_Ready_Flag(void)
{
	return gps_data_ready_flag;
}

void GPS_Send_Data(uint8_t *data, uint32_t len)
{
    uint32_t i;
    if (len > 1000)
        len = 1000;
    for (i=0;i<len;i++)
    {
        Uart_Put_Char(UART_GPS_CHANNEL,*(data + i ));
    }
}

uint16_t GPS_Parse_Cog(uint8_t *data)
{
    uint8_t i;
    uint16_t ret = 0;
    if (*data == 0)
        return 0;
    for(i=0;i<3;i++)
    {
        if (*(data+i) == 0)
            break;
        ret *= 10;
        ret += *(data+i)-'0';
    }
    return ret;
}

void GPS_Get_Sig(uint8_t *sig)
{
    uint8_t i;
    if (gps_data.gps_sat_info.viewed_sat_num == 0)
    {
        memset(sig, 0, 8);
        return;
    }
    for (i=0; i<12; i++)
    {
        if (gps_data.gps_sat_info.sat_info[i].sat_sig > sig[1])
        {
            sig[7] = sig[5];
            sig[6] = sig[4];
            sig[5] = sig[3];
            sig[4] = sig[2];
            sig[3] = sig[1];
            sig[2] = sig[0];
            sig[1] = gps_data.gps_sat_info.sat_info[i].sat_sig;
            sig[0] = gps_data.gps_sat_info.sat_info[i].sat_num;
        }
        else if (gps_data.gps_sat_info.sat_info[i].sat_sig > sig[3])
        {
            sig[7] = sig[5];
            sig[6] = sig[4];
            sig[5] = sig[3];
            sig[4] = sig[2];
            sig[3] = gps_data.gps_sat_info.sat_info[i].sat_sig;
            sig[2] = gps_data.gps_sat_info.sat_info[i].sat_num;
        }
        else if (gps_data.gps_sat_info.sat_info[i].sat_sig > sig[5])
        {
            sig[7] = sig[5];
            sig[6] = sig[4];
            sig[5] = gps_data.gps_sat_info.sat_info[i].sat_sig;
            sig[4] = gps_data.gps_sat_info.sat_info[i].sat_num;
        }
        else if (gps_data.gps_sat_info.sat_info[i].sat_sig > sig[7])
        {
            sig[7] = gps_data.gps_sat_info.sat_info[i].sat_sig;
            sig[6] = gps_data.gps_sat_info.sat_info[i].sat_num;
        }
    }
}

static void convert_gps_lon(uint8_t *gps_data, uint8_t *dst_buf)
{
    uint8_t i=0;
    uint8_t point_ptr = 0;
    uint32_t tmp_min = 0;
    for(i=0;i<(GPS_POS_DATE_LEN-1);i++)
    {
        if (*(gps_data+i) == '.')
        {
            point_ptr = i;
            break;
        }
    }
    if (0 == point_ptr)
    {
        memset(dst_buf,0,4);
    }
    else
    {
        uint32_t decimal_base;
        int32_t gps_min_dec = 0;
        int32_t gps_degree_dec = 0;
        tmp_min = (*(gps_data+point_ptr-2)-'0') * 10 + (*(gps_data+point_ptr-1) - '0');
        for (i=point_ptr+1;i<(point_ptr+7);i++)
        {
            if (*(gps_data+i) == '\0')
            {
                tmp_min *= 10;
            }
            else
            {
                tmp_min *= 10;
                tmp_min += (*(gps_data+i)-'0');
            }
        }
        gps_min_dec = (tmp_min) / 60;
        //*(dst_buf+1) = gps_min_dec/10000;
        //gps_min_dec = gps_min_dec % 10000;
        //*(dst_buf+2) = gps_min_dec/100;
        //*(dst_buf+3) = gps_min_dec%100;
        decimal_base = 1;
        for (i=(point_ptr-2);i>0;i--)
        {
            gps_degree_dec += (*(gps_data+i-1)-'0') * decimal_base;
            decimal_base *= 10;
        }
        gps_min_dec+=gps_degree_dec*1000000;
        *(dst_buf) = (gps_min_dec>>24) & 0xff;
        *(dst_buf+1) = (gps_min_dec >> 16) & 0xff;
        *(dst_buf+2) = (gps_min_dec >> 8) & 0xff;
        *(dst_buf+3) = gps_min_dec & 0xff;
    }
}

// Convert altitude to meter
static void convert_gps_alt(uint8_t *gps_data, uint8_t *dst_buf)
{
    uint8_t i=0;
    uint8_t point_ptr = 0;
    for(i=0;i<(GPS_POS_DATE_LEN-1);i++)
    {
        if (*(gps_data+i) == '.')
        {
            point_ptr = i;
            break;
        }
    }
    if (0 == point_ptr)
    {
        memset(dst_buf,0,4);
    }
    else
    {
        uint32_t decimal_base;
        int32_t gps_degree_dec = 0;
        decimal_base = 1;
        for (i=point_ptr-2;i<=point_ptr;i++)
        {
            gps_degree_dec += (*(gps_data+i-1)-'0') * decimal_base;
            decimal_base *= 10;
        }
        *(dst_buf) = (gps_degree_dec>>24) & 0xff;
        *(dst_buf+1) = (gps_degree_dec >> 16) & 0xff;
        *(dst_buf+2) = (gps_degree_dec >> 8) & 0xff;
        *(dst_buf+3) = gps_degree_dec & 0xff;
    }
}

// Convert speed to km/h
static void convert_gps_knot(uint8_t *gps_data, uint8_t *dst_buf)
{
    uint8_t i=0;
    uint8_t point_ptr = 0;
    for(i=0;i<(GPS_SPEED_DATE_LEN-1);i++)
    {
        if (*(gps_data+i) == '.')
        {
            point_ptr = i;
            break;
        }
    }
    if (0 == point_ptr)
    {
        memset(dst_buf,0,2);
    }
    else
    {
        uint32_t decimal_base;
        int32_t gps_spd_dec = 0;
//        int32_t gps_degree_dec = 0;
        for (i=point_ptr+1;i<(point_ptr+2);i++)
        {
            if (*(gps_data+i) == '\0')
            {
                gps_spd_dec *= 10;
            }
            else
            {
                gps_spd_dec *= 10;
                gps_spd_dec += (*(gps_data+i)-'0');
            }
        }
        gps_spd_dec *=100000;
        decimal_base = 1;
        for (i=0;i<point_ptr;i++)
        {
            gps_spd_dec += (*(gps_data+i)-'0') * decimal_base * 1000000;
            decimal_base *= 10;
        }
        gps_spd_dec=(gps_spd_dec*36)/(KNOTS_DIV*10);
        *dst_buf=(gps_spd_dec>>8)&0xff;
        *(dst_buf+1)=(gps_spd_dec)&0xff;
    }
}

static void convert_gps_cog(uint8_t *gps_data, uint8_t *dst_buf)
{
    uint8_t i=0;
    uint8_t point_ptr = 0;
    for(i=0;i<(GPS_COG_DATE_LEN-1);i++)
    {
        if (*(gps_data+i) == '.')
        {
            point_ptr = i;
            break;
        }
    }
    if (0 == point_ptr)
    {
        memset(dst_buf,0,2);
    }
    else
    {
        uint32_t decimal_base;
        int32_t gps_cog_dec = 0;
        decimal_base = 1;
//        for (i=point_ptr-1;i>0;i--)
        {
            gps_cog_dec += (*(gps_data+point_ptr-1)-'0') * decimal_base;
            decimal_base *= 10;
        }
        *dst_buf=(gps_cog_dec>>8)&0xff;
        *(dst_buf+1)=(gps_cog_dec)&0xff;
    }
}

static void convert_gps_dop(uint8_t *gps_data, uint8_t *dst_buf)
{
    uint8_t i=0;
    uint8_t point_ptr = 0;
    for(i=0;i<(GPS_COG_DATE_LEN-1);i++)
    {
        if (*(gps_data+i) == '.')
        {
            point_ptr = i;
            break;
        }
    }
    if (0 == point_ptr)
    {
        memset(dst_buf,0,2);
    }
    else
    {
        uint32_t decimal_base;
        int32_t gps_dop_dec = 0;
        decimal_base = 10;
        gps_dop_dec=(*(gps_data+point_ptr+1)-'0');
//        for (i=point_ptr-1;i>0;i--)
        {
            gps_dop_dec += (*(gps_data+point_ptr-1)-'0') * decimal_base;
//            decimal_base *= 10;
        }
//        gps_dop_dec*=10;
        *dst_buf=(gps_dop_dec>>8)&0xff;
        *(dst_buf+1)=(gps_dop_dec)&0xff;
    }
}

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/

