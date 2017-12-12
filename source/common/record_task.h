#ifndef _RECORD_TASK_H_
#define _RECORD_TASK_H_
/**********************************************************************
   Title                      : record_task.h         
                                                                         
   Module Description         : This file is the task for data records
	
   Author                     : 
   
 *********************************************************************/
/**********************************************************************
 * define
 *********************************************************************/
#define LEVEL3_ALARM_ASSERT     (uint8_t)0
#define LEVEL3_ALARM_DEASSERT   (uint8_t)1

typedef enum record_event_tag
{
    RECORD_EVT_NOP,
    RECORD_EVT_RLTM_DATA_REFRESH,
    RECORD_EVT_ALARM,
    RECORD_EVT_CLEAN_RECORD,
    RECORD_NUM_EVENTS
}record_event_t;
typedef struct
{
    Telm_Data_Vehicle vhclInifo;    //vehicle data
    Telm_Data_Ec_Engine mtrInfo;    //motor data
    Telm_Data_Position gpsInfo;     //gps data
    Telm_Data_Abs_Val extrValInfo;  //extreme value data
    Telm_Data_Alarm alrmInfo;       //alarm data
    Telm_Data_Batt_Volt batVoltInfo;//the voltage of the power battery
    Telm_Data_Batt_Temp batTempInfo;//the temperature of the power battery
    Telm_Data_Extended extInof;     //extend data
//    uint8_t packetNum;
}realTimeInfo_t;
/*********************************************************************/
/* interface for modules                                             */
/*********************************************************************/

extern void Record_Task(void* pvParameters);
extern uint8_t device_unmount_status(void);
TelmRealtime_t *Record_GetRecord(void);
realTimeInfo_t *Record_GetLastRltm(void);
/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif //_GPS_H_
