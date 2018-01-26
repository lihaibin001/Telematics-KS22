/**********************************************************************
   Title                      : diag_task.h                                  
                                                                         
   Module Description         :                                      
                                                                       
   Author                     :                          
                                                                       
   Created                    :                                                
                                                                       
   Configuration ID           :                                     
                                       
 *********************************************************************/

/*---------------------------------------------------------------------  
 *   Instructions for using this module if any:
 *
 *-------------------------------------------------------------------*/  
#ifndef  DIAG_TASK_H
#define  DIAG_TASK_H 1
/*********************************************************************/
/* Include User Header file                                          */
/*********************************************************************/
#include    "standard.h"                           

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
/*USER DEFINITION*/


/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef  enum  Diag_Events_Tag
{
	DIAG_EVT_NOP,
    DIAG_EVT_UDS_EVT,
	//DIAG_EVT_DTC_
   	DIAG_NUM_EVENTS
} Diag_Events_Type;

typedef enum
{
    DIAG_EVT_UDS_SEND_VIN = 0,
    DIAG_EVT_UDS_WRITW_VIN_1,
    DIAG_EVT_UDS_WRITW_VIN_2,
    DIAG_EVT_UDS_CNT
}Diag_INTER_EVT_Type;

typedef struct Self_Diag_Tag //
{
    uint8_t sim_ready; //
    uint8_t network_connected; //
    uint8_t net_data_sent; //
    uint8_t light_sensor; //
    uint16_t voltage; //
    uint8_t temp; //
    uint8_t gps_fixed; //
    uint8_t flash_ok; //
} Self_Diag_T;

/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/
extern void Diag_Task (void *pvParameters);
extern void diag_get_result(Self_Diag_T *result);
extern void diag_set_flash_result(uint8_t result);

#endif
/**********************************************************************
 *                                                                 
 * REVISION RECORDS                                                
 *                                                                 
 **********************************************************************/
/*********************************************************************/
/* $HISTROY$
 *
 *********************************************************************/
