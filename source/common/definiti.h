/*Header$*/

#ifndef  DEFINITION_H_FILE
#define  DEFINITION_H_FILE 1

/**********************************************************************
 *       Title:   definition.h
 *
 *  Description:  Gloabl defines of miscellaneous useful stuff.
 *
 *      Author:   
 *
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "stdint.h"
#include "stdbool.h"
/**********************************************************************
 * Global Constant and Macro Definitions using #define                        
 *********************************************************************/
#define  To_Boolean(bit)            ((bit) ? true : false)                 /* convert bit/logical to normalized boolean */
#define  Bit_Copy(target, source)   (target = ((source) ? true : false))   /* set target equal to source */
#define  Bit_Toggle(target)         (target = ((target) ? false : true))   /* toggle target              */
#define  FOREVER                    for(;;)

#define icat(x, y) x ## y
#define iins(x, y, z) x ## y ## z
#define cat(x, y)  icat(x,y)
/*
 * DESCRIPTION: macros to append symbols together
 * ---------------------------------------------------------------- */

#ifndef  UP
#define  DOWN           false
#define  UP             !DOWN
#endif

#ifndef  OFF
#define  OFF             false
#define  ON              !OFF
#endif

//#ifndef  CLEARED
//#define  CLEARED         false
//#define  SET             !CLEARED
//#endif

/*
 * DESCRIPTION: Standard boolean labels
 * ---------------------------------------------------------------- */

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

typedef enum                   // Return status from RTOS functions
   {
      OS_E_OK             =  0,                    // NO Error, successful
      OS_E_ERROR,                                  // General Error
      OS_E_OS_ACCESS,
      OS_E_OS_CALLEVEL,
      OS_E_OS_ID,
      OS_E_OS_LIMIT,
      OS_E_OS_NOFUNC,
      OS_E_OS_RESOURCE,
      OS_E_OS_STATE,
      OS_E_OS_VALUE,
      OS_E_COM_NOMSG,
      OS_E_COM_LOCKED,
      OS_E_TASK_SUSPENDED,
      OS_E_TIMEOUT,                                // Error due to timeout
      OS_E_OUT_OF_RANGE,                           // Error due to parameter out of range
      OS_E_DATAEXISTS,                     /* data to be transferred next exists in TXBn register */
      OS_E_INVALID_CONDITION                       // Error due to invalid conditions
   } Status_Type;

typedef  void (*void_fptr) (void);                 // void void Function pointer typedef
typedef  bool (*bool_fptr) (void);                 // boolean void function typedef
typedef  void (*void_int16_fptr) (int16_t data);   // void funtion taking 16 bit integer

/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/

/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/

/**********************************************************************
 *
 * Modification Record
 *
 *********************************************************************
 * 
 *********************************************************************/
 
#endif
