/**********************************************************************
                Title : standard.h

   Module Description : This file contains all standard include statements

               Author : 

 *********************************************************************/
#ifndef _STANDARD_H_
#define _STANDARD_H_
/**********************************************************************
 * Standard defines
 *********************************************************************/

/**********************************************************************
 * Compiler / Part Specific Defines
 *********************************************************************/
#include "compiler.h"

/*********************************************************************
 * Standard C Library
 *********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/**********************************************************************
 *  Includes of System setup header
 *********************************************************************/

/**********************************************************************
 * Includes of standard C definitions (Compiler / micro independent)
 *********************************************************************/

/**********************************************************************
 * Includes of I/O assignments 
 *********************************************************************/

/**********************************************************************
 * Includes operating system definitions
 *********************************************************************/
#include "rtos.h"
#include "rtos_ext.h"
/**********************************************************************
 * General System includes
 *********************************************************************/
#include "gps.h"
#include "prj_config.h"
#include "int2evt.h"  
#include "timers.h"
#include "lowpower.h"
#include "gensubs.h"
#include "main.h"
#include "regulator.h"
#include "psync.h"
#include "relays.h"
#include "diag_task.h"
#include "ring_buf.h"
#include "delay.h"
#include "spi_flash_freertos.h"
#include "record_task.h"
#include "crc_ccitt.h"
#include "str_lib.h"
#include "timer.h"
#include "wdog.h"
#include "adc.h"

/**********************************************************************
 * Application specific Header files
 *********************************************************************/
#include "periodic.h"
#include "powerfail.h"
#include "GPRS.h"
#include "TelmProtocol.h"
#include "system.h"
#include "board.h"
#include "vehicle.h"
#endif //(#ifndef _STANDARD_H_)
/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/

/**********************************************************************

 *********************************************************************/

