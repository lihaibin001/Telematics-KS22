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
   Title                    : DELAY.C

   Module Description       : This is the standard code file for DELAY.

   Author                   : 
   Created                  : 

 *********************************************************************/

/**********************************************************************
 * Include header files                                                
 *********************************************************************/
#include "standard.h"

#ifndef configSYSTICK_CLOCK_HZ
    #error "cpu clock speed undefined."
#endif 
/*===========================================================================*\
 * Function Prototypes for Private Functions with File Level Scope
\*===========================================================================*/

/*===========================================================================*\
 * ROM Const Variables With File Level Scope
\*===========================================================================*/


/*===========================================================================*\
 * Function Definitions
\*===========================================================================*/

/*===========================================================================*\
 *    Function: uDelay
 *
 *  Parameters: number of uSec to DELAY
 *
 *     Returns: void
 *
 * Description: This routine will DELAY at least the amount of time in uSec
 *              passes in as the DELAY time
 *
\*===========================================================================*/
void uDelay(uint32_t usec)
{
	static uint32_t i;
	while(usec--)
	{
		i=1;
		while(i--);
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
	}
}

/*===========================================================================*\
 * File Revision History
 *===========================================================================
 *
 * Date         userid    (Description on following lines)
 * -----------  --------  ------------
 *
\*===========================================================================*/
