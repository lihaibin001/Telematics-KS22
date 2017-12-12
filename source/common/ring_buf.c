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
 *             Title:   Ring_Buffer.C
 *
 *       Description:   
 *
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
/*                                    */
/*********************************************************************/
#include   "compiler.h"        /* include type definitions */
#include   "ring_buf.h"
#include    "standard.h"
/*********************************************************************
 * File level pragmas
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/

/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/

/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/


/**********************************************************************
 * Add User defined functions
 *********************************************************************/


/**********************************************************************
 * Function Definitions
 *********************************************************************/


/**********************************************************************
 *    Purpose: Reset / Initialize ring buffer to empty
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/
extern void Ring_Buf_Reset(Ring_Buf_Type *buf, uint8_t size)
{
   buf->in = 0;
   buf->out = 0;
   buf->full = false;
   buf->size = size;
}

/**********************************************************************
 *    Purpose: returns whether buffer is empty
 * Parameters: Pointer to ring buf control structure
 *    Returns: true if buffer is empty
 *********************************************************************/
extern bool Ring_Buf_Is_Empty(Ring_Buf_Type *buf)
{
   return( !buf->full && (buf->in == buf->out));
}

/**********************************************************************
 *    Purpose: returns wether buffer is full
 * Parameters: Pointer to ring buf control structure
 *    Returns: true if buffer is full
 *********************************************************************/
extern bool Ring_Buf_Is_Full(Ring_Buf_Type *buf)
{
   return (buf->full);
}

/**********************************************************************
 *    Purpose: Logical adds element to ring buffer
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/
extern void Ring_Buf_Add(Ring_Buf_Type *buf)
{
   if (!buf->full)
   {
      buf->in++;                          // increment in index
      if (buf->in >= buf->size)           // wrapping as necessary
      {
         buf->in = 0;
      }
      buf->full = (buf->in == buf->out);  // if in pointer reaches out then buffer is full
   }
}

/**********************************************************************
 *    Purpose: Logical remove element  from ring buffer
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/
extern void Ring_Buf_Remove(Ring_Buf_Type *buf)
{
   if (!Ring_Buf_Is_Empty(buf))
   {
      buf->out++;                         // incrment out index
      if (buf->out >= buf->size)          // wrapping as necessary
      {
         buf->out = 0;
      }
      buf->full = false;                  // can not be full if we just removed one
   }
}

/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 * Intial Version
 *
 *********************************************************************/
