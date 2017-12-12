#ifndef  Ring_Buf_H_FILE
#define  Ring_Buf_H_FILE 1

/**********************************************************************
 *
 *
 *********************************************************************/
/**********************************************************************
 *             Title:   Ring_Buf.H
 *
 *       Description:   
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

/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/
typedef struct Ring_Buf_Type_Tag
{
   uint8_t    in;            /* index of next array element to add to */
   uint8_t    out;           /* index of oldest element to remove */
   uint8_t    size;          /* number of elements in ring */
   bool  full;          /* Flag to indicate between full and empty buffer */
} Ring_Buf_Type;

/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/
 
extern void Ring_Buf_Reset(Ring_Buf_Type *buf, uint8_t size);
/**********************************************************************
 *    Purpose: Reset / Initialize ring buffer to empty
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/

extern bool Ring_Buf_Is_Empty(Ring_Buf_Type *buf);
/**********************************************************************
 *    Purpose: returns whether buffer is empty
 * Parameters: Pointer to ring buf control structure
 *    Returns: TRUE if buffer is empty
 *********************************************************************/

extern bool Ring_Buf_Is_Full(Ring_Buf_Type *buf);
/**********************************************************************
 *    Purpose: returns wether buffer is full
 * Parameters: Pointer to ring buf control structure
 *    Returns: TRUE if buffer is full
 *********************************************************************/

extern void Ring_Buf_Add(Ring_Buf_Type *buf);
/**********************************************************************
 *    Purpose: Logical adds element to ring buffer 
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/

extern void Ring_Buf_Remove(Ring_Buf_Type *buf);
/**********************************************************************
 *    Purpose: Logical remove element  from ring buffer 
 * Parameters: Pointer to ring buf control structure
 *    Returns: None
 *********************************************************************/

/**********************************************************************
 *
 * Revision Record
 *
 **********************************************************************
 * 
 * 
 *********************************************************************/
#endif
