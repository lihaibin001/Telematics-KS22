#ifndef INP2EVT_H
#   define INP2EVT_H
/*===========================================================================*/
/**
 * @file inp2evt.h
 *
 * @todo Add a one line description of the header file here.
 *
 * DESCRIPTION:
 *
 * @todo Add full description here
 *
 * ABBREVIATIONS:
 *   - @todo List any abbreviations, precede each with a dash ('-').
 *
 * TRACEABILITY INFO:
 *   - Design Document(s):
 *     - @todo Update list of design document(s).
 *
 *   - Requirements Document(s):
 *     - @todo Update list of requirements document(s)
 *
 * DEVIATIONS FROM STANDARDS:
 *   - @todo List of deviations from standards in this file, or "None".
 *
 * @defgroup inp2evt Provide API description and define/delete next line
 * @ingroup <parent_API> (OPTIONAL USE if part of another API, else delete)
 * @{
 */
/*==========================================================================*/

/*===========================================================================*\
 * Header Files
\*===========================================================================*/

/*===========================================================================*\
 * Exported Preprocessor #define Constants
\*===========================================================================*/

/*===========================================================================*\
 * Exported Preprocessor #define MACROS
\*===========================================================================*/

/*===========================================================================*\
 * Exported Type Declarations
\*===========================================================================*/
typedef struct INPUT_REC_TAG
{
   Message_Type   msg;
   uint16_t       mode;/*the specific system message handling is allowed only current source is 'mode'*/
   int16_t        code;
} INPUT_REC;

typedef struct INP2EVT_TABLE_TAG
{
   const INPUT_REC   *input_table;
   int16_t            input_size;
}  INP2EVT_TABLE;


/*---------------------------------------------------------------------
 * The following type information is for the input routines called by
 * Post Office
 *-------------------------------------------------------------------*/
typedef struct Input_States_Tag
{
   uint8_t keyboard;
   uint8_t index;  
} Input_States_T;    

typedef  void (*Input_Routine) (Input_States_T*, Data_Message_T);  
                                                // funtional prototype of an input routine

/*===========================================================================*\
 * Exported Const Object Declarations
\*===========================================================================*/

/*===========================================================================*\
 * Exported Function Prototypes
\*===========================================================================*/
int16_t Input_To_Event(Input_States_T *index, Message_Type msg, uint16_t mode, const INP2EVT_TABLE *inp2evt_table);
/*===========================================================================*\
 * Exported Inline Function Definitions and #define Function-Like Macros
\*===========================================================================*/

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
 * 
\*===========================================================================*/
/** @} doxygen end group */
#endif /* INP2EVT_H */

