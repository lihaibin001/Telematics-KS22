/*===========================================================================*/
/**
 * @file fsm.h
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
 * @defgroup fsm Provide API description and define/delete next line
 * @ingroup <parent_API> (OPTIONAL USE if part of another API, else delete)
 * @{
 */
/*==========================================================================*/
#ifndef FSM_H
#define FSM_H

/*===========================================================================*\
 * Header Files
\*===========================================================================*/
#include "fsm_acfg.h"
#include "rtos.h"

/*===========================================================================*\
 * Exported Preprocessor #define Constants
\*===========================================================================*/

/*===========================================================================*\
 * Exported Preprocessor #define MACROS
\*===========================================================================*/

/* special event keywords */
#define START             0xFFFD //253
#define ENTRY             0xFFFE //254
#define EXIT              0xFFFF //255

/* special next state keywords */
#define CONDITION         253
#define NEXT              254
#define INTERNAL          255

/* special return values */
#define EVENT_NOT_VALID   253
#define CS_NOT_VALID      254                
#define TREE_NOT_VALID    255

/*===========================================================================*\
 * Exported Type Declarations
\*===========================================================================*/

typedef uint8_t (*uint8t_fptr)(void);          

typedef struct
{
  uint16_t    event;
  uint8_t     next_state;
  uint8t_fptr action;
} trans_type;

typedef struct
{
  uint8_t       parent_state;
  uint8t_fptr   cs_action;
  const trans_type *trans_tab;
  uint8_t       trans_len;
} tree_type;        

/*===========================================================================*\
 * Exported Const Object Declarations
\*===========================================================================*/

/*===========================================================================*\
 * Exported Function Prototypes
\*===========================================================================*/

uint8_t FSM_Process_Evt (Data_Message_T msg, uint8_t current_state, tree_type const *tree_ptr);
void    FSM_Process_CS  (uint8_t current_state, tree_type const *tree_ptr);

//uint8_t no_action (void);

/*===========================================================================*\
 * Exported Inline Function Definitions and #define Function-Like Macros
\*===========================================================================*/

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
\*===========================================================================*/
/** @} doxygen end group */
#endif /* FSM_H */
