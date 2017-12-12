/*===========================================================================*/
/**
 * @file fsm.c
 *
 * @todo Add a one line description of the implementation.
 *
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
 */
/*==========================================================================*/

/*===========================================================================*\
 * Header Files
\*===========================================================================*/
#include "compiler.h"
#include "definiti.h"
#include "fsm.h"

/*===========================================================================*\
 * Local Preprocessor #define Constants
\*===========================================================================*/

/*===========================================================================*\
 * Local Preprocessor #define MACROS
\*===========================================================================*/

/*===========================================================================*\
 * Local Type Declarations
\*===========================================================================*/

/*===========================================================================*\
 * Exported Const Object Definitions
\*===========================================================================*/

/*===========================================================================*\
 * Local Object Definitions
\*===========================================================================*/

/*===========================================================================*\
 * Local Function Prototypes
\*===========================================================================*/

/*===========================================================================*\
 * Local Inline Function Definitions and Function-Like Macros
\*===========================================================================*/

/*===========================================================================*\
 * Function Definitions
\*===========================================================================*/

/************************************************************************
*                                                                       *
* FUNCTION      :   FSM_Process_Evt                                     *
*                                                                       *
* DESCRIPTION   :   starting at current state, search for event in      *
*                   transition list of this state. if not found go to   *
*                   parent state and look there. If found then do       *
*                   transition action and go to next state. the search  *
*                   for event continues until root of tree is reached.  *
*                                                                       *
* PARAMETERS    :   msg: message contains the event and data that is    *
*                   passed with the event                               *
*                    msg.part.msg:  input event to stimulate tree g     *
*                    msg.part.data: 2 byte parameter for the transition *
*                                   action (optional)                   *
*                   current_state: cs to start the search               *
*                   tree_ptr:      pointer to the tree                  *
*                                                                       *
* RETURN        :   new current state                                   *
*                                                                       *
************************************************************************/
uint8_t FSM_Process_Evt(Data_Message_T msg, uint8_t current_state, tree_type const * tree_ptr)
{
   bool        found;
   uint8_t     i;
   uint8_t     length;
   uint8_t     search_state;
   uint8_t     new_current_state;
   trans_type  const * trans_ptr;
   

   #if CONDITION_OPTION_IS
   uint8_t condition_offset;
   #endif

   #if EXIT_OPTION_IS
   uint8_t exits[MAX_EXITS];
   uint8_t nr_of_exits = 0;
   #endif

   found = false;
   search_state      = current_state;
   new_current_state = current_state;

   if (tree_ptr == NULL)
   {
      return(TREE_NOT_VALID);
   }

   do
   {
      trans_ptr = tree_ptr[search_state].trans_tab;
      length    = tree_ptr[search_state].trans_len;

      #if EXIT_OPTION_IS      /*save state when exit action is defined */
      if ((trans_ptr != NULL) && (trans_ptr[length-1].event == EXIT))
      {
         exits[nr_of_exits++] = search_state;  /* save index number if exit is found */
      }
      #endif /*EXIT_OPTION_IS */

      /************************************************************************ */
      /****      search in the state for the transition                     *** */
      /************************************************************************ */

      i = 0;                               /* index of first transition */

      while (i < length)
      {
         if (trans_ptr->event == msg.parts.msg)
         {
            #if NEXT_OPTION_IS
            while (trans_ptr->next_state == NEXT)     /*like next transition !!! */
            {
               trans_ptr++;
               i++;
            }
            #endif
            #if CONDITION_OPTION_IS
            if (trans_ptr->next_state == CONDITION)   /* condition ? */
            {
                // condition_offset = (*trans_ptr->action)(msg.parts.data); /*execute the condition check action */
               condition_offset = (*trans_ptr->action)(); /*execute the condition check action */
               trans_ptr += condition_offset;
            }
            #endif
            found = true;
            break;
         }
         trans_ptr++;
         i++;
      } /* end search event loop */

      /************************************************************************ */
      /****      execute all exit, transition and entry actions             *** */
      /************************************************************************ */

      if (found)
      {
         if (trans_ptr->next_state == INTERNAL)
         {
            /********** execute transition action only ************** */
            (*trans_ptr->action)();        /*execute the desired function */
            //(*trans_ptr->action)(msg.parts.data);        /*execute the desired function */
         }
         else /* if (trans_ptr->next_state != INTERNAL) */
         {
            #ifdef EXIT_OPTION_IS
            /********** execute all exit actions first ***************** */

            if (nr_of_exits)                 /* EXIT handling */
            {
               i=0;
               while (i < nr_of_exits)
               {
                  length = tree_ptr[exits[i]].trans_len;
                  tree_ptr[exits[i]].trans_tab[length-1].action();
                  i++;
               }
            }
            #endif

            /********** execute then the transition action **************** */
            (*trans_ptr->action)();        /* execute the desired function */

           // (*trans_ptr->action)(msg.parts.data);        /* execute the desired function */

            /********** set new current state ***************************** */

            new_current_state = trans_ptr->next_state;   /* set current to next state */

            /********** execute all entry actions now ********************* */

            do                            /* do entry actions until at leaf */
            {
               i = new_current_state;     /* i now used to check change of current state,   */
                                          /* that might be changed inside the entry actions */
                                          /* default, history, deep history                 */

               trans_ptr = tree_ptr[i].trans_tab;
               if (trans_ptr[0].event == ENTRY)
               {
                  new_current_state = trans_ptr[0].action();
               }
            } while (i != new_current_state); /* cs was set new in entry action                 */
                                              /* so next states entry action has to be checked  */
         }
      } /*** end if found ***/

      else /* if (!found) */
      {
         if (search_state)  /*search_state is not yet the root(=0) of the tree */
         {
            /* for next round in loop -> start again in the parent state */
            search_state = tree_ptr[search_state].parent_state;
         }
         else               /*if root reached and evt not found -> end */
         {
            break;
         }
      }

  } while(!found);

  return (new_current_state);
}

/************************************************************************
*                                                                       *
* FUNCTION      :   FSM_Process_CS                                      *
*                                                                       *
* DESCRIPTION   :   starting at current state, executes the cs_routine  *
*                   of this state. Then go to parent state and execute  *
*                   this cs_routine. This continues until root of tree  *
*                   is reached.                                         *
*                                                                       *
* PARAMETERS    :   current_state: cs to start with execution           *
*                   tree_ptr:      pointer to the tree                  *
*                                                                       *
* RETURN        :   none                                                *
*                                                                       *
************************************************************************/
void FSM_Process_CS (uint8_t current_state, tree_type const *tree_ptr)
{
   uint8_t search_state = current_state;

   for(;;)
   {
      tree_ptr[search_state].cs_action();

      if (search_state)         /* search_state was not yet root state (=0) */
      {
         search_state = tree_ptr[search_state].parent_state;
      }
      else                      /* search state was root state, now finish */
      {
         break;
      }
   }
}

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
\*===========================================================================*/


