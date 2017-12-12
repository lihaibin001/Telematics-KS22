/* $Header:   ps_stt.h     $*/
/***********************************************************************

   Title                      : ps_stt.h

   Module Description         : Contains the state transition table
                                for power/operation mode synchronization
                                . This file is included at 3
                                different positions in psync.c and is used
                                to let the precompiler build 3 different tables
                                (1 enum, 2 arrays).

   Author                     : 

   Created                    : 

   Configuration ID           : 

 **********************************************************************/
TREE (tree_psync)

/*========================== 0 - PS_ROOT =========================*/

STATE (PS_ROOT,				0,				ps_cs_root)
TRANS (ENTRY,				INTERNAL,		ps_entry_root)
TRANS (START,				PS_ROOT,		ps_start_action)
STATE_END

/*========================== 1 - PS_IDLE =========================*/

STATE (PS_IDLE,				PS_ROOT,		ps_cs_idle)
TRANS (ENTRY,				INTERNAL,		ps_entry_idle)
TRANS (PS_EVT_AWAKE,		PS_AWAKE,		no_action)
STATE_END

/*========================== 2 - PS_AWAKE ========================*/

STATE (PS_AWAKE,			PS_ROOT,		ps_cs_awake)
TRANS (ENTRY,				INTERNAL,		ps_entry_awake)

TRANS (PS_EVT_DEV_ON,		CONDITION,		ps_check_dev_active)
TRANS (PS_EVT_DEV_ON,		INTERNAL,		no_action)
TRANS (PS_EVT_DEV_ON,		PS_USER_ON,     no_action)

TRANS (PS_EVT_IGN_ON,		CONDITION,		ps_check_dev_active_user_on)
TRANS (PS_EVT_IGN_ON,		INTERNAL,		no_action)
TRANS (PS_EVT_IGN_ON,		PS_ENG_ON,      no_action)

TRANS (PS_EVT_IGN_OFF,		INTERNAL,		ps_start_stdby_tmr)

TRANS (PS_EVT_IDLE,			CONDITION,		ps_check_can_sleep)
TRANS (PS_EVT_IDLE,			PS_IDLE,	    no_action)
TRANS (PS_EVT_IDLE,			INTERNAL,		ps_restart_stdby_tmr)

TRANS (PS_EVT_GO_IDLE,		PS_IDLE,		no_action)

TRANS (EXIT,			    INTERNAL,		ps_exit_awake)
STATE_END

/*========================== 3 - PS_SYS_ON =======================*/

STATE (PS_SYS_ON,			PS_ROOT,		ps_cs_sys_on)
TRANS (ENTRY,				INTERNAL,		ps_entry_sys_on)
TRANS (PS_EVT_GO_IDLE,		PS_AWAKE,		no_action)
STATE_END

/*========================== 4 - PS_USER_ON ======================*/

STATE (PS_USER_ON,			PS_SYS_ON,		ps_cs_user_on)
TRANS (ENTRY,				INTERNAL,		ps_entry_user_on)
TRANS (PS_EVT_IGN_ON,		INTERNAL,		no_action)

TRANS (PS_EVT_IGN_OFF,		PS_AWAKE,		ps_set_ign_on_to_awake)
//TRANS (PS_EVT_IGN_OFF,		INTERNAL,		no_action)

TRANS (PS_EVT_DEV_ON_TMR_EXP, NEXT,	no_action)

TRANS (PS_EVT_DEV_OFF,	CONDITION,		ps_check_user_mode_condition)
TRANS (PS_EVT_DEV_OFF,	PS_AWAKE,		ps_set_force_user_off)
TRANS (PS_EVT_DEV_OFF,	PS_AWAKE,		ps_set_user_off_start_stdby_tmr)

TRANS (PS_EVT_ENG_ON,	PS_ENG_ON,		ps_stop_dev_on_tmr)

TRANS (EXIT,			INTERNAL,       ps_exit_user_on)
STATE_END

/*========================== 5 - PS_ENG_ON =====================*/

STATE (PS_ENG_ON,			PS_SYS_ON,		ps_cs_eng_on)
TRANS (ENTRY,				INTERNAL,		ps_entry_eng_on)
//TRANS (PS_EVT_ENG_OFF,		PS_AWAKE,	    no_action)
TRANS (PS_EVT_IGN_OFF,		PS_AWAKE,	    no_action)
TRANS (EXIT,				INTERNAL,		ps_exit_eng_on)
STATE_END
TREE_END (tree_psync)

/**********************************************************************
*                                                                     *
* REVISION RECORDS                                                    *
*                                                                     *
**********************************************************************/
/*********************************************************************/
/* $Log:   ps_stt.h  $
 *
 *********************************************************************/
