/*===========================================================================*/
/**
 * @file fsm_tran.h
 *
 * @todo Add a one line description of the implementation.
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
#ifndef FSM_TRAN_H
#define FSM_TRAN_H

#undef TREE
#undef STATE
#undef STATE_END
#undef TRANS
#undef TREE_END

#define TREE(n)
#define STATE(s,p,a) const trans_type s##_[]={
#define STATE_END };
#define TRANS(e,n,a) {e,n,(uint8t_fptr)a},
#define TREE_END(n)

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
 *
\*===========================================================================*/
#endif
