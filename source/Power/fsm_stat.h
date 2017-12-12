/*===========================================================================*/
/**
 * @file fsm_stat.h
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
#ifndef FSM_STAT_H
#define FSM_STAT_H

#undef TREE
#undef STATE
#undef STATE_END
#undef TRANS
#undef TREE_END

#define TREE(n) typedef enum n##_states { /* for debugging */
/* #define TREE(n) enum n##_states {      // for finite product */
#define STATE(s,p,a) s,
#define STATE_END
#define TRANS(e,n,a)
/* #define TREE_END(n) dummy_##n}; */
#define TREE_END(n) dummy_##n} n##_states_type;

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
\*===========================================================================*/
#endif
