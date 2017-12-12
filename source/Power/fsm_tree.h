/*===========================================================================*/
/**
 * @file fsm_tree.h
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
 *
 * DEVIATIONS FROM STANDARDS:
 *   - @todo List of deviations from standards in this file, or "None".
 *
 */
/*==========================================================================*/
#ifndef FSM_TREE_H
#define FSM_TREE_H


#undef TREE
#undef STATE
#undef STATE_END
#undef TRANS
#undef TREE_END

#define TREE(n) const tree_type n []={
#define STATE(s,p,cs) {p,(uint8t_fptr)cs,s##_,sizeof(s##_)/sizeof(trans_type)},
#define STATE_END
#define TRANS(e,n,a)
#define TREE_END(n) };

/*===========================================================================*\
 * File Revision History (top to bottom: last revision to first revision)
 *===========================================================================
 *
 *
\*===========================================================================*/
#endif
