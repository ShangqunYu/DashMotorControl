/*
 * fsm.h
 *
 * Motor controller finite-state machine.
 * Adapted from Ben Katz's original fsm.h; state constants replaced by the
 * motor_state enum in PMSM_motor.h.
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_


#include <stdint.h>
#include "PMSM_motor.h"   /* motor_state */

/* ── FSM state struct ─────────────────────────────────────────────────────
 * curr_state : currently executing state
 * next_state : requested next state (written by CAN ISR via motor.pending_fsm_cmd)
 */
typedef struct {
    motor_state curr_state;
    motor_state next_state;
} FSMStruct;

/* ── Public API ──────────────────────────────────────────────────────────── */
void run_fsm(FSMStruct *fsmstate);
void fsm_enter_state(FSMStruct *fsmstate);
void fsm_exit_state(FSMStruct *fsmstate);
void enable_motor(void);
void disable_motor(void);

#endif /* INC_FSM_H_ */
