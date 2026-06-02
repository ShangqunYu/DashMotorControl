/*
 * fsm.h
 *
 * Motor controller finite-state machine.
 * Adapted from Ben Katz's original fsm.h; state constants replaced by the
 * motor_state enum in foc.h.
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "foc.h"   /* motor_state */

/* ── Serial command characters ───────────────────────────────────────────── */
#define MENU_CMD    27    /* ESC – return to menu from any state */
#define MOTOR_CMD   'm'   /* enter torque control mode */
#define CAL_CMD     'c'   /* start encoder calibration */
#define ENCODER_CMD 'e'   /* enter encoder display mode */
#define SETUP_CMD   's'   /* enter setup / parameter config */
#define ZERO_CMD    'z'   /* capture current position as mechanical zero */
#define ENTER_CMD   13    /* carriage return – confirm setup entry */

/* ── CAN MIT command buffer ───────────────────────────────────────────────
 * Written by the CAN ISR, consumed by run_fsm every FOC cycle.
 * Mode transitions go through FSMStruct.next_state directly (not here).
 */
typedef struct {
    float   des_pos;
    float   des_vel;
    float   kp;
    float   kd;
    uint8_t mit_pending;   /* 1 = new MIT params waiting */
} can_cmd_t;

extern volatile can_cmd_t g_can_cmd;

/* ── FSM state struct ─────────────────────────────────────────────────────
 * state      : currently executing state
 * next_state : requested next state (set from CAN ISR or update_fsm)
 * ready      : 1 once fsm_exit_state has finished cleanup; gate for transition
 */
typedef struct {
    motor_state state;
    motor_state next_state;
    uint8_t     ready;
    char        cmd_buff[8];
    char        bytecount;
    char        cmd_id;
} FSMStruct;

/* ── Public API ──────────────────────────────────────────────────────────── */
void run_fsm(FSMStruct *fsmstate);
void update_fsm(FSMStruct *fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct *fsmstate);
void fsm_exit_state(FSMStruct *fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void process_user_input(FSMStruct *fsmstate);

#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
