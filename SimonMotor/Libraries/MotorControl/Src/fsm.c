/*
 * fsm.c
 *
 * Motor controller finite-state machine.
 * Adapted from Ben Katz's original fsm.c; control calls updated to match
 * this project's FOC API.
 */

#include "fsm.h"
#include "PMSM_motor.h"
#include "foc_calibration.h"
#include "analog_sensor.h"
#include "angle_sensor.h"
#include "drv8353.h"
#include "user_config.h"
#include "hw_config.h"
#include "usart.h"
#include "preference_writer.h"
#include "can.h"           /* pending_save: request a flash write from the mgr task */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ── Globals owned by main.c ─────────────────────────────────────────────── */
extern PMSM_motor       motor;
extern CalStruct        hcal;

/* ── check_safety ────────────────────────────────────────────────────────────
 * Per-cycle watchdog for the active-drive modes. Advances the timeout counters
 * and returns the first tripped fault (FAULT_NONE if healthy). Must be called
 * exactly once per FOC cycle while driving, since it increments the counters. */
static motor_fault_t check_safety(PMSM_motor *m)
{
    /* No CAN command for too long — drop drive rather than run on a stale command. */
    if (++m->cmd.timeout_counter > (uint32_t)CFG_CAN_TIMEOUT)  return FAULT_CAN_TIMEOUT;
    /* Encoder angle stopped updating (dead/unplugged) — don't commutate blind. */
    if (++m->angle_sensor.stale_counter > ENC_STALE_TIMEOUT)   return FAULT_ENCODER_STALE;
    /* Bus voltage out of range (V_BUS_MIN defaults to 0 → under-volt disabled). */
    if (m->v_bus > V_BUS_MAX)                                  return FAULT_OVERVOLTAGE;
    if (m->v_bus < V_BUS_MIN)                                  return FAULT_UNDERVOLTAGE;
    /* Winding temperature (updated at ~1 Hz by the mgr task; float read is atomic). */
    if (m->motor_temp > CFG_TEMP_CUTOFF)                       return FAULT_OVERTEMP;
    return FAULT_NONE;
}

/* ── run_fsm ─────────────────────────────────────────────────────────────── */

void run_fsm(FSMStruct *fsmstate)
{
    /* 1. Per-cycle pre-processing ----------------------------------------- */
    update_power_voltage(&motor.v_bus);
    angle_sensor_update(&motor.angle_sensor);

    /* 2. Handling pending commands from CAN ISR -------------------------------- */
    if (motor.cmd.pending_fsm_cmd != NO_PENDING_MODE) {
        fsmstate->next_state  = (motor_state)motor.cmd.pending_fsm_cmd;
        motor.cmd.pending_fsm_cmd  = NO_PENDING_MODE;
    }
    if (motor.cmd.new_cmd) {
        motor.cmd.curr_cmd = motor.cmd.cmd_buf;
        motor.cmd.new_cmd = 0;
        motor.cmd.timeout_counter = 0;
    }

    /* 3. FSM transition management ---------------------------------------- */
    if (fsmstate->next_state != fsmstate->curr_state) {
        fsm_exit_state(fsmstate);
        fsmstate->curr_state = fsmstate->next_state;
        fsm_enter_state(fsmstate);
    }

    /* 4. Execute current curr_state -------------------------------------------- */
    switch (fsmstate->curr_state) {

        case MENU_MODE:
            /* Coast — no PWM drive */
            break;

        case MIT_MODE: {
            motor_fault_t fault = check_safety(&motor);
            if (fault != FAULT_NONE) {
                motor.fault = fault;
                fsmstate->next_state = MENU_MODE;
                break;
            }
            mit_control_update(&motor);
            current_control_update(&motor);
            break;
        }

        case CALIBRATION_MODE:
            if (hcal.cal_state != CAL_STATE_IDLE &&
                hcal.cal_state != CAL_STATE_LUT_POSTPROC_PENDING) {
                cal_encoder_misalignment_update(&motor, &hcal);
            }
            /* LUT_POSTPROC_PENDING: postprocessing is compute-intensive; the
             * FreeRTOS task calls foc_cal_lut_postprocess() from the main loop.
             * When it finishes, cal_state returns to IDLE — auto-transition here. */
            if (hcal.cal_state == CAL_STATE_IDLE &&
                fsmstate->next_state == CALIBRATION_MODE) {
                /* Calibration just completed — move to encoder display */
                fsmstate->next_state = ENCODER_MODE;
            }
            break;

        case ENCODER_MODE:
        case SET_ZERO_MODE:
            open_loop_voltage_control(&motor, 0.0f, 0.0f, 0.0f);
            break;

        case R_MEAS_MODE:
            foc_r_meas_update(&motor);
            if (motor.meas_done) {
                motor.meas_done = 0;
                /* Store to RAM config so it can be read back over CAN (no ISR printf),
                 * then request the mgr task to persist it to flash. */
                CFG_RESISTANCE = motor.Rs;
                pending_save = 1;
                fsmstate->next_state = MENU_MODE;
            }
            break;

        case L_MEAS_MODE:
            foc_l_meas_update(&motor);
            if (motor.meas_done) {
                motor.meas_done = 0;
                /* Single inductance slot: store the D/Q average (Ld ~= Lq for this
                 * motor). Read back over CAN; no ISR printf. Persist via mgr task. */
                CFG_INDUCTANCE = 0.5f * (motor.Ld + motor.Lq);
                pending_save = 1;
                fsmstate->next_state = MENU_MODE;
            }
            break;

        default:
            break;
    }
}

/* ── fsm_enter_state ─────────────────────────────────────────────────────── */

void fsm_enter_state(FSMStruct *fsmstate)
{
    switch (fsmstate->curr_state) {

        case MENU_MODE:
            break;

        case MIT_MODE:
            enable_motor(&motor);
            motor.cmd.timeout_counter = 0;
            motor.angle_sensor.stale_counter = 0;  /* don't inherit a stale count from before entry */
            motor.fault = FAULT_NONE;
            break;

        case CALIBRATION_MODE:
            enable_motor(&motor);
            foc_cal_encoder_misalignment_start(&motor, &hcal);
            break;

        case ENCODER_MODE:
        case SET_ZERO_MODE:
            break;

        case R_MEAS_MODE:
            enable_motor(&motor);
            motor.meas_inj_amp = 3.0f;
            motor.meas_inj_n = 0;
            motor.meas_done  = 0;
            break;

        case L_MEAS_MODE:
            enable_motor(&motor);
            motor.meas_inj_amp = 3.0f;
            motor.meas_inj_n   = 0;
            motor.meas_done    = 0;
            motor.l_meas_Vc    = 0.0f;
            motor.l_meas_Vs    = 0.0f;
            motor.l_meas_Ic    = 0.0f;
            motor.l_meas_Is    = 0.0f;
            motor.l_meas_phase = 1;
            break;

        default:
            break;
    }
}

/* ── fsm_exit_state ──────────────────────────────────────────────────────── */

void fsm_exit_state(FSMStruct *fsmstate)
{
    switch (fsmstate->curr_state) {

        case MIT_MODE:
            disable_motor(&motor);
            break;
        case CALIBRATION_MODE:
            disable_motor(&motor);
            break;
        case R_MEAS_MODE:
            disable_motor(&motor);
            break;
        case L_MEAS_MODE:
            disable_motor(&motor);
            break;
        case MENU_MODE:
            break;
        case ENCODER_MODE:
            break;
        case SET_ZERO_MODE:
            break;
        default:
            break;
    }
}


