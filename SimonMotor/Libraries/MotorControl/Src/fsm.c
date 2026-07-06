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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ── Globals owned by main.c ─────────────────────────────────────────────── */
extern PMSM_motor       motor;
extern CalStruct        hcal;

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

        case MIT_MODE:
            mit_control_update(&motor);
            current_control_update(&motor);
            break;

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
                printf("\r\nRs = %.4f Ohm\r\n", motor.Rs);
                fsmstate->next_state = MENU_MODE;
            }
            break;

        case L_MEAS_MODE:
            foc_l_meas_update(&motor);
            if (motor.meas_done) {
                motor.meas_done = 0;
                printf("\r\nLd = %.4f mH   Lq = %.4f mH\r\n",
                       motor.Ld * 1000.0f, motor.Lq * 1000.0f);
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
            enable_motor();
            break;

        case CALIBRATION_MODE:
            enable_motor();
            foc_cal_encoder_misalignment_start(&motor, &hcal);
            break;

        case ENCODER_MODE:
        case SET_ZERO_MODE:
            break;

        case R_MEAS_MODE:
            enable_motor();
            motor.meas_inj_amp = 3.0f;
            motor.meas_inj_n = 0;
            motor.meas_done  = 0;
            printf("\r\nStarting R measurement (Vd = %.2f V)...\r\n", motor.meas_inj_amp);
            break;

        case L_MEAS_MODE:
            enable_motor();
            motor.meas_inj_amp = 3.0f;
            motor.meas_inj_n   = 0;
            motor.meas_done    = 0;
            motor.l_meas_Vc    = 0.0f;
            motor.l_meas_Vs    = 0.0f;
            motor.l_meas_Ic    = 0.0f;
            motor.l_meas_Is    = 0.0f;
            motor.l_meas_phase = 1;
            printf("\r\nStarting L measurement (V = %.2f V, f = 1000 Hz)...\r\n",
                   motor.meas_inj_amp);
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
            disable_motor();
        case CALIBRATION_MODE:
            disable_motor();
        case R_MEAS_MODE:
        case L_MEAS_MODE:
            disable_motor();
            break;

        case MENU_MODE:
            break;

        case ENCODER_MODE:
        case SET_ZERO_MODE:
            break;

        default:
            break;
    }
}


void disable_motor(){
    zero_commands(&motor);
    drv_disable_gd(motor.gateDriver);
    HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);
}

void enable_motor(){
    zero_commands(&motor);
    drv_enable_gd(motor.gateDriver);
    HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET);
}
