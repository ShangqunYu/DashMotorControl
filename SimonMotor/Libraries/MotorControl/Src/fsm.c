/*
 * fsm.c
 *
 * Motor controller finite-state machine.
 * Adapted from Ben Katz's original fsm.c; control calls updated to match
 * this project's FOC API.
 */

#include "fsm.h"
#include "foc.h"
#include "foc_calibration.h"
#include "analog_sensor.h"
#include "angle_sensor.h"
#include "drv8353.h"
#include "math_ops.h"
#include "user_config.h"
#include "hw_config.h"
#include "usart.h"
#include "preference_writer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ── Globals owned by main.c ─────────────────────────────────────────────── */
extern foc_t            hfoc;
extern CalStruct        hcal;
extern PreferenceWriter prefs;
extern DRVStruct        drv;

/* ── run_fsm ─────────────────────────────────────────────────────────────── */

void run_fsm(FSMStruct *fsmstate)
{
    /* 1. Per-cycle pre-processing ----------------------------------------- */
    hfoc.v_bus = get_power_voltage();
    foc_update_velocity(&hfoc, FOC_TS);

    /* 2. Apply pending MIT parameters (not a mode change) ----------------- */
    if (hfoc.new_cmd) {
        hfoc.mit_cmd    = hfoc.mit_buf;
        hfoc.new_cmd = 0;
    }

    /* 3. FSM transition management ---------------------------------------- */
    if (fsmstate->next_state != fsmstate->state) {
        fsm_exit_state(fsmstate);
        if (fsmstate->ready) {
            fsmstate->state       = fsmstate->next_state;
            hfoc.control_mode     = fsmstate->state;
            fsm_enter_state(fsmstate);
        }
    }

    /* 4. Execute current state -------------------------------------------- */
    switch (fsmstate->state) {

        case MENU_MODE:
        case SETUP_MODE:
            /* Coast — no PWM drive */
            break;

        case TORQUE_CONTROL_MODE:
            foc_current_control_update(&hfoc, FOC_TS);
            break;

        case SPEED_CONTROL_MODE:
            foc_speed_control_update(&hfoc, hfoc.vel_ref);
            foc_current_control_update(&hfoc, FOC_TS);
            break;

        case POSITION_CONTROL_MODE:
            foc_speed_control_update(&hfoc, hfoc.vel_ref);
            foc_current_control_update(&hfoc, FOC_TS);
            break;

        case MIT_MODE:
            foc_mit_control_update(&hfoc);
            foc_current_control_update(&hfoc, FOC_TS);
            break;

        case CALIBRATION_MODE:
            if (hcal.cal_state != CAL_STATE_IDLE &&
                hcal.cal_state != CAL_STATE_LUT_POSTPROC_PENDING) {
                foc_cal_encoder_misalignment_update(&hfoc, &hcal, FOC_TS);
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
            open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
            break;

        default:
            break;
    }
}

/* ── fsm_enter_state ─────────────────────────────────────────────────────── */

void fsm_enter_state(FSMStruct *fsmstate)
{
    switch (fsmstate->state) {

        case MENU_MODE:
            enter_menu_state();
            fsmstate->ready = 1;
            break;

        case SETUP_MODE:
            enter_setup_state();
            fsmstate->ready = 1;
            break;

        case TORQUE_CONTROL_MODE:
        case SPEED_CONTROL_MODE:
        case POSITION_CONTROL_MODE:
        case MIT_MODE:
            drv_enable_gd(drv);
            foc_current_control_update(&hfoc, 0.0f);   /* reset PI integrators */
            HAL_GPIO_WritePin(LED, GPIO_PIN_SET);
            fsmstate->ready = 1;
            hfoc.mit_cmd.kd =0;
            hfoc.mit_cmd.kp = 0;
            hfoc.mit_cmd.t_ff = 0;
            hfoc.mit_cmd.p_des = 0;
            hfoc.mit_cmd.v_des = 0;
            break;

        case CALIBRATION_MODE:
            foc_cal_encoder_misalignment_start(&hfoc, &hcal);
            drv_enable_gd(drv);
            fsmstate->ready = 1;
            break;

        case ENCODER_MODE:
        case SET_ZERO_MODE:
            fsmstate->ready = 1;
            break;

        default:
            fsmstate->ready = 1;
            break;
    }
}

/* ── fsm_exit_state ──────────────────────────────────────────────────────── */

void fsm_exit_state(FSMStruct *fsmstate)
{
    switch (fsmstate->state) {

        case TORQUE_CONTROL_MODE:
        case SPEED_CONTROL_MODE:
        case POSITION_CONTROL_MODE:
        case MIT_MODE:
        case CALIBRATION_MODE:
            drv_disable_gd(drv);
            open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
            hfoc.id_ref = 0.0f;
            hfoc.iq_ref = 0.0f;
            HAL_GPIO_WritePin(LED, GPIO_PIN_RESET);
            fsmstate->ready = 1;
            break;

        case MENU_MODE:
        case ENCODER_MODE:
        case SET_ZERO_MODE:
        case SETUP_MODE:
            fsmstate->ready = 1;
            break;

        default:
            fsmstate->ready = 1;
            break;
    }
}

/* ── update_fsm ──────────────────────────────────────────────────────────── */

void update_fsm(FSMStruct *fsmstate, char fsm_input)
{
    /* ESC always returns to menu from any state */
    if (fsm_input == MENU_CMD) {
        fsmstate->next_state = MENU_MODE;
        fsmstate->ready = 0;
        return;
    }

    switch (fsmstate->state) {

        case MENU_MODE:
            switch (fsm_input) {
                case CAL_CMD:
                    fsmstate->next_state = CALIBRATION_MODE;
                    fsmstate->ready = 0;
                    break;
                case MOTOR_CMD:
                    fsmstate->next_state = MIT_MODE;
                    fsmstate->ready = 0;
                    break;
                case ENCODER_CMD:
                    fsmstate->next_state = ENCODER_MODE;
                    fsmstate->ready = 0;
                    break;
                case SETUP_CMD:
                    fsmstate->next_state = SETUP_MODE;
                    fsmstate->ready = 0;
                    break;
                case ZERO_CMD:
                    angle_sensor_set_m_zero(&hfoc.angle_sensor);
                    M_ZERO_RAD = hfoc.angle_sensor.m_zero;
                    if (!preference_writer_ready(prefs)) { preference_writer_open(&prefs); }
                    preference_writer_flush(&prefs);
                    preference_writer_close(&prefs);
                    preference_writer_load(prefs);
                    printf("Saved zero: %.4f rad\r\n", M_ZERO_RAD);
                    break;
            }
            break;

        case SETUP_MODE:
            if (fsm_input == ENTER_CMD) {
                process_user_input(fsmstate);
                break;
            }
            if (fsmstate->bytecount == 0) { fsmstate->cmd_id = fsm_input; }
            else { fsmstate->cmd_buff[fsmstate->bytecount - 1] = fsm_input; }
            fsmstate->bytecount++;
            break;

        case ENCODER_MODE:
        case TORQUE_CONTROL_MODE:
        case SPEED_CONTROL_MODE:
        case MIT_MODE:
        case CALIBRATION_MODE:
        default:
            break;
    }
}

/* ── Serial menu helpers ─────────────────────────────────────────────────── */

void enter_menu_state(void)
{
    printf("\r\n\r\n");
    printf(" Commands:\r\n");
    printf(" m - Motor Mode (torque control)\r\n");
    printf(" c - Calibrate Encoder\r\n");
    printf(" s - Setup\r\n");
    printf(" e - Display Encoder\r\n");
    printf(" z - Set Zero Position\r\n");
    printf(" ESC - Exit to Menu\r\n");
}

void enter_setup_state(void)
{
    printf("\r\n Configuration Options \r\n");
    printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
    printf("\r\n Motor:\r\n");
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "g", "Gear Ratio", "0", "-", GR);
    printf(" %-4s %-31s %-5s %-6s %.5f\r\n", "k", "Torque Constant (N-m/A)", "0", "-", KT);
    printf("\r\n Control:\r\n");
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "l", "Current Limit (A)", "0.0", "75.0", I_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "p", "Max Position Setpoint (rad)", "-", "-", P_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "v", "Max Velocity Setpoint (rad/s)", "-", "-", V_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0", KP_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0", KD_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "c", "Continuous Current (A)", "0.0", "40.0", I_MAX_CONT);
    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "a", "Calibration Current (A)", "0.0", "20.0", I_CAL);
    printf("\r\n CAN:\r\n");
    printf(" %-4s %-31s %-5s %-6s %-5i\r\n", "i", "CAN ID", "0", "127", CAN_ID);
    printf(" %-4s %-31s %-5s %-6s %-5i\r\n", "m", "CAN TX ID", "0", "127", CAN_MASTER);
    printf(" %-4s %-31s %-5s %-6s %d\r\n",   "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    printf(" \r\n To change a value, type 'prefix''value''ENTER'\r\n e.g. 'b1000''ENTER'\r\n ");
    printf("VALUES NOT ACTIVE UNTIL POWER CYCLE!\r\n\r\n");
}

void process_user_input(FSMStruct *fsmstate)
{
    switch (fsmstate->cmd_id) {
        case 'b':
            I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 100.0f);
            printf("I_BW set to %f\r\n", I_BW);
            break;
        case 'i':
            CAN_ID = atoi(fsmstate->cmd_buff);
            printf("CAN_ID set to %d\r\n", CAN_ID);
            break;
        case 'm':
            CAN_MASTER = atoi(fsmstate->cmd_buff);
            printf("CAN_TX_ID set to %d\r\n", CAN_MASTER);
            break;
        case 'l':
            I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 75.0f), 0.0f);
            printf("I_MAX set to %f\r\n", I_MAX);
            break;
        case 'f':
            I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
            printf("I_FW_MAX set to %f\r\n", I_FW_MAX);
            break;
        case 't':
            CAN_TIMEOUT = atoi(fsmstate->cmd_buff);
            printf("CAN_TIMEOUT set to %d\r\n", CAN_TIMEOUT);
            break;
        // case 'h':
        //     TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
        //     printf("TEMP_MAX set to %f\r\n", TEMP_MAX);
        //     break;
        case 'c':
            I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
            printf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
            break;
        case 'a':
            I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
            printf("I_CAL set to %f\r\n", I_CAL);
            break;
        case 'g':
            GR = fmaxf(atof(fsmstate->cmd_buff), 0.001f);
            printf("GR set to %f\r\n", GR);
            break;
        case 'k':
            KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);
            printf("KT set to %f\r\n", KT);
            break;
        case 'x':
            KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
            printf("KP_MAX set to %f\r\n", KP_MAX);
            break;
        case 'd':
            KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
            printf("KD_MAX set to %f\r\n", KD_MAX);
            break;
        case 'p':
            P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
            P_MIN = -P_MAX;
            printf("P_MAX set to %f\r\n", P_MAX);
            break;
        case 'v':
            V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
            V_MIN = -V_MAX;
            printf("V_MAX set to %f\r\n", V_MAX);
            break;
        default:
            printf("\r\n '%c' not a valid command prefix\r\n\r\n", fsmstate->cmd_id);
            break;
    }

    if (!preference_writer_ready(prefs)) { preference_writer_open(&prefs); }
    preference_writer_flush(&prefs);
    preference_writer_close(&prefs);
    preference_writer_load(prefs);

    enter_setup_state();

    fsmstate->bytecount = 0;
    fsmstate->cmd_id    = 0;
    memset(fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
}
