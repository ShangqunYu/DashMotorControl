/*
 * foc.c
 *
 *  Created on: July 01, 2026
 *      Author: Simon
 */

#include "PMSM_motor.h"
#include "angle_sensor.h"
#include "hw_config.h"
#include "tim.h"
#include "adc.h"
#include "analog_sensor.h"
#include "pid_utils.h"
#include "math_utils.h"
#include "drv8353.h"
#include <math.h>
#include <stdio.h>
#include "user_config.h"
#include "foc.h"


void enable_motor(PMSM_motor *motor) {
    zero_commands(motor);
    /* Discard integrator state left over from the previous drive session —
     * a wound-up integral would replay as a voltage kick on the first cycles. */
    pid_reset(&motor->id_ctrl);
    pid_reset(&motor->iq_ctrl);
    drv_enable_gd(motor->gateDriver);
    HAL_GPIO_WritePin(RED_LED, GPIO_PIN_SET);
}

void disable_motor(PMSM_motor *motor) {
    zero_commands(motor);
    drv_disable_gd(motor->gateDriver);
    HAL_GPIO_WritePin(RED_LED, GPIO_PIN_RESET);
}

void zero_commands(PMSM_motor *motor) {
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.0f;
    motor->cmd.curr_cmd.kd    = 0;
    motor->cmd.curr_cmd.kp    = 0;
    motor->cmd.curr_cmd.t_ff  = 0;
    motor->cmd.curr_cmd.p_des = 0;
    motor->cmd.curr_cmd.v_des = 0;
}


void timer_init(PMSM_motor *motor, TIM_HandleTypeDef *htim) {
	motor->pwm_resolution = __HAL_TIM_GET_AUTORELOAD(htim);
}


void set_pwm_dtc(PMSM_motor *motor, float dtc_u, float dtc_v, float dtc_w) {
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_A, (uint32_t)(dtc_u * motor->pwm_resolution));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_B, (uint32_t)(dtc_v * motor->pwm_resolution));
    __HAL_TIM_SET_COMPARE(&TIM_PWM, PWM_C, (uint32_t)(dtc_w * motor->pwm_resolution));
}


void set_limit_current(PMSM_motor *motor, float i_limit) {
	motor->max_current = i_limit;
}


void mit_control_update(PMSM_motor *motor){
    motor->id_ref = 0.0f;
    float pos_error = motor->cmd.curr_cmd.p_des - motor->angle_sensor.mech_angle_rad;
    float vel_error = motor->cmd.curr_cmd.v_des - motor->angle_sensor.mech_angle_vel;
    float torque_des = motor->cmd.curr_cmd.kp * pos_error + motor->cmd.curr_cmd.kd * vel_error + motor->cmd.curr_cmd.t_ff;
    motor->iq_ref = torque_des * (1.0f / (CFG_KT*CFG_GR));
    // Cap iq_ref for safety
    motor->iq_ref = constrain(motor->iq_ref, -motor->max_current, motor->max_current);
}


void open_loop_voltage_control(PMSM_motor *motor, float vd_ref, float vq_ref, float angle_rad) {
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    float va, vb, vc;
    float dtc_u, dtc_v, dtc_w;
    abc(sin_theta, cos_theta, vd_ref, vq_ref, &va, &vb, &vc);
    svm(motor->v_bus, DTC_MIN, DTC_MAX, OVERMODULATION, va, vb, vc, &dtc_u, &dtc_v, &dtc_w);
    set_pwm_dtc(motor, 1.0f-dtc_u, 1.0f-dtc_v, 1.0f-dtc_w); // invert duty cycle because if you want current to flow, you need to drive low (sink current) not high (source current)   
}

void motor_init(PMSM_motor *motor) {

    /* Gate Driver DRV8353 setup */
    drv_init(motor->gateDriver, CFG_I_MAX);

    /* FOC sensor init */
    motor->angle_sensor        = angle_sensor_init();
    motor->cmd.pending_fsm_cmd = NO_PENDING_MODE;

    /* Timer related */
    HAL_TIM_PWM_Start(&TIM_PWM, PWM_A);
    HAL_TIM_PWM_Start(&TIM_PWM, PWM_B);
    HAL_TIM_PWM_Start(&TIM_PWM, PWM_C);
    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CHANNEL_4);
    // shift ADC trigger to occur slightly before the PWM edge to allow for sampling during the deadtime
    TIM_PWM.Instance->CCR4 = TIM_PWM.Instance->ARR - ADC_TRIG_OFFSET;

    timer_init(motor, &TIM_PWM);
    set_limit_current(motor, CFG_I_MAX);
    init_trig_lut();

    /* PI controller initialization for D/Q axis */
    pid_init(&motor->id_ctrl, FOC_TS, CFG_KP_DQ, CFG_KI_DQ, 0.8f, 0.0001f);
    pid_init(&motor->iq_ctrl, FOC_TS, CFG_KP_DQ, CFG_KI_DQ, 0.8f, 0.0001f);

    motor->fsm.curr_state = MENU_MODE;
    motor->fsm.next_state = MENU_MODE;

    CurrentSensor_init(&motor->current_sensor,
                       &(ADC1->JDR1), &(ADC2->JDR1), &(ADC3->JDR1),
                       I_SCALE, 2048, 2048, 2048);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    HAL_Delay(50);

    enable_motor(motor);
    set_pwm_dtc(motor, 0.0f, 0.0f, 0.0f);
    CurrentSensor_calibrate(&motor->current_sensor, 1000U);
    printf("ADC offsets: A=%d, B=%d, C=%d\r\n",
           motor->current_sensor.adc_a_offset,
           motor->current_sensor.adc_b_offset,
           motor->current_sensor.adc_c_offset);
    disable_motor(motor);
}

void current_control_update(PMSM_motor *motor) {
	float id_ref = motor->id_ref;
	float iq_ref = motor->iq_ref;
    const float v_bus = motor->v_bus;

    float sin_theta, cos_theta;

    float vd_ref = 0.0f, vq_ref = 0.0f;

    float dtc_u, dtc_v, dtc_w;

    // get currents
    CurrentSensor_update(&motor->current_sensor);

    // Hard limit references
    id_ref = constrain(id_ref, -motor->max_current, motor->max_current);
    iq_ref = constrain(iq_ref, -motor->max_current, motor->max_current);

    // pre calculate sin & cos
    pre_calc_sin_cos(motor->angle_sensor.e_rad, &sin_theta, &cos_theta);
    // pre_calc_sin_cos(0.0f, &sin_theta, &cos_theta);

    float id, iq;
    clarke_park_transform(motor->current_sensor.ia, motor->current_sensor.ib, motor->current_sensor.ic,
                          sin_theta, cos_theta, &id, &iq);
    

    // set dynamic max output vd and vq
    motor->id_ctrl.out_max = motor->id_ctrl.out_max_dynamic * v_bus;
    motor->iq_ctrl.out_max = motor->iq_ctrl.out_max_dynamic * v_bus;

    vd_ref = pi_control(&motor->id_ctrl, id_ref - id);
    vq_ref = pi_control(&motor->iq_ctrl, iq_ref - iq);

    float va, vb, vc;
    abc(sin_theta, cos_theta, vd_ref, vq_ref, &va, &vb, &vc);
    svm(v_bus, DTC_MIN, DTC_MAX, OVERMODULATION, va, vb, vc, &dtc_u, &dtc_v, &dtc_w);
    set_pwm_dtc(motor, 1.0f-dtc_u, 1.0f-dtc_v, 1.0f-dtc_w); // invert duty cycle because if you want current to flow, you need to drive low (sink current) not high (source current)
    

    // copy to struct for debug
    motor->id = id;
    motor->iq = iq;

}
