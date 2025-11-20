/* pwm_util.h - small helper to configure TIM2 PWM frequency at runtime */
#ifndef PWM_UTIL_H
#define PWM_UTIL_H

#include "stdio.h"
#include "usart.h"

/**
 * Set the PWM frequency for TIM2 (runtime change).
 * Preserves safety checks and applies the ARR change to hardware.
 *
 * Note: This function expects TIM2 to be configured and running.
 */
void setPwmFreqency(float desiredPwmFrequencyHz);
void playNote();

#endif // PWM_UTIL_H
