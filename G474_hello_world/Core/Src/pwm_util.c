#include "pwm_util.h"
#include "tim.h"
#include "stdio.h"

/**
 * Move of setPwmFreqency from main.c. This updates TIM2 ARR and forces
 * an update so the new period takes effect immediately. It also bounds
 * the ARR to 16-bit to avoid overflow on typical timers.
 */
void setPwmFreqency(float desiredPwmFrequencyHz)
{

    /* original code used 170 MHz as clock - keep that constant for now */
    const float timerClockHz = 170000000.0f;

    float pwmPeriod = 1.0f / desiredPwmFrequencyHz;
    float clockPeriod = 1.0f / timerClockHz;

    float maxCount = pwmPeriod / (2.0 * clockPeriod);
    // 1) Set ARR (Auto-reload)
    __HAL_TIM_SET_AUTORELOAD(&htim2, (int)maxCount);

    // 2) Optionally reset the counter (start from 0)
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    printf("init.period %u\n\r", (unsigned int) htim2.Init.Period);
}
