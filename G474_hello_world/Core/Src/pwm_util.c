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

void playNote(){
    uint8_t buf[1];
        if (HAL_UART_Receive(&huart2, buf, 1, 1)==HAL_OK){
            // print out

        // music
        if ((char)buf[0]=='c') {
        setPwmFreqency(261.625);
        }
        if ((char)buf[0]=='d') {
        setPwmFreqency(293.6648);
        }
        if ((char)buf[0]=='e') {
        setPwmFreqency(329.6276);
        }
        if ((char)buf[0]=='f') {
        setPwmFreqency(349.2282);
        }
        if ((char)buf[0]=='g') {
        setPwmFreqency(391.9954);
        }
        if ((char)buf[0]=='a') {
        setPwmFreqency(440.0);
        }
        if ((char)buf[0]=='b') {
        setPwmFreqency(493.8833);
        }
    }    
}
