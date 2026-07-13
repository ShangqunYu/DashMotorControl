/*
 * hw_config.h
 *
 *  Created on: Jan 12, 2026
 *      Author: Simon
 */

#ifndef INC_HW_CONFIG_H_
#define INC_HW_CONFIG_H_

/* Timer and PWM */
#define TIM_PWM			    htim1				// PWM/ISR timer handle
#define PWM_A		        TIM_CHANNEL_3		// Terminal W timer channel
#define PWM_B		        TIM_CHANNEL_2		// Terminal V timer channel
#define PWM_C		        TIM_CHANNEL_1		// Terminal U timer channel

/* DRV Gate drive */
#define DRV_ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_CS			    GPIOA, GPIO_PIN_4	// DRV CS pin
#define DRV_SPI			    hspi1				// DRV SPI handle

/* Magnetic encoder */
#define ENC_SPI			    hspi3				// Encoder SPI handle
#define ENC_CS_INT_PORT     GPIOA
#define ENC_CS_INT_PIN      GPIO_PIN_15         // INTERNAL Encoder SPI CS pin
#define ENC_CS_EXT_PORT     GPIOD
#define ENC_CS_EXT_PIN      GPIO_PIN_2          // EXTERNAL Encoder SPI CS pin
/* Convenience two-arg forms (for HAL_GPIO_WritePin calls with a fixed target) */
#define ENC_CS_INT          ENC_CS_INT_PORT, ENC_CS_INT_PIN
#define ENC_CS_EXT          ENC_CS_EXT_PORT, ENC_CS_EXT_PIN
#define ENC_CPR			65536				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x0000				// Encoder read command

/* Misc. GPIO */
#define RED_LED         	GPIOC, GPIO_PIN_5	// RED_LED Pin

/* CAN */
#define CAN_H			hcan1				// CAN handle
#define FILTER_MASK 0b00000000000000000000011111111   // CAN ID filter mask (only look at last 8 bits, i.e. 0-255)

#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle

#define BLDC_PWM_FREQ           40000
#define FOC_TS                  (1.0f / (float)BLDC_PWM_FREQ)

/* Encoder staleness watchdog: max consecutive FOC cycles with no completed
 * encoder DMA read before faulting to MENU_MODE. 40 cycles @ 40 kHz = 1 ms.
 * Normal reads complete every 1-2 cycles, so this only trips on a dead encoder. */
#define ENC_STALE_TIMEOUT       40U

#define V_BUS_MIN           0.0f			// min drive voltage (faults below this)
#define V_BUS_MAX			60.0f			// max drive voltage (faults above this)


/* Current controller */
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation


// ANALOG SENSOR 
#define I_SCALE 			0.0201416f * 2  // Amps per A/D Count at 40X amplifier gain
#define V_SCALE 			0.0169189f    // Bus volts per A/D Count, the V_SCALE is calculated based on 0.0128906 = 3.3 × 16 ÷ 4096
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter
#define TEMP_FILT_ALPHA     .8f     // 1st order Winding temp filter
#define BASE_RESISTOR_RESISTANCE       10000.0f      //Resistor that is in series with the thermistor for temperature measurement
#define THERMISTOR_NOMINAL_RESISTANCE  550.0f      // Nominal resistance of the thermistor at 25 degrees C
#define THERMISTOR_NOMINAL_TEMP           25.0f 
#define THERMISTOR_160C     1400.0f      // Resistance of the thermistor at 160 degrees C (for beta calculation)
#define OHM_PER_DEGREE_C    ((THERMISTOR_160C - THERMISTOR_NOMINAL_RESISTANCE) / (160.0f - 25.0f)) // Resistance change per degree C, calculated from the two known points (25C and 160C)



/* ADC TRIGGERING OFFSET
*  The total ADC (sampling + conversion) takes about 2.5 - 5 us. So we want to shift our ADC trigger a bit earlier 
*  to have more duty cycle available for the motor control. Therefore, we shift the ADC trigger earlier by 1.25 us.
*  (for center-aligned PWM at 40 kHz, timer counts up to 2250 and then count down to 0, so 2250 * 2 / 25 * 1.25 = 225)
*/
#define ADC_TRIG_OFFSET 225  

// CALIBRATION RELATED PARAMETERS
#define W_CAL               10.0f   // LUT sweep speed (electrical rad/s)
#define LUT_SAMPLES_PER_PPAIR 128U  // raw samples per pole-pair during sweep
#define ERROR_LUT_SIZE      128U    // encoder nonlinearity correction LUT entries
#define PPAIRS_MAX          64U     // max supported pole pairs (sizes calibration buffer)
#define N_DETECT_ELECTRIC_CYCLE 20  // how many electric cycle to rotate before calculating the number of pole pair


#endif /* INC_HW_CONFIG_H_ */
