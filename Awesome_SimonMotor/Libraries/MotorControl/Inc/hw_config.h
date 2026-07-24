/*
 * hw_config.h
 *
 *  Created on: Jan 12, 2026
 *      Author: simon
 */

#ifndef INC_HW_CONFIG_H_
#define INC_HW_CONFIG_H_

/* Timer and PWM */
#define TIM_PWM			htim1				// PWM/ISR timer handle
#define TIM_CH_W		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_U		TIM_CHANNEL_3		// Terminal W timer channel

/* ISRs */
#define PWM_ISR			TIM1_UP_TIM10_IRQn	// PWM Timer ISR
#define CAN_ISR			CAN1_RX0_IRQn		// CAN Receive ISR

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

/* SPI encoder */
#define ENC_SPI			hspi3				// Encoder SPI handle

#define USE_EXTERNAL_ENCODER 1		// Set to 1 to use external SPI encoder, 0 to use internal SPI encoder (on same SPI bus as DRV)
#define ENC_CS_INT			GPIOA, GPIO_PIN_15	// Encoder SPI CS pin
#define ENC_CS_EXT			GPIOD, GPIO_PIN_2	// External Encoder SPI CS pin

#if USE_EXTERNAL_ENCODER
#define ENC_CS				ENC_CS_EXT
#else
#define ENC_CS				ENC_CS_INT
#endif


#define ENC_CPR			65536				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x0000				// Encoder read command

/* Misc. GPIO */
#define LED         	GPIOC, GPIO_PIN_5	// LED Pin

/* CAN */
#define CAN_H			hcan1				// CAN handle

/* Other hardware-related constants */
#define I_SCALE 			0.0201416f * 2  // Amps per A/D Count at 40X amplifier gain
#define V_SCALE 			0.0169189f    // Bus volts per A/D Count, the V_SCALE is calculated based on 0.0128906 = 3.3 × 16 ÷ 4096
#define BASE_RESISTOR       10000.0f      //Resistor that is in series with the thermistor for temperature measurement
#define THERMISTOR_NOMINAL  550.0f      // Nominal resistance of the thermistor at 25 degrees C
#define THERMISTOR_160C     1400.0f      // Resistance of the thermistor at 160 degrees C (for beta calculation)
#define OHM_PER_DEGREE_C    ((THERMISTOR_160C - THERMISTOR_NOMINAL) / (160.0f - 25.0f)) // Resistance change per degree C, calculated from the two known points (25C and 160C)


#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define BLDC_PWM_FREQ           40000
#define FOC_TS                  (1.0f / (float)BLDC_PWM_FREQ)
#define SPEED_CONTROL_CYCLE     10
#define SPEED_TS                (FOC_TS * SPEED_CONTROL_CYCLE)
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization
#define V_BUS_MIN           0.0f			// min drive voltage (faults below this)
#define V_BUS_MAX			60.0f			// max drive voltage (faults above this)
#define TEMP_MIN           -40.0f          // min temperature (C) (faults below this)
#define TEMP_MAX           125.0f          // max temperature (C) (faults above this)

/* Current controller */
#define L_D .00004f				// D axis inductance
#define L_Q .00004f				// Q axis inductance
#define K_D .1f                    // Loop gain,  Volts/Amp
#define K_Q .1f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.00012f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.045f                // PI zero, in radians per sample
#define KI_Q 0.045f                // PI zero, in radians per sample
#define OVERMODULATION 1.0f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

#define FILTER_MASK 0b00000000000000000000011111111   // CAN ID filter mask (only look at last 8 bits, i.e. 0-255)

// timer counts after PWM edge to trigger ADC sampling 
// (for center-aligned PWM at 40 kHz, timer counts up to 2250, 
// so 2250 / 25 * 2.5 = 225) we shift the ADC trigger 
// earlier by 2.5 us to allow for some margin, since the total ADC reading takes about 5 us.
#define ADC_TRIG_OFFSET 225  
#define KV 330.0f

#define W_CAL               10.0f   // LUT sweep speed (electrical rad/s)
#define MEASUREMENTS_PER_POLE_PAIR 128U  // raw samples per pole-pair during sweep
#define ERROR_LUT_SIZE      128U    // encoder nonlinearity correction LUT entries
#define PPAIRS_MAX          64U     // max supported pole pairs (sizes calibration buffer)

#define CURRENT_NOISE_CUTOFF_HZ 5000.0f

#define CURRENT_SENSE_TIME_CONST 1.0f / (2.0f * 3.14159265f * CURRENT_NOISE_CUTOFF_HZ) // time constant for first-order low-pass filter on current measurements


#define CURRENT_FILTER_ALPHA (CURRENT_SENSE_TIME_CONST / (CURRENT_SENSE_TIME_CONST + FOC_TS))

//0.466512f

#define CAL_ITERATION 100

#define MIN_TORQUE_AT_OUTPUT_TO_OVERCOME_STATIC_FRICTION 2.75f

#endif /* INC_HW_CONFIG_H_ */
