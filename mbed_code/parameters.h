#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

/*
    !!! STRONG RECOMMENDATION !!!
      - Do not change the Pin Numbers.
      - You should modify 'numeric values' only.
*/

// Loop period in milliseconds
#define LOOP_PERIOD_MS 2.5 // in miiliseconds, 2: 500 Hz, 2.5: 400 Hz, 4: 250 Hz, 5: 200 Hz, 10: 100 Hz
#define LOOP_PERIOD_US LOOP_PERIOD_MS*1000

// Serial communication parameters
#define BAUD_RATE      921600 // Maximum 921600. Please set it to at most 460800.
#define SERIAL_TX_PIN  USBTX
#define SERIAL_RX_PIN  USBRX

// Sonar distance sensor parameters
#define SONAR_SAMPLING_PERIOD_MS 10 // 340 m : 3 m = 1000 ms : 8.8 ms (speed of sound)
#define SONAR_TIMEOUT_MS         9
#define SONAR_TRG      PA_9
#define SONAR_ECHO     PA_10

// IMU parameters
#define ICM42605_SPI_MOSI PC_1
#define ICM42605_SPI_MISO PC_2 
#define ICM42605_SPI_CLK  PB_13
#define ICM42605_SPI_CS   PB_12
#define ICM42605_SPI_INT  PB_14

#define ICM42688_SPI_MOSI PC_1
#define ICM42688_SPI_MISO PC_2 
#define ICM42688_SPI_CLK  PB_13
#define ICM42688_SPI_CS   PB_12
#define ICM42688_SPI_INT  PB_14

#define WAIT_US_FOR_SPI   10 // us, DO NOT CHANGE

// PWM parameters
#define NUM_MOTORS     8
#define PWM_FREQUENCY  500   // [Hz] // For DC motor of the UGV, pwm frequency over 15 kHz is needed.
#define MOTOR_0_PWM    PA_0  // pwm2/1, // PWM (TIMER 2)
#define MOTOR_1_PWM    PA_1  // pwm2/2
#define MOTOR_2_PWM    PB_10 // pwm2/3
#define MOTOR_3_PWM    PB_2  // pwm2/4
#define MOTOR_4_PWM    PC_6  // pwm8/1, // PWM (TIMER 8)
#define MOTOR_5_PWM    PC_7  // pwm8/2
#define MOTOR_6_PWM    PC_8  // pwm8/3
#define MOTOR_7_PWM    PC_9  // pwm8/4

// AnalogIn parameters
#define ADC1_PIN       PB_0
#define ADC2_PIN       PB_1

// Sensor Trigger Pin parameters
#define TRIGGER_PIN    PA_8

// Encoder parameters
#define MOTOR_ENCODER_PERIOD_MS  10   // in miiliseconds
#define PULSE_PER_MOTOR_TURN     256 // depends on encoder spec.
#define GEAR_RATIO               27  // depends on the gear spec.
#define PULSE_PER_ROTATION       ((PULSE_PER_MOTOR_TURN)*(GEAR_RATIO))
#define RADIAN_PER_ROTATION      (6.28318530718f)
// RADIAN_PER_ROTATION / PULSE_PER_ROTATION == RADIAN_PER_PULSE
// PULSE_PER_MOTOR_TURN*GEAR_RATIO == PULSE_PER_ROTATION
// delta_counter / PULSE_PER_ROTATION == rotated ratio
// delta_pulses * RADIAN_PER_PULSE == delta_radian
#endif