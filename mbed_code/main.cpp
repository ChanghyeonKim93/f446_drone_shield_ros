/*
    
    F446_DRONE_SHIELD
      Designed by Changhyeon Kim, 2023.01.
      e-mail: hyun91015 at gmail.com

    See documentation and software in the below github link:
        https://github.com/ChanghyeonKim93


*/

// YOU CAN CHANGE THE PARAMETERS IN 'parameter.h'
#include "parameters.h" 







// ==============================================================================
// ==============================================================================
// ======================== DO NOT MODIFY FROM THIS LINE ========================
// ==============================================================================
// ==============================================================================
#include "mbed.h"
#include "BufferedSerial.h"

#include "union_struct.h"
#include <chrono>

void startSignalLED(void);

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// Serial communication
#include "serial_comm_mbed.h"
uint8_t packet_send[256];
uint8_t packet_recv[256];
SerialCommunicatorMbed serial(BAUD_RATE, SERIAL_TX_PIN, SERIAL_RX_PIN);

// Encoder library
#include "encoder_library/Encoder.h"
TIM_Encoder_InitTypeDef encoder3, encoder4;
TIM_HandleTypeDef       timer3, timer4;

// Sonar distance sensor parameters
#include "ultrasonic_library/ultrasonic.h"
void sonarCallbackFunction(int dist) { return; };
ULTRASONIC ultra_sonic(SONAR_TRG, SONAR_ECHO, SONAR_SAMPLING_PERIOD_MS, SONAR_TIMEOUT_MS, &sonarCallbackFunction);

// Drone Motors PWM signals
#include "drone_motor_pwm.h"
DroneMotorPwm motor_pwm;
void setMotorPWM(uint8_t n, uint16_t pwm_ushort) {
    if(pwm_ushort > 4095) pwm_ushort = 4095;
    if(pwm_ushort < 0)    pwm_ushort = 0;
    motor_pwm.setPWM(n, pwm_ushort);
};
void setMotorPWM_01234567(uint16_t pwm_ushort[8]) {
    // for(int i = 0; i < 8; ++i){
    //     if(pwm_ushort[i] < 0 )   pwm_ushort[i] = 0;
    //     if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    // }
    motor_pwm.setPWM_all(pwm_ushort);
};

// Analog-Digital Converter (Battery Voltage and current)
USHORT_UNION adc1_voltage_ushort;
USHORT_UNION adc2_voltage_ushort;
AnalogIn adc1(ADC1_PIN);
AnalogIn adc2(ADC2_PIN);

// Initialise the digital pin LED1 as an output
DigitalOut led(LED1);
int main()
{ 
    // printf("Start F446 DRONE SHIELD\r\n");

    startSignalLED();
    
    // Timer starts.
    Timer timer;
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;
    
    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));
    
    //counting on both A&B inputs, 4 ticks per cycle, full 16-bit count
    EncoderInit(&encoder3, &timer3, TIM3, 0xffff, TIM_ENCODERMODE_TI12);
    EncoderInit(&encoder4, &timer4, TIM4, 0xffff, TIM_ENCODERMODE_TI12);

    // Ultrasonic initialize
    ultra_sonic.startUpdates();

    // Values
    uint16_t bat_voltage = 0;
    uint16_t bat_current = 0;

    int sonar_distance_in_mm = 0;

    uint16_t count3 = 0, count4 = 0;
    int8_t dir3, dir4;

    uint16_t pwm_values[8] = {0,500,1000,1500,2000,2500,3000,3500};

    while (true) {
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> delta_time = time_curr - time_send_prev;

        if(delta_time.count() > LOOP_PERIOD_US){ // 2.5 ms interval

            // Get ultrasonic distance
            ultra_sonic.checkDistance(); //call checkDistance() as much as possible, as this is where the class checks if dist needs to be called.
            sonar_distance_in_mm = ultra_sonic.getCurrentDistance(); // timeout 

            // Get encoder values
            // count3 = __HAL_TIM_GET_COUNTER(&timer3);
            // dir3   = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer3);
            // count4 = __HAL_TIM_GET_COUNTER(&timer4);
            // dir4   = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer4);
            // printf("E1: %d%s, E2: %d%s\r\n", count3, dir3 == 0 ? "+":"-", count4, dir4 == 0 ? "+":"-" );
            
            // Get ADC values
            bat_voltage = adc1.read_u16();
            bat_current = adc2.read_u16();

            printf("ADC1: %5d, ADC2: %5d, Dist.: %5d [mm]\r\n", bat_voltage, bat_current, sonar_distance_in_mm);

            // Set PWM values
            pwm_values[0] += 20;
            pwm_values[1] += 20;
            pwm_values[2] += 20;
            pwm_values[3] += 20;
            pwm_values[4] += 20;
            pwm_values[5] += 20;
            pwm_values[6] += 20;
            pwm_values[7] += 20;
            if(pwm_values[0] > 4000) pwm_values[0] = 0;
            if(pwm_values[1] > 4000) pwm_values[1] = 0;
            if(pwm_values[2] > 4000) pwm_values[2] = 0;
            if(pwm_values[3] > 4000) pwm_values[3] = 0;
            if(pwm_values[4] > 4000) pwm_values[4] = 0;
            if(pwm_values[5] > 4000) pwm_values[5] = 0;
            if(pwm_values[6] > 4000) pwm_values[6] = 0;
            if(pwm_values[7] > 4000) pwm_values[7] = 0;
            setMotorPWM_01234567(pwm_values);
        }
    }
}

void startSignalLED(void){
    led = 1;    wait_us(500000);
    led = 0;    wait_us(500000);

    led = 1;    wait_us(100000);
    led = 0;    wait_us(100000);

    led = 1;    wait_us(100000);
    led = 0;    wait_us(100000);

    led = 1;    wait_us(100000);
    led = 0;    wait_us(100000);
};
