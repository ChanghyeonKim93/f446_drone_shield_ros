/*
    F446_DRONE_SHIELD
        Designed by Changhyeon Kim, 2023.01.
        e-mail: hyun91015 at gmail.com

    See documentation and software in the below github link:
        https://github.com/ChanghyeonKim93/f446_drone_shield_ros
*/

// YOU CAN CHANGE THE PARAMETERS IN 'parameter.h'
#include "parameters.h" 
 


// ==============================================================================
// ==============================================================================
// ======================== DO NOT MODIFY FROM THIS LINE ========================
// ==============================================================================
// ==============================================================================
#include "mbed.h"
#include "union_struct.h"
#include <chrono>

// Set event queue and worker thread
Thread thread_poll(osPriorityRealtime); // polling all interrupt signals. https://os.mbed.com/docs/mbed-os/v6.10/apis/thread.html
EventQueue event_queue(128 * EVENTS_EVENT_SIZE);

// LED signal functions
DigitalOut led_signal(LED1); // On-board green LED. (PA_5)
void startSignalLED(DigitalOut& led);
void ledSignals_OK(DigitalOut& led, int n_blink);

// Sonar distance sensor parameters
#include "ultrasonic_library/ultrasonic.h"
USHORT_UNION sonar_dist_mm;
void sonarCallbackFunction(int dist) { return; };
ULTRASONIC ultra_sonic(SONAR_TRG, SONAR_ECHO, SONAR_SAMPLING_PERIOD_MS, SONAR_TIMEOUT_MS, &sonarCallbackFunction);

// Drone Motors PWM signals
#include "drone_motor_pwm.h"
uint16_t pwm_values[8] = {0,0,0,0,0,0,0,0};
DroneMotorPwm motor_pwm;
void setMotorPWM_01234567(uint16_t pwm_ushort[8]) {
    for(int i = 0; i < 8; ++i){
        if(pwm_ushort[i] < 0 )   pwm_ushort[i] = 0;
        if(pwm_ushort[i] > 4095) pwm_ushort[i] = 4095;
    }
    motor_pwm.setPWM_all(pwm_ushort);
};

// Analog-Digital Converter (Battery Voltage and current)
USHORT_UNION adc1_voltage_ushort;
USHORT_UNION adc2_voltage_ushort;
AnalogIn adc1(ADC1_PIN);
AnalogIn adc2(ADC2_PIN);

// Camera Trigger signal
#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010
volatile uint8_t trigger_step  = 25;
volatile uint8_t trigger_count = 0;
volatile uint8_t flag_camera_trigger = CAMERA_TRIGGER_LOW;
DigitalOut signal_trigger(TRIGGER_PIN);

// Encoder library
// #include "encoder_library/Encoder.h"
#include "encoder_library/motor_encoder.h"
// TIM_Encoder_InitTypeDef encoder3, encoder4;
// TIM_HandleTypeDef       timer3, timer4;
FLOAT_UNION radian_per_sec_A;
FLOAT_UNION radian_per_sec_B;
MotorEncoder motor_encoder(MOTOR_ENCODER_PERIOD_MS);


// Serial communication
#include "serial_comm_mbed.h"
Timeout timeout_serial_read;
void flipLED() { led_signal = !led_signal; };

uint8_t  packet_send[256];
uint8_t  packet_recv[256];
uint32_t len_packet_recv = 0;
uint32_t len_packet_send = 0;
enum MessageTypeByLength {
    EMPTY = 0,
    PWMCOMMAND = 16
};
SerialCommunicatorMbed serial_usb(BAUD_RATE, SERIAL_TX_PIN, SERIAL_RX_PIN);

// IMU
#include "icm42605_library/icm42605_spi.h"
ICM42605_SPI imu;

void workfunction_readSerialUSB() {
    if(serial_usb.tryToReadSerialBuffer()) { // packet ready!
        led_signal = 1;
        timeout_serial_read.attach(callback(&flipLED), 10ms);

        int len_recv_message = 0;
        len_recv_message = serial_usb.getReceivedMessage(packet_recv); 

        if( len_recv_message == MessageTypeByLength::PWMCOMMAND ) 
        { 

            // Successfully received the packet.
            // In case of 8 PWM signals.
            // ======== USER-DEFINED CODE START ======== 
            USHORT_UNION pwm_tmp;
            pwm_tmp.bytes_[0] = packet_recv[0];  pwm_tmp.bytes_[1] = packet_recv[1];   pwm_values[0] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[2];  pwm_tmp.bytes_[1] = packet_recv[3];   pwm_values[1] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[4];  pwm_tmp.bytes_[1] = packet_recv[5];   pwm_values[2] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[6];  pwm_tmp.bytes_[1] = packet_recv[7];   pwm_values[3] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[8];  pwm_tmp.bytes_[1] = packet_recv[9];   pwm_values[4] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[10]; pwm_tmp.bytes_[1] = packet_recv[11];  pwm_values[5] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[12]; pwm_tmp.bytes_[1] = packet_recv[13];  pwm_values[6] = pwm_tmp.ushort_;
            pwm_tmp.bytes_[0] = packet_recv[14]; pwm_tmp.bytes_[1] = packet_recv[15];  pwm_values[7] = pwm_tmp.ushort_;

            setMotorPWM_01234567(pwm_values);     
            // ======== USER-DEFINED CODE END ========      
        }
        else if( len_recv_message == MessageTypeByLength::EMPTY)
        {
            // empty message is recieved.
            // ======== USER-DEFINED CODE START ======== 

            // ======== USER-DEFINED CODE END ========      
        }
    }
};
void workfunction_sendSerialUSB() {
    if(serial_usb.writable()){
        serial_usb.send_withChecksum(packet_send, len_packet_send);
        len_packet_send = 0;
    }
};
void tryToReadSerialUSB(){ event_queue.call(workfunction_readSerialUSB); };
void tryToSendSerialUSB(){ event_queue.call(workfunction_sendSerialUSB); };

// Timer 
Timer timer;
uint64_t us_curr;
USHORT_UNION tsec;
UINT_UNION   tusec;

// MAIN FUNCTION
int main()
{ 
    // printf("Start F446 DRONE SHIELD\r\n");
    startSignalLED(led_signal);
    
    // Start the event queue
    thread_poll.start(callback(&event_queue, &EventQueue::dispatch_forever));
    
    // Start Timer
    timer.start();
    std::chrono::microseconds time_send_prev = timer.elapsed_time();
    std::chrono::microseconds time_recv_prev = timer.elapsed_time();
    std::chrono::microseconds time_curr;
    ledSignals_OK(led_signal, 1);

    // Ultrasonic initialize
    ultra_sonic.startUpdates();
    ledSignals_OK(led_signal, 2);

    // IMU initialize
    imu.reset(); // software reset ICM42605 to default registers

    /* Specify sensor parameters (sample rate is twice the bandwidth)
    * choices are:
        AFS_2G, AFS_4G, AFS_8G, AFS_16G  
        GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
        AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
        GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
    */ 
    uint8_t Ascale = AFS_8G, Gscale = GFS_2000DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;

    FLOAT_UNION ax, ay, az;
    FLOAT_UNION gx, gy, gz;

    imu.reset();
    float aRes = imu.getAres(Ascale);
    float gRes = imu.getGres(Gscale);
    imu.init(Ascale, Gscale, AODR, GODR);
    ledSignals_OK(led_signal, 3);

    // Encoder initialize
    ledSignals_OK(led_signal, 4);

    while (true) {
        time_curr = timer.elapsed_time();
        std::chrono::duration<int, std::micro> delta_time = time_curr - time_send_prev;

        if(delta_time.count() > LOOP_PERIOD_US) { // 2.5 ms interval
            // Get ultrasonic distance
            ultra_sonic.checkDistance(); //call checkDistance() as much as possible, as this is where the class checks if dist needs to be called.
            sonar_dist_mm.ushort_ = ultra_sonic.getCurrentDistance(); // timeout 

            // Get encoder values
            int32_t dcnt_A = motor_encoder.getDeltaCounter_A();
            int32_t dcnt_B = motor_encoder.getDeltaCounter_B();
            radian_per_sec_A.float_ = motor_encoder.getAngularVelocity_A();
            radian_per_sec_B.float_ = motor_encoder.getAngularVelocity_B();
            
            // Get ADC values
            adc1_voltage_ushort.ushort_ = adc1.read_u16();
            adc2_voltage_ushort.ushort_ = adc2.read_u16();

            // printf("A1: %5d A2: %5d D: %5d[mm]\r\n", adc1_voltage_ushort.ushort_, adc2_voltage_ushort.ushort_, sonar_distance_in_mm);
            int16_t ICM42605Data[7];        // Stores the 16-bit signed sensor output
            imu.readData(ICM42605Data); 

            // Now we'll calculate the accleration value into actual g's
            float ax = (float)ICM42605Data[1]*aRes;  // get actual g value, this depends on scale being set
            float ay = (float)ICM42605Data[2]*aRes;   
            float az = (float)ICM42605Data[3]*aRes;  

            // Calculate the gyro value into actual degrees per second
            float gx = (float)ICM42605Data[4]*gRes;  // get actual gyro value, this depends on scale being set
            float gy = (float)ICM42605Data[5]*gRes;  
            float gz = (float)ICM42605Data[6]*gRes; 

            printf("acc: %d %d %d, gyro: %d %d %d\r\n",
                ICM42605Data[1], ICM42605Data[2], ICM42605Data[3],
                ICM42605Data[4], ICM42605Data[5], ICM42605Data[6]);

            if(serial_usb.writable()) { // If serial USB can be written,
                // Time 
                us_curr      = timer.elapsed_time().count(); // Current time
                tsec.ushort_ = (uint16_t)(us_curr/1000000);
                tusec.uint_  = (uint32_t)(us_curr-((uint32_t)tsec.ushort_)*1000000);

                // Camera trigger signal output.
                ++trigger_count;
                if(trigger_count >= trigger_step) {
                    trigger_count = 0;
                    flag_camera_trigger = CAMERA_TRIGGER_HIGH;
                    signal_trigger = 1;
                    wait_us(100);
                    signal_trigger = 0;
                }

                /*
                    <Packet structure>
                        D[0] ~D[17] 3-axis acc. 3-axis gyro, 3-axis mag.
                        D[18],D[19] time (second part)
                        D[20]~D[23] time (microsecond part)
                        D[24]       Cam trigger state
                        D[25],D[26] ADC1
                        D[27],D[28] ADC1
                        D[29],D[30] Sonar distance

                */

                // IMU data (3D acc, 3D gyro, 3D magnetometer)
                // packet_send[0]  = acc_short[0].bytes_[0]; packet_send[1]  = acc_short[0].bytes_[1];
                // packet_send[2]  = acc_short[1].bytes_[0]; packet_send[3]  = acc_short[1].bytes_[1];
                // packet_send[4]  = acc_short[2].bytes_[0]; packet_send[5]  = acc_short[2].bytes_[1];
                
                // packet_send[6]  = gyro_short[0].bytes_[0]; packet_send[7]  = gyro_short[0].bytes_[1];
                // packet_send[8]  = gyro_short[1].bytes_[0]; packet_send[9]  = gyro_short[1].bytes_[1];
                // packet_send[10] = gyro_short[2].bytes_[0]; packet_send[11] = gyro_short[2].bytes_[1];

                // packet_send[12] = mag_short[0].bytes_[0]; packet_send[13] = mag_short[0].bytes_[1];
                // packet_send[14] = mag_short[1].bytes_[0]; packet_send[15] = mag_short[1].bytes_[1];
                // packet_send[16] = mag_short[2].bytes_[0]; packet_send[17] = mag_short[2].bytes_[1];

                packet_send[18]  = tsec.bytes_[0];  // time (second part, low)
                packet_send[19]  = tsec.bytes_[1];  // time (second part, high)
                
                packet_send[20]  = tusec.bytes_[0]; // time (microsecond part, lowest)
                packet_send[21]  = tusec.bytes_[1]; // time (microsecond part, low)
                packet_send[22]  = tusec.bytes_[2]; // time (microsecond part, high)
                packet_send[23]  = tusec.bytes_[3]; // time (microsecond part, highest)

                // Camera trigger state
                packet_send[24]     = flag_camera_trigger; // 'CAMERA_TRIGGER_HIGH' or 'CAMERA_TRIGGER_LOW'
                flag_camera_trigger = CAMERA_TRIGGER_LOW;

                // AnalogIn (battery voltage data)
                packet_send[25]  = adc1_voltage_ushort.bytes_[0];
                packet_send[26]  = adc1_voltage_ushort.bytes_[1];
                packet_send[27]  = adc2_voltage_ushort.bytes_[0];
                packet_send[28]  = adc2_voltage_ushort.bytes_[1];

                // Sonar distance 
                packet_send[29] = sonar_dist_mm.bytes_[0];
                packet_send[30] = sonar_dist_mm.bytes_[1];

                // Encoder signals (4 bytes per encoder, values in float)
                packet_send[31] = radian_per_sec_A.bytes_[0];
                packet_send[32] = radian_per_sec_A.bytes_[1];
                packet_send[33] = radian_per_sec_A.bytes_[2];
                packet_send[34] = radian_per_sec_A.bytes_[3];

                packet_send[35] = radian_per_sec_B.bytes_[0];
                packet_send[36] = radian_per_sec_B.bytes_[1];
                packet_send[37] = radian_per_sec_B.bytes_[2];
                packet_send[38] = radian_per_sec_B.bytes_[3]; // total 

                // Send length
                len_packet_send = 39;

                // tryToSendSerialUSB(); // Send!

                // flag_imu_ready = false;
                time_send_prev = time_curr;

            }
        }

        // Read data if data exists.
        if(serial_usb.readable()) {
            tryToReadSerialUSB();
        }
    }
}

void startSignalLED(DigitalOut& led){
    for(int i = 0; i < 16; ++i) {
        led = 1; ThisThread::sleep_for(20ms);
        led = 0; ThisThread::sleep_for(20ms);
    }
    ThisThread::sleep_for(400ms);

    // Guk-Bbong bit
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(100ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(300ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(100ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(300ms);
    // led = 1;    ThisThread::sleep_for(100ms);
    // led = 0;    ThisThread::sleep_for(50ms);
    // ThisThread::sleep_for(500ms);
};

void ledSignals_OK(DigitalOut& led, int n_blink){
    for(int i = 0; i < n_blink; ++i) {
        led = 1; ThisThread::sleep_for(150ms);
        led = 0; ThisThread::sleep_for(75ms);
    }
    ThisThread::sleep_for(200ms);
};

