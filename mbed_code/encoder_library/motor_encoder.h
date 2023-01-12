#ifndef _MOTOR_ENCODER_H_
#define _MOTOR_ENCODER_H_


#include "mbed.h"
#include "Encoder.h"

#include "parameters.h"


class MotorEncoder
{
public:
    MotorEncoder(int update_speed_ms);

    float getAngularVelocity_A();
    float getAngularVelocity_B();

    int16_t getDeltaCounter_A();
    int16_t getDeltaCounter_B();

public:
    void callbackUpdateAngularVelocity_AB();

private:
    TIM_Encoder_InitTypeDef encoder_A_, encoder_B_;
    TIM_HandleTypeDef       timer_A_, timer_B_;

    Ticker ticker_; // 10 ms 당 한번씩. 

    unsigned long long update_speed_ms_;
    float inv_period_sec_;
    
private:
    uint16_t counter_A_, counter_B_;
    int8_t  dir_A_, dir_B_;

    int32_t delta_counter_A_, delta_counter_B_;
    float angular_velocity_A_, angular_velocity_B_; // rad. / sec.

    uint16_t counter_reference_value_;
    float radian_per_pulse_;
};


#endif