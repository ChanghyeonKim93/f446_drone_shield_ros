#include "motor_encoder.h"

// PULSE_PER_MOTOR_TURN*GEAR_RATIO          == PULSE_PER_ROTATION
// RADIAN_PER_ROTATION / PULSE_PER_ROTATION == RADIAN_PER_PULSE
// delta_counter * RADIAN_PER_PULSE == delta_radian
MotorEncoder::MotorEncoder(int update_speed_ms)
: encoder_A_(), encoder_B_(), timer_A_(),timer_B_(),
update_speed_ms_(update_speed_ms), counter_reference_value_(32767),
radian_per_pulse_(RADIAN_PER_ROTATION/((float)PULSE_PER_ROTATION)), ticker_()
{
    inv_period_sec_ = 1000.0f/(float)(update_speed_ms_);
    // Encoder initialize
    EncoderInit(&encoder_A_, &timer_A_, TIM3, 0xffff, TIM_ENCODERMODE_TI12);  // counting on both A&B inputs, 4 ticks per cycle, full 16-bit count
    EncoderInit(&encoder_B_, &timer_B_, TIM4, 0xffff, TIM_ENCODERMODE_TI12);

    // Initialize counter    
    __HAL_TIM_SET_COUNTER(&timer_A_, counter_reference_value_);
    __HAL_TIM_SET_COUNTER(&timer_B_, counter_reference_value_);

    ticker_.attach(callback(this,&MotorEncoder::callbackUpdateAngularVelocity_AB),  operator""ms(update_speed_ms_));
};

float MotorEncoder::getAngularVelocity_A(){
    return angular_velocity_A_;
};

float MotorEncoder::getAngularVelocity_B(){
    return angular_velocity_B_;
};

int16_t MotorEncoder::getDeltaCounter_A(){
    return delta_counter_A_;
};

int16_t MotorEncoder::getDeltaCounter_B(){
    return delta_counter_B_;
};

void MotorEncoder::callbackUpdateAngularVelocity_AB()
{
    // Get counter and direction
    counter_A_ = __HAL_TIM_GET_COUNTER(&timer_A_);
    counter_B_ = __HAL_TIM_GET_COUNTER(&timer_B_);
    dir_A_     = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer_A_); // direction might not be used.
    dir_B_     = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timer_B_);

    // Reset counter
    __HAL_TIM_SET_COUNTER(&timer_A_, counter_reference_value_);
    __HAL_TIM_SET_COUNTER(&timer_B_, counter_reference_value_);

    // Calculate angular velocity
    delta_counter_A_ = (int32_t)counter_A_-(int32_t)counter_reference_value_;
    delta_counter_B_ = (int32_t)counter_B_-(int32_t)counter_reference_value_;

    angular_velocity_A_ = ((float)delta_counter_A_ * radian_per_pulse_)*inv_period_sec_;
    angular_velocity_B_ = ((float)delta_counter_B_ * radian_per_pulse_)*inv_period_sec_;

    // timeout_.detach();
    // timeout_.attach(callback(this,&MotorEncoder::callbackUpdateAngularVelocity_AB),  operator""ms(update_speed_ms_));
    // printf("E1: %d%s, E2: %d%s\r\n", count3, dir3 == 0 ? "+":"-", count4, dir4 == 0 ? "+":"-" );
    
    // printf("ang A: %5d , ang B: %5d", delta_count_A, delta_count_B);
};