#ifndef _DRONE_MOTOR_PWM_H_
#define _DRONE_MOTOR_PWM_H_
#include "mbed.h"
#include "parameters.h"

class DroneMotorPwm {
private:
    PwmOut pwm_[8]; // Maximum 8 motors

public:
    DroneMotorPwm();

    void setPWM_all(uint16_t duty[NUM_MOTORS]);
    void setPWM(uint16_t motor_number, uint16_t duty);

};


#endif