#include "ultrasonic.h"
#include <chrono>

ULTRASONIC::ULTRASONIC(PinName trigPin, PinName echoPin, float updateSpeed, float timeout)
:trig_pin_(trigPin), echo_pin_(echoPin)
{
    update_speed_ms_ = updateSpeed;
    timeout_ms_      = timeout;
    timer_.start();
};

ULTRASONIC::ULTRASONIC(PinName trigPin, PinName echoPin, float updateSpeed, float timeout, void onUpdate(int))
:trig_pin_(trigPin), echo_pin_(echoPin)
{
    _onUpdateMethod  = onUpdate; // function.
    update_speed_ms_ = updateSpeed;
    timeout_ms_      = timeout;
    timer_.start();
};

void ULTRASONIC::_startT()
{ 
    if(timer_.read_us() > 600) { // 0.6 ms
        timer_.reset ();
    }
    start = timer_.read_us();
};
    
void ULTRASONIC::_updateDist()
{
    end  = timer_.read_us();
    done = 1;
    _distance = (end - start)/6;       
    timeout_.detach();
    // timeout_.attach(callback(this,&ULTRASONIC::_startTrig), _updateSpeed);   
    timeout_.attach(callback(this,&ULTRASONIC::_startTrig), operator""ms(update_speed_ms_));   
};

void ULTRASONIC::_startTrig(void)
{
        timeout_.detach();
        trig_pin_ = 1;
        wait_us(10);        
        done = 0;            
        echo_pin_.rise(callback(this, &ULTRASONIC::_startT));   
        echo_pin_.fall(callback(this, &ULTRASONIC::_updateDist));
        echo_pin_.enable_irq ();
        // timeout_.attach(callback(this,&ULTRASONIC::_startTrig), timeout_ms);
        timeout_.attach(callback(this,&ULTRASONIC::_startTrig), operator""ms(timeout_ms_));
        trig_pin_ = 0;                     
};

int ULTRASONIC::getCurrentDistance(void)
{
    return _distance;
};

void ULTRASONIC::pauseUpdates(void)
{
    timeout_.detach();
    echo_pin_.rise(NULL);
    echo_pin_.fall(NULL);
};

void ULTRASONIC::startUpdates(void)
{
    _startTrig();
};

void ULTRASONIC::changeUpdateSpeed(float updateSpeed)
{
    update_speed_ms_ = updateSpeed;
};

float ULTRASONIC::getUpdateSpeed()
{
    return update_speed_ms_;
};

int ULTRASONIC::isUpdated(void)
{
    //printf("%d", done);
    d = done;
    done = 0;
    return d;
};

void ULTRASONIC::checkDistance(void)
{
    if(isUpdated())
    {
        (*_onUpdateMethod)(_distance);
    }
};
