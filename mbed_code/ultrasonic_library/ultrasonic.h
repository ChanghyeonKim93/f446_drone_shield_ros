#ifndef _MBED_ULTRASONIC_H_
#define _MBED_ULTRASONIC_H_

#include "mbed.h"

class ULTRASONIC
{
    public:
        /**iniates the class with the specified trigger pin, echo pin, update speed and timeout**/
        ULTRASONIC(PinName trigPin, PinName echoPin, float updateSpeed, float timeout);
        /**iniates the class with the specified trigger pin, echo pin, update speed, timeout and method to call when the distance changes**/
        ULTRASONIC(PinName trigPin, PinName echoPin, float updateSpeed, float timeout, void onUpdate(int));
       
        /**pauses measuring the distance**/
        void pauseUpdates(void);
        /**starts mesuring the distance**/
        void startUpdates(void);

    public:
        /**call this as often as possible in your code, eg. at the end of a while(1) loop,
        and it will check whether the method you have attached needs to be called**/
        void checkDistance(void);
        /** returns the last measured distance**/
        int getCurrentDistance(void);
        /**gets whether the distance has been changed since the last call of isUpdated() or checkDistance()**/
        int isUpdated(void);

    public:
        /**changes the speed at which updates are made**/
        void changeUpdateSpeed(float updateSpeed);
        /**gets the speed at which updates are made**/
        float getUpdateSpeed(void);

    private:
        DigitalOut  trig_pin_;
        InterruptIn echo_pin_;
        Timer   timer_;
        Timeout timeout_;
        int _distance;
        unsigned long long update_speed_ms_;
        int start;
        int end;
        volatile int done;
        void (*_onUpdateMethod)(int);
        void _startT(void);
        void _updateDist(void);
        void _startTrig(void);
        unsigned long long timeout_ms_;
        int d;
};
#endif