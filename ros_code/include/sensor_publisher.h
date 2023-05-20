#ifndef _SENSOR_PUBLISHER_H_
#define _SENSOR_PUBLISHER_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>

#include "union_struct.h"

#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010

class SensorPublisher {
private:
    // IMU related
    double time_;
    double acc_[3];
    double gyro_[3];
    double mag_[3];

    double acc_scale_;
    double gyro_scale_;
    double mag_scale_;

    // ADC related
    double adc_data_[2];

    // ROS related
    ros::NodeHandle nh_;
    ros::Subscriber sub_serial_;

    ros::Publisher pub_imu_;
    ros::Publisher pub_battery_state_[2];
    ros::Publisher pub_sonar_;

    ros::Time rostime_prev_;

    bool is_reference_time_initialized_ = {false};
    ros::Time rostime_reference_;
    double mcutime_reference_;

private:
    void callbackSerial(const std_msgs::UInt8MultiArray::ConstPtr& msg){
        ros::Time rostime_curr = ros::Time::now();

        if(msg->data.size() == 57){
            FLOAT_UNION val;
            int idx = 0;
            val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[0] = (double)val.float_ * acc_scale_;

            idx = 4; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[1] = (double)val.float_ * acc_scale_;

            idx = 8; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            acc_[2] = (double)val.float_ * acc_scale_;

            idx = 12; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[0] = (double)val.float_ * gyro_scale_;

            idx = 16; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[1] = (double)val.float_ * gyro_scale_;

            idx = 20; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            gyro_[2] = (double)val.float_ * gyro_scale_;

            idx = 24; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[0] = (double)val.float_ * mag_scale_;

            idx = 28; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[1] = (double)val.float_ * mag_scale_;

            idx = 32; val.bytes_[0] = msg->data[idx]; val.bytes_[1] = msg->data[++idx]; val.bytes_[2] = msg->data[++idx]; val.bytes_[3] = msg->data[++idx];
            mag_[2] = (double)val.float_ * mag_scale_;

            USHORT_UNION sec;
            UINT_UNION   usec;

            idx = 36;
            sec.bytes_[0]  = msg->data[idx];  sec.bytes_[1] = msg->data[++idx];
            idx = 38;
            usec.bytes_[0] = msg->data[idx]; usec.bytes_[1] = msg->data[++idx];
            usec.bytes_[2] = msg->data[++idx]; usec.bytes_[3] = msg->data[++idx];

            idx = 42;
            uint8_t cam_trigger_state = msg->data[idx];
            time_ = ((double)sec.ushort_ + (double)usec.uint_*1e-6);

            if(!is_reference_time_initialized_){
                rostime_reference_ = ros::Time::now();
                mcutime_reference_ = time_;
                is_reference_time_initialized_=true;
            }

            // AnalogRead data
            USHORT_UNION adc[2];
            idx = 43;
            adc[0].bytes_[0] = msg->data[idx];   adc[0].bytes_[1] = msg->data[++idx];
            adc[1].bytes_[0] = msg->data[++idx]; adc[1].bytes_[1] = msg->data[++idx];

            // Sonar distance
            idx = 47;
            USHORT_UNION sonar_dist_in_mm;
            sonar_dist_in_mm.bytes_[0] = msg->data[idx]; sonar_dist_in_mm.bytes_[1] = msg->data[++idx]; 

            // Encoder data
            FLOAT_UNION encoder_A, encoder_B;
            idx = 49; encoder_A.bytes_[0] = msg->data[idx]; encoder_A.bytes_[1] = msg->data[++idx]; encoder_A.bytes_[2] = msg->data[++idx]; encoder_A.bytes_[3] = msg->data[++idx];
            idx = 53; encoder_B.bytes_[0] = msg->data[idx]; encoder_B.bytes_[1] = msg->data[++idx]; encoder_B.bytes_[2] = msg->data[++idx]; encoder_B.bytes_[3] = msg->data[++idx]; 

            // Fill IMU data
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time(rostime_reference_.toSec() + time_ - mcutime_reference_);

            msg.angular_velocity.x = gyro_[0];
            msg.angular_velocity.y = gyro_[1];
            msg.angular_velocity.z = gyro_[2];
            
            msg.linear_acceleration.x = acc_[0];
            msg.linear_acceleration.y = acc_[1];
            msg.linear_acceleration.z = acc_[2];
            
            // Publish all data
            pub_imu_.publish(msg);

            // Fill battery state data
            float analog_in_scaler = 3.3f/65535.0f;
            sensor_msgs::BatteryState msg_bat[4];
            for(int j = 0; j < 2; ++j){
                msg_bat[j].header.stamp = msg.header.stamp;
                msg_bat[j].voltage = (float)adc[j].ushort_*analog_in_scaler;
                pub_battery_state_[j].publish(msg_bat[j]);
            }

            // Sonar range data
            sensor_msgs::Range msg_sonar;
            msg_sonar.header.stamp = msg.header.stamp;
            msg_sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
            msg_sonar.field_of_view = 10/180.0*M_PI; // radian
            msg_sonar.range = (float)(sonar_dist_in_mm.ushort_)*0.001f; // meters
            msg_sonar.min_range = 0.05f; // meters
            msg_sonar.max_range = 3.3f; // meters

            pub_sonar_.publish(msg_sonar);

            rostime_prev_ = rostime_curr;
        }
    };  

    void run(){
        ros::Rate rate(4000);
        while(ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
    }

public:
    SensorPublisher(ros::NodeHandle& nh) 
    : nh_(nh) 
    {
        acc_scale_   = 9.8065; // m/s2
        gyro_scale_  = M_PI/180.0f; // rad/s
        mag_scale_   = 10.0*4219.0/32760.0; // milliGauss

        // Subscriber
        sub_serial_ = nh.subscribe<std_msgs::UInt8MultiArray>("/serial/pc/from_fmu",1, &SensorPublisher::callbackSerial, this);

        // publisher
        pub_imu_ = nh.advertise<sensor_msgs::Imu>("/icm42605/imu",1);

        pub_battery_state_[0] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/0",1);
        pub_battery_state_[1] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/1",1);

        pub_sonar_ = nh_.advertise<sensor_msgs::Range>("/hcsr04/range",1);
        
        this->run();
    };


};
#endif