#ifndef _SENSOR_PUBLISHER_H_
#define _SENSOR_PUBLISHER_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include "union_struct.h"

#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010

class SensorPublisher{
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
    double adc_data_[4];

    // ROS related
    ros::NodeHandle nh_;
    ros::Subscriber sub_serial_;

    ros::Publisher pub_imu_;
    ros::Publisher pub_battery_state_[4];

private:
    void callbackSerial(const std_msgs::Int8MultiArray::ConstPtr& msg){
        // ROS_INFO_STREAM("Data recv: " << msg->data.size() );
        
        if(msg->data.size() == 33){
            SHORT_UNION val;
            val.bytes_[0] = msg->data[0];
            val.bytes_[1] = msg->data[1];
            acc_[0] = (double)val.short_ * acc_scale_;

            val.bytes_[0] = msg->data[2];
            val.bytes_[1] = msg->data[3];
            acc_[1] = (double)val.short_ * acc_scale_;

            val.bytes_[0] = msg->data[4];
            val.bytes_[1] = msg->data[5];
            acc_[2] = (double)val.short_ * acc_scale_;


            val.bytes_[0] = msg->data[6];
            val.bytes_[1] = msg->data[7];
            gyro_[0] = (double)val.short_ * gyro_scale_;

            val.bytes_[0] = msg->data[8];
            val.bytes_[1] = msg->data[9];
            gyro_[1] = (double)val.short_ * gyro_scale_;

            val.bytes_[0] = msg->data[10];
            val.bytes_[1] = msg->data[11];
            gyro_[2] = (double)val.short_ * gyro_scale_;


            val.bytes_[0] = msg->data[12];
            val.bytes_[1] = msg->data[13];
            mag_[0] = (double)val.short_ * mag_scale_;

            val.bytes_[0] = msg->data[14];
            val.bytes_[1] = msg->data[15];
            mag_[1] = (double)val.short_ * mag_scale_;

            val.bytes_[0] = msg->data[16];
            val.bytes_[1] = msg->data[17];
            mag_[2] = (double)val.short_ * mag_scale_;

            USHORT_UNION sec;
            UINT_UNION   usec;
            sec.bytes_[0]  = msg->data[18];  sec.bytes_[1] = msg->data[19];
            usec.bytes_[0] = msg->data[20]; usec.bytes_[1] = msg->data[21];
            usec.bytes_[2] = msg->data[22]; usec.bytes_[3] = msg->data[23];

            uint8_t cam_trigger_state = msg->data[24];
            time_ = ((double)sec.ushort_ + (double)usec.uint_/1000000.0);

            // AnalogRead data
            USHORT_UNION adc[4];
            adc[0].bytes_[0] = msg->data[25]; adc[0].bytes_[1] = msg->data[26];
            adc[1].bytes_[0] = msg->data[27]; adc[1].bytes_[1] = msg->data[28];
            adc[2].bytes_[0] = msg->data[29]; adc[2].bytes_[1] = msg->data[30];
            adc[3].bytes_[0] = msg->data[31]; adc[3].bytes_[1] = msg->data[32];

            
            // Fill IMU data
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time::now();

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
            for(int j = 0; j < 4; ++j){
                msg_bat[j].header.stamp = ros::Time::now();
                msg_bat[j].voltage = (float)adc[j].ushort_*analog_in_scaler;
                pub_battery_state_[j].publish(msg_bat[j]);
            }
        }
    };  

    void run(){
        ros::Rate rate(20000);
        while(ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
    }

public:
    SensorPublisher(ros::NodeHandle& nh) 
    : nh_(nh) 
    {
        acc_scale_ = 8.0/32768.0 * 9.81; // m/s2
        gyro_scale_ = 1000.0/32768.0/(180.0)*M_PI; // rad/s
        mag_scale_   = 10.0*4219.0/32760.0; // milliGauss
        // Subscriber
        sub_serial_ = nh.subscribe<std_msgs::Int8MultiArray>("/serial/pc/from_fmu",1, &SensorPublisher::callbackSerial, this);

        // publisher
        pub_imu_ = nh.advertise<sensor_msgs::Imu>("/mpu9250/imu",1);
        pub_battery_state_[0] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/0",1);
        pub_battery_state_[1] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/1",1);
        pub_battery_state_[2] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/2",1);
        pub_battery_state_[3] = nh.advertise<sensor_msgs::BatteryState>("/battery_state/3",1);
        
        this->run();
    };


};
#endif