#ifndef _SERIAL_COMM_ROS_H_
#define _SERIAL_COMM_ROS_H_

#include <iostream>
#include <ros/ros.h>

#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include "serial_communicator.h"

#define BUF_SIZE 1024

class SerialCommROS{
public:
    SerialCommROS(ros::NodeHandle& nh);
    ~SerialCommROS();

private:
    void getParameters();
    void run();

    void callbackToSend(const std_msgs::UInt16MultiArray::ConstPtr& msg);
    
    void showSerialStatistics(double dt);


// Packet manipulation
private:
    bool isPacketReady();
    uint32_t getMessage(unsigned char* data);

    void sendMessage(unsigned char* data, int len);
    int fill16bitsTo8bits(const std_msgs::UInt16MultiArray::ConstPtr& msg, unsigned char* buf_send);

// Serial port related (Boost)
private:
    std::string portname_;
    int         baudrate_;
    std::shared_ptr<SerialCommunicator> serial_communicator_;

    unsigned char buf_send_[BUF_SIZE];
    unsigned char buf_recv_[BUF_SIZE];

// ROS related
private:
    ros::NodeHandle nh_;

    int loop_frequency_;

    // TX messages
    std::string topicname_msg_to_send_;
    ros::Subscriber sub_msg_to_send_;
    std_msgs::UInt16MultiArray msg_to_send_;

    // RX messages
    std::string topicname_msg_recv_;
    ros::Publisher pub_msg_recv_;
    std_msgs::Int8MultiArray msg_recv_;

};
#endif