#ifndef _SERIAL_COMM_ROS_H_
#define _SERIAL_COMM_ROS_H_

#include <iostream>
#include <ros/ros.h>

#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include "serial_communicator.h"

#define BUF_SIZE 1024

/// @brief This ROS wrapper publishes the messages received from the Nucleo board, and subscribe and send the user message to the Nucleo.
class SerialCommROS{
public:
    SerialCommROS(ros::NodeHandle& nh);
    ~SerialCommROS();

private:
    /// @brief Get ROS parameters.
    void getParameters();

    /// @brief Run the loop.
    void run();

    /// @brief Function is called by the 'sub_msg_to_send_', and send the user message to the Nucleo.
    /// @param msg packet from the user. In this implementation, PWM signals (uint16_t) are only considered.
    void callbackToSend(const std_msgs::UInt16MultiArray::ConstPtr& msg);

private:
    void showSerialStatistics(double dt);


// Packet manipulation (receiving)
private:
    /// @brief If the packet from the Nucleo is ready, this function gives 'true'
    /// @return boolean. true: packet from Nucleo is ready.
    bool isRecvPacketReady();

    /// @brief Get RecvMessage.
    /// @param data received uint8_t data from the Nucleo.
    /// @return length of the received data.
    uint32_t getRecvMessage(unsigned char* data);

// Packet manipulation (sending user's message.)
private:
    void sendMessageToNucleo(unsigned char* data, int len);
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

    // TX messages (message from the Nucleo. Received packet is published as a form of UInt16MultiArray.)
    std::string topicname_msg_to_send_;
    ros::Subscriber sub_msg_to_send_;
    std_msgs::UInt16MultiArray msg_to_send_;

    // RX messages (message which the user want to send to the Nucleo.)
    std::string topicname_msg_recv_;
    ros::Publisher pub_msg_recv_;
    std_msgs::UInt8MultiArray msg_recv_;
};
#endif