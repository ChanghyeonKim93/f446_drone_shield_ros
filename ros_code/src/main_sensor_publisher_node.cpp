#include <iostream>
#include <string>

#include <ros/ros.h>
#include "sensor_publisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_publisher_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("sensor_publisher_node - starts.");

	try{
        SensorPublisher sensor_publisher(nh);
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("sensor_publisher_node - terminated.");
	return 0;
}