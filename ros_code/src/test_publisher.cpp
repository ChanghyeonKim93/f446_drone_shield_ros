#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_publisher");

    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("test_publisher - STARTS.");
	
    ros::Publisher pub = nh.advertise<std_msgs::UInt16MultiArray>("/serial/pwm",1);

	try{
        uint16_t pwm_values[80] = {0,};
        uint16_t a = 0;
        for(int jj = 0; jj < 80; ++jj){
            pwm_values[jj] = a; 
            a += 50;
        }
        uint8_t cnt[8] = {0,2,4,6,8,10,12,14};
		if(ros::ok()){
            ros::Rate rate(400);
            while(ros::ok()){
                std_msgs::UInt16MultiArray msg;
                for(int i = 0; i < 8; ++i){
                    ++cnt[i];
                    if(cnt[i]==80) cnt[i]=0;
                    msg.data.push_back(pwm_values[cnt[i]]);
                }
                pub.publish(msg);
                ros::spinOnce();
                rate.sleep();
            }
		}
		else{
			throw std::runtime_error("ros not ok");
		}
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("test_publisher - TERMINATED.");
	return 0;
}