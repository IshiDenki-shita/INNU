#include <ros/ros.h>
//#include <pigpio.h> 
#include <pigpiod_if2.h>
//#include <iostream>
//#include <chrono>
//#include <thread>
#include "std_msgs/Float32MultiArray.h"

#include <std_msgs/Bool.h>
#include "sensor_msgs/LaserScan.h"
#define RAD2DEG(x) ((x)*180./M_PI)

float deg_0;
float deg_90;
float deg_180;
float deg_270;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    deg_0 = (scan->ranges[0]);
    deg_90 = (scan->ranges[286]);
    deg_180 = (scan->ranges[572]);
    deg_270 = (scan->ranges[857]);
//    ROS_INFO("[%f, %f, %f, %f]", deg_0, deg_90, deg_180, deg_270);
}

float beacon_F;
float beacon_A;
float beacon_D;

void chaterCallback(const std_msgs::Float32MultiArray & msg)
{
	int num = msg.data.size();
	for (int i=0; i<num; i++)
		ROS_INFO("[%i] %f", i, msg.data[i]);
		beacon_F = (msg.data[0]);
		beacon_A = (msg.data[1]);
		beacon_D = (msg.data[2]);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "servo");

    ros::NodeHandle n;

    int pi;
    pi = pigpio_start(NULL,NULL);

    int i = 0;

    ros::Rate rate(50);
    set_servo_pulsewidth(pi,26,1500);

    set_PWM_frequency(pi, 20, 3000);
    set_PWM_frequency(pi, 21, 3000);

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

     while(ros::ok()){

        if (deg_0 > 0.5){
 	        set_PWM_dutycycle(pi, 20, 0);
	        set_PWM_dutycycle(pi, 21, 250);
        }else{
            set_PWM_dutycycle(pi, 20, 250);
            set_PWM_dutycycle(pi, 21, 0);
        }

	ROS_INFO("0=%f, 90=%f, 180=%f, 270=%f", deg_0, deg_90, deg_180, deg_270);
	ROS_INFO("F=%f, A=%f, D=%F", beacon_F, beacon_A, beacon_D);

//	if(i == 0){
//      set_servo_pulsewidth(pi,26,500);
//	  i = 1;
//	}else if(i == 1){
//	  set_servo_pulsewidth(pi,26,2500);
//	  i = 2;
//	}else{
//	  set_servo_pulsewidth(pi,26,1500);
//	  i = 0;
//	}

//	ROS_INFO("%d",i);

	ros::spin();
	rate.sleep();
  }
  set_servo_pulsewidth(pi,26,0);
  return 0;
}
