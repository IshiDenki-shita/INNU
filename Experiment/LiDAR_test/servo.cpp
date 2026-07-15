ubuntu@ubuntu:~/Desktop/INNU$ cat Experiment/LiDAR_test/servo.cpp 
#include <ros/ros.h>
//#include <pigpio.h> 
#include <pigpiod_if2.h>
//#include <iostream>
//#include <chrono>
//#include <thread>

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
    ROS_INFO("[%f, %f, %f, %f]", deg_0, deg_90, deg_180, deg_270);
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
	ros::spinOnce();
	rate.sleep();
  }
  set_servo_pulsewidth(pi,26,0);
  return 0;
}
