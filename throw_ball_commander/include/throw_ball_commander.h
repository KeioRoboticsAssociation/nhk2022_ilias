#pragma once

//include
#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "rogi_link_msgs/RogiLink.h"

// Hard IDs
const char RRMD = 0x0a;//Right Roller
const char LRMD = 0x0b;//Left Roller
const char NKUD = 0x0c;//Neck Motor Up-Down
const char NKRL = 0x0d;//Neck Motor Right-Left
const char MZSV_1 = 0x10;//First Servo Motor in the Magazine
const char MZSV_2 = 0x11;//Second Servo Motor in the Magazine
const char MZSV_3 = 0x12;//Third Servo Motor in the Magazine 


class Throw_Ball_Commander
{
    public:
    //Constructor & Destructor
    Throw_Ball_Commander();
    ~Throw_Ball_Commander(){};
    
    private:
    //Handlers
    ros::NodeHandle &nh_;

    //Publishers for gazebo
    ros::Publisher pub_RR;//publisher for the Right Roller
    ros::Publisher pub_LR;//publisher for the Left Roller
    ros::Publisher pub_NKUD;//publisher for the motor to move the neck up and down 
    ros::Publisher pub_NKRL;//publisher for the motor to moce the neck right and left
    ros::Publisher pub_SV1;//publisher for Servo Motor 1
    ros::Publisher pub_SV2;//publisher for Servo Motor 2
    ros::Publisher pub_SV3;//publisher for Servo Motor 3
    

    //Publishers for real
    ros::Publisher pub_ctrl;
    

    //Subscrivers
    ros::Subscriber sub_target;

}