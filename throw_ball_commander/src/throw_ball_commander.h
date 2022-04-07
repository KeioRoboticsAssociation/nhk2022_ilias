#pragma once

//include
#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
//#include "rogi_link_msgs/RogiLink.h"
#include <math.h>


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
    Throw_Ball_Commander(ros::NodeHandle &_nh,int &_loop_rate, int &_lost_time_threshold,
                        float &_neck_length, float & _rise_rate,float &_motor_click, float &_luck_click);
    ~Throw_Ball_Commander(){};
    
    private:
    //Handlers
    ros::NodeHandle &nh;

    //Publishers for gazebo    

    //Publishers for real
    ros::Publisher pub_aim;//publisher for aiming target
    ros::Publisher pub_fire;//publisher for shot and charge

    //Subscrivers
    ros::Subscriber sub_target;
    ros::Subscriber sub_shot;
    ros::Subscriber sub_emergency_stop;
    //Configurations;
    int loop_rate;
    int lost_time_threshold;
    float neck_length;
    float rise_rate;
    float motor_click;//モーターのギア数
    float luck_click;//ラックのギア数

    //variables;
    float target_x;
    float target_y;

    float target_distance;
    float target_theta;

    //flags
    bool emergency_stop_flag;
    bool connection_flag;
    bool fire_flag;

    // Timers
    std::chrono::system_clock::time_point last_sub_vel_time;


    //Methods
    //initializers
    void init_drivers();
    void init_variables();
    //publishers
    void aim_commander();
    void shot_commander();
    //subscribers
    void target_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
    void shot_flag_callback(const std_msgs::Bool::ConstPtr &msg);
    void emergency_stop_callback(const std_msgs::Empty::ConstPtr &msg);
    //caliculators
    void convert_theta_to_click();
    void cal_aimming();

    void reset();
    void update();
};