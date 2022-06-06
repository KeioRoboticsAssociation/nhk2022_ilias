#pragma once

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "rogi_link_msgs/RogiLink.h"
#include <math.h>

const char ELV_MT = 0x0F;
const char TRN_MT = 0x10;

const float GEAR_PROPORTION = 10;
const float ELV_GAIN = 0;
const float misalignment = -0.08333;
float MAX_ELV = 20; //仰角最大値
float MAX_TRN = 0.222222222222222; //振り角最大値

class Auto_Aimer
{
    public:
    //constructor
        Auto_Aimer(ros::NodeHandle &_nh, int &_loop_rate, int &_lost_time_threshold);
    //destructor
        ~Auto_Aimer(){};

    private:
    //handle
        ros::NodeHandle &nh;

    //publisher
        ros::Publisher aim_pub;
        ros::Publisher cmd_pub;

        // subscribers
        ros::Subscriber sub_emergence;  //emergency_flag
        ros::Subscriber sub_connection; //connection_flag
        ros::Subscriber sub_teleop;     //teleop_flag

        ros::Subscriber sub_target;     //LIDARでの敵座標
        ros::Subscriber sub_joy;        //joyの指示

    //configurations
        int loop_rate;                  //looprate
        int lost_time_threshold;        //しきい値

        // timer
        std::chrono::system_clock::time_point last_sub_vel_time;

    //variables
        float target_x;                 //標的のx座標
        float target_y;                 //標的のy座標

        float target_distance;          //標的の距離
        float target_theta;             //標的の偏角

        float cmd_ELV;                  //上下の操作量
        float cmd_TRN;                  //左右の操作量

    //flags
        bool emergency_stop_flag;       //緊急停止
        bool connection_flag;           //接続
        bool teleop_flag;               //trueで手動

        // methods
        // initializers
        void init();

        //callbacks
        void emergence_callback(const std_msgs::Empty::ConstPtr &msg);
        void connection_callback(const std_msgs::Bool::ConstPtr &msg);
        void teleop_callback(const std_msgs::Bool::ConstPtr &msg);
        void target_callback(const std_msgs::Float32MultiArray &msg);
        void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);

        // others
        void publishMsg();
        void autoAimer();
        void handAimer();
        void update();
};