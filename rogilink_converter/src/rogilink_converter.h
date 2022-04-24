#pragma once

#include <ros/ros.h>
#include <queue>
#include <std_msgs/Float32MultiArray.h>
#include "rogi_link_msgs/RogiLink.h"

class Rogilink_Converter
{
    public:
        Rogilink_Converter(ros::NodeHandle &_nh, int _loop_rate, int _lost_time_threshold);
        ~Rogilink_Converter(){};

    private:
    //Handles
        ros::NodeHandle &nh;

        ros::Subscriber sub_conv;
        ros::Publisher pub_06;
        ros::Publisher pub_07;
        ros::Publisher pub_0f;
        ros::Publisher pub_10;
        ros::Publisher pub_12;

        // Variables
        int loop_rate;
        int lost_time_threshold;

        std::queue<rogi_link_msgs::RogiLink> que;
        std::chrono::system_clock::time_point last_sub_vel_time_;

        void sub_conv_callback(rogi_link_msgs::RogiLink &msg);
        void pub_msgs();
        void update();
};