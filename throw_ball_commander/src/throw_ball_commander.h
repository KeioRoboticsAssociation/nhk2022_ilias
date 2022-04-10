#pragma once

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "rogi_link_msgs/RogiLink.h"
#include <math.h>

// Hard IDs
const char RRMD = 0x07; // Right Roller
const char LRMD = 0x06; // Left Roller
const char NKUD = 0x0F; // Neck Motor Up-Down
const char NKRL = 0x10; // Neck Motor Right-Left
const char MZSV = 0x0E; // First Servo Motor in the Magazine

class Throw_Ball_Commander
{
    public:
    //constructors and destructors
        Throw_Ball_Commander();
        ~Throw_Ball_Commander(){};

    private:
    //Handle
        ros::NodeHandle &nh;

    //Publishers
        ros::Publisher pub_ctrl;

    //Subscribers
        ros::Subscriber sub_emergence;//emergency_flag
        ros::Subscriber sub_connection;//connection_flag
        
        ros::Subscriber sub_target;//ターゲットの座標
        ros::Subscriber sub_shot;//shot コマンド

        ros::Subscriber sub_bullet;//給弾関係

        ros::Subscriber sub_limitUD;//上下のリミットスイッチ
        ros::Subscriber sub_limitRL;//左右のリミットスイッチ

    //Configurations
        int loop_rate;//制御周期
        int lost_time_threshold;

        float limit_UD;//仰角の最大値
        float limit_RL;//左右振り角の最大値

        float rise_rate;//一回転で何mm上がるか
        float roll_rate;//一回転で何度回るか

    //Timers
        std::chrono::system_clock::time_point last_sub_vel_time;

    //flags
        bool emergency_stop_flag;
        bool connection_flag;
        
        
};
// #pragma once

// // include
// #include <ros/ros.h>
// #include <iostream>
// #include <chrono>

// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/Bool.h>
// #include <geometry_msgs/Twist.h>
// #include "rogi_link_msgs/RogiLink.h"
// #include <math.h>

// // Hard IDs
// const char RRMD = 0x07; // Right Roller
// const char LRMD = 0x06; // Left Roller
// const char NKUD = 0x0F; // Neck Motor Up-Down
// const char NKRL = 0x10; // Neck Motor Right-Left
// const char MZSV = 0x0E; // First Servo Motor in the Magazine

// // define shotter cmd
// enum Shotter
// {
//     INIT_CMD = 0x02,
//     SHOT_CMD = 0x20
// };

// class Throw_Ball_Commander
// {
// public:
//     // Constructor & Destructor
//     Throw_Ball_Commander(ros::NodeHandle &_nh, int &_loop_rate, int &_lost_time_threshold,
//                          float &_neck_length, float &_rise_rate, float &_motor_click, float &_luck_click);
//     ~Throw_Ball_Commander(){};

// private:
//     // Handlers
//     ros::NodeHandle &nh;

//     // Publishers for gazebo

//     // Publishers for real
//     ros::Publisher pub_ctrl; // publisher for aiming target
//     ros::Publisher elevator;
//     // Subscrivers
//     ros::Subscriber limit_UD;//仰角のリミットスイッチ
//     ros::Subscriber limit_RL;//首振りのリミットスイッチ
//     ros::Subscriber bullet_feed;//給弾フラグ
//     ros::Subscriber sub_target;//ターゲットの位置
//     ros::Subscriber sub_shot;//発射フラグ
//     ros::Subscriber sub_emergency_stop;//緊急停止フラグ
//     ros::Subscriber sub_connection;//接続フラグ

//     // Configurations;
//     int loop_rate;
//     int lost_time_threshold;
//     float neck_length;
//     float rise_rate;
//     float motor_click; //モーターのギア数
//     float luck_click;  //ラックのギア数

//     // variables;
//     float target_x;
//     float target_y;

//     float target_distance;
//     float target_theta;

//     float roller_duty;
//     float neckUD_cmd;
//     float neckRL_cmd;

//     // flags
//     bool emergency_stop_flag;
//     bool connection_flag;
    
//     bool neckUD_limit_flag;
//     bool neckRL_limit_flag;
//     bool fire_flag;

//     // Timers
//     std::chrono::system_clock::time_point last_sub_vel_time;

//     // Methods
//     // initializers
//     void init_drivers();
//     void init_variables();
//     // publishers
//     void aim_commander();
//     void shot_commander();
//     // subscribers
//     void target_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel);
//     void shot_flag_callback(const std_msgs::Bool::ConstPtr &msg);
//     void emergency_stop_callback(const std_msgs::Empty::ConstPtr &msg);
//     void ConnectionFlagCallback(const std_msgs::Bool::ConstPtr &msg);
//     // caliculators
//     void converter();
//     void cal_aimming();

//     bool isSubscribed();
//     void reset();
//     void update();
// };