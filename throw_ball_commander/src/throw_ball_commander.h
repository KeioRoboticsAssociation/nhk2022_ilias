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
const char MZDC = 0x12; // Reloading Motor
class Throw_Ball_Commander
{
    public:
    //constructors and destructors
        Throw_Ball_Commander(ros::NodeHandle &_nh,int &_loop_rate,int &_lost_time_threshold,
        float &_limit_UD, float &_limit_RL, float &_rise_rate, float &_roll_rate,
        float &_neck_length);
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

        ros::Subscriber sub_cocking; //コッキング
        ros::Subscriber sub_reload; //給弾関係

        ros::Subscriber sub_limitSW;//上下のリミットスイッチ

        ros::Subscriber sub_RL_location;//左右の回転角のモニタ

        // Configurations
        int loop_rate;//制御周期
        int lost_time_threshold;

        float limit_UD;//仰角の最大値
        float limit_RL;//左右振り角の最大値

        float rise_rate;//一回転で何mm上がるか
        float roll_rate;//一回転で何度回るか

        float neck_length;//首の長さ        

        // Timers
        std::chrono::system_clock::time_point last_sub_vel_time;

    //Variables
        float target_x;//標的のx座標
        float target_y;//標的のy座標

        float target_distance;//標的との距離
        float target_theta;//標的の偏角

        float cmd_UD;//上下の操作量
        float cmd_RL;//左右の操作量

        int remaining_bullets;//
        float RL_location;

        // Flags
        bool emergency_stop_flag;//緊急停止
        bool connection_flag;//接続確認

        bool init_flag;//初期化処理か否かのフラグ

        bool shot_flag;//射出フラグ
        bool cocked_flag;//コッキングフラグ
        bool reloading_flag; //リロード要請フラグ
        bool reloaded_flag;//リロード終了フラグ

        bool limit_UD_flag;//リミットスイッチが押されたら立てる
        bool limit_RL_flag;//リミットスイッチが押されたら立てる
        bool send_emergence_flag;//リミットスイッチが押されたら立てる

    // Methods
        // initializers
        void init_drivers();
        void init_variables();
        void init_flags();

        // callbacks
        void emergence_callback(const std_msgs::Empty::ConstPtr &msg);
        void connection_callback(const std_msgs::Bool::ConstPtr &msg);
        void target_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void shot_callback(const std_msgs::Bool::ConstPtr &msg);
        void cocking_callback(const std_msgs::Float32MultiArray &msg);
        void reload_callback(const std_msgs::Bool::ConstPtr &msg);
        void limitSW_callback(const std_msgs::Float32MultiArray &msg);
        void RL_location_callback(const std_msgs::Float32MultiArray &msg);
        
        // others
        bool isSubscribed(); 
        void publishMsg();
        void startup_cal();
        void main_cal();
        void reset();
        void update();
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