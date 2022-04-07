#include "throw_ball_commander.h"

Throw_Ball_Commander::Throw_Ball_Commander(ros::NodeHandle &_nh,int &_loop_rate, int &_lost_time_threshold, 
                        float &_neck_length, float & _rise_rate,float &_motor_click, float &_luck_click)
:nh(_nh),loop_rate(_loop_rate),lost_time_threshold(_lost_time_threshold),
 neck_length(_neck_length),rise_rate(_rise_rate),motor_click(_motor_click),luck_click(_luck_click)
{
    ROS_INFO("Crating Throw_Ball_commander");
    ROS_INFO_STREAM("loop_rate: " << loop_rate);
    ROS_INFO_STREAM("lost_time_threshold: " << lost_time_threshold);
    ROS_INFO_STREAM("neck_length: " << neck_length);
    ROS_INFO_STREAM("rise_rate: " << rise_rate);
    ROS_INFO_STREAM("motor_click: " << motor_click);
    ROS_INFO_STREAM("luck_click: " << luck_click);

    init_variables();
    sub_target = nh.subscribe("/target",1,&Throw_Ball_Commander::target_sub_callback,this);
    sub_shot = nh.subscribe("/shot",1,&Throw_Ball_Commander::shot_flag_callback,this);
    last_sub_vel_time = std::chrono::system_clock::now();
}

//initializers
void Throw_Ball_Commander::init_drivers()
{

}

void Throw_Ball_Commander::init_variables()
{

}

//publishers
void Throw_Ball_Commander::aim_commander()
{

}

void Throw_Ball_Commander::shot_commander()
{

}

//subscribers
void Throw_Ball_Commander::target_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    
}

void Throw_Ball_Commander::shot_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{

}

void Throw_Ball_Commander::emergency_stop_callback(const std_msgs::Empty::ConstPtr &msg)
{

}

//caliculators
void Throw_Ball_Commander::convert_theta_to_click()
{

}

void Throw_Ball_Commander::cal_aimming()
{

}

void Throw_Ball_Commander::reset()
{

}
void Throw_Ball_Commander::update()
{
    
}
