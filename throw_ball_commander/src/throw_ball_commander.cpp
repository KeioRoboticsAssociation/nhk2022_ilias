#include "throw_ball_commander.h"

Throw_Ball_Commander::Throw_Ball_Commander(ros::Nodehandle &_nh,int &_loop_rate, int &_lost_time_threshold
                        float &_neck_length, float & _rise_rate,float &_motor_click, float &_luck_click)
:nh(_nh),loop_rate(_loop_rate),lost_time_threshold(_lost_time_threshold),
 neck_length(_neck_length),rise_rate(_rise_rate),motor_click(_motor_click),luck_click(_luck_click)
{


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
void Throw_Ball_Commander::target_sub_callback()
{

}

void Throw_Ball_Commander::shot_flag_callback()
{

}

//caliculators
void Throw_Ball_Commander::convert_theta_t0_click()
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
