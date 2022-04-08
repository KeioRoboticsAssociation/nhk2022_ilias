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

    sub_target = nh.subscribe("/target",1,&Throw_Ball_Commander::target_sub_callback,this);
    sub_shot = nh.subscribe("/shot",1,&Throw_Ball_Commander::shot_flag_callback,this);
    sub_emergency_stop = nh.subscribe("/emergency_stop_flag", 1, &Throw_Ball_Commander::emergency_stop_callback, this);
    last_sub_vel_time = std::chrono::system_clock::now();

    update();
}

//initializers
void Throw_Ball_Commander::init_drivers()
{
    rogi_link_msgs::RogiLink init_msg;

    init_msg.id = LRMD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    init_msg.id = RRMD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    init_msg.id = NKUD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    init_msg.id = NKRL << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    init_msg.id = MZSV << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);
}

void Throw_Ball_Commander::init_variables()
{
    target_x = 0;
    target_y = 0;
    target_distance = 0;
    target_theta = 0;
    
    emergency_stop_flag = false;
    connection_flag = false;
    fire_flag = false;
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
    target_x = cmd_vel->linear.x;
    target_y = cmd_vel->linear.y;

    target_distance = 0;
    target_theta = 0;

}

void Throw_Ball_Commander::shot_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
    fire_flag = msg->data;
}

void Throw_Ball_Commander::emergency_stop_callback(const std_msgs::Empty::ConstPtr &msg)
{
    target_x = 0;
    target_y = 0;
    target_distance = target_x * target_x + target_y * target_y;
    target_theta = atan(target_y/target_x);

    emergency_stop_flag = !emergency_stop_flag;
}

void Throw_Ball_Commander::ConnectionFlagCallback(const std_msgs::Bool::ConstPtr &msg)
{
    connection_flag= msg->data;
}
//caliculators
void Throw_Ball_Commander::convert_theta_to_click()
{

}

void Throw_Ball_Commander::cal_aimming()
{
    
}

bool Throw_Ball_Commander::isSubscribed() {
    auto current_time = std::chrono::system_clock::now();
    const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_sub_vel_time).count();

    if (vel_elapsed < lost_time_threshold) {
        return true;
    } else {
        return false;
    }
}

void Throw_Ball_Commander::reset()
{
    ROS_ERROR_ONCE("throw_ball_commander: unable to subscribe topics. Reset variables...");

    target_x = 0;
    target_y = 0;
    target_distance = 0;
    target_theta = 0;
}

void Throw_Ball_Commander::update()
{
    ros::Rate r(loop_rate);

    while(ros::ok() && connection_flag==false){
        ROS_WARN_ONCE("waiting for serial connection");
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("serial connected");
    init_drivers();
    init_variables();

    while(ros::ok())
    {
        if(isSubscribed())
        {

        }else
        {
            reset();
        }
    }
}
