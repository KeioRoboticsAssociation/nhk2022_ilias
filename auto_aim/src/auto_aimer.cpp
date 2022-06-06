#include "auto_aimer.h"
std::string node_name = "auto_aimer";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate = 30;
    int lost_time_threshold = 500;

    pnh.getParam("loop_rate", loop_rate);
    pnh.getParam("lost_time_threshold", lost_time_threshold);

    Auto_Aimer auto_aimer(nh, loop_rate, lost_time_threshold);
}

Auto_Aimer::Auto_Aimer(ros::NodeHandle &_nh, int &_loop_rate, int &_lost_time_threshold)
    : nh(_nh), loop_rate(_loop_rate), lost_time_threshold(_lost_time_threshold)
{
    ROS_INFO("Creating auto_aimer");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate);
    ROS_INFO_STREAM("lost_time_threshold [ms]: " << lost_time_threshold);

    // init publisher
    aim_pub = nh.advertise<rogi_link_msgs::RogiLink>("send_serial", 100);
    cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("current_cmd", 100);

    // init subscriber
    sub_emergence = nh.subscribe("/emergency_stop_flag", 1, &Auto_Aimer::emergence_callback, this);
    sub_connection = nh.subscribe("/connection_status", 1, &Auto_Aimer::connection_callback, this);
    sub_teleop = nh.subscribe("/teleop_flag", 1, &Auto_Aimer::teleop_callback, this);
    sub_target = nh.subscribe("/BoH_location", 1, &Auto_Aimer::target_callback, this);
    sub_joy = nh.subscribe("joy", 1, &Auto_Aimer::joy_callback, this);

    last_sub_vel_time = std::chrono::system_clock::now();

    init();
    update();
}

void Auto_Aimer::init()
{
    target_x = 0;
    target_y = 0;
    target_distance = 0;
    target_theta = 0;
    cmd_ELV = 0;
    cmd_TRN = 0;
    emergency_stop_flag = false;
    connection_flag = false;
    teleop_flag = true;
}

void Auto_Aimer::emergence_callback(const std_msgs::Empty::ConstPtr &msg)
{
    emergency_stop_flag = !emergency_stop_flag;
}
void Auto_Aimer::connection_callback(const std_msgs::Bool::ConstPtr &msg)
{
    connection_flag = msg->data;
}
void Auto_Aimer::teleop_callback(const std_msgs::Bool::ConstPtr &msg)
{
    teleop_flag = msg->data;
}
void Auto_Aimer::target_callback(const std_msgs::Float32MultiArray &msg)
{
    target_x = msg.data[0];
    target_y = msg.data[1];

    target_distance = sqrt(target_x * target_x + target_y * target_y);
    target_theta = atan(target_y / target_x);

    ROS_INFO("dist:%f,theta:%f",target_distance,target_theta);
    last_sub_vel_time = std::chrono::system_clock::now();
}

void Auto_Aimer::joy_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
    cmd_ELV += msg->axes[5]/10;
    cmd_TRN += msg->axes[4]/360;
    // ROS_INFO("nya : %f,%f",cmd_ELV,cmd_TRN);
}

void Auto_Aimer::publishMsg()
{
    rogi_link_msgs::RogiLink cmd_msg;
    std_msgs::Float32MultiArray crt_cmd;
    crt_cmd.data.resize(2);
    ROS_INFO("auto_aimer : %f, %f" ,cmd_ELV, cmd_TRN * GEAR_PROPORTION);

    cmd_msg.id = ELV_MT << 6 | 0x03;
    *(float *)(&cmd_msg.data[0]) = cmd_ELV;
    aim_pub.publish(cmd_msg);

    cmd_msg.id = TRN_MT << 6 | 0x03;
    *(float *)(&cmd_msg.data[0]) = cmd_TRN*GEAR_PROPORTION;
    aim_pub.publish(cmd_msg);

    crt_cmd.data[0] = cmd_TRN*GEAR_PROPORTION;
    crt_cmd.data[1] = cmd_ELV;
    cmd_pub.publish(crt_cmd);
}
void Auto_Aimer::autoAimer()
{
    cmd_ELV = -0.4475 * pow(target_distance, 6) + 5.2814 * pow(target_distance, 5) - 24.424 * pow(target_distance, 4) + 56.043 * pow(target_distance, 3) - 67.331 * pow(target_distance, 2) + 44.159 * target_distance - 10.077;
    cmd_TRN = target_theta / 6.28318530718 - misalignment;
    if (cmd_ELV < 0)
        cmd_ELV = 0;
    else if (cmd_ELV > MAX_ELV)
        cmd_ELV = MAX_ELV;

    if (cmd_TRN < 0)
        cmd_TRN = 0;
    else if (cmd_TRN > MAX_TRN)
        cmd_TRN = MAX_TRN;
}
void Auto_Aimer::handAimer()
{
    if(cmd_ELV<0)
    {
        cmd_ELV = 0;
    }
    else if(cmd_ELV>MAX_ELV)
    {
        cmd_ELV = MAX_ELV;
    }

    if(cmd_TRN<0)
    {
        cmd_TRN = 0;
    }
    else if(cmd_TRN>MAX_TRN)
    {
        cmd_TRN = MAX_TRN;
    }
}

void Auto_Aimer::update()
{
    ros::Rate r(loop_rate);

    while (ros::ok() && connection_flag == false)
    {
        ROS_WARN_ONCE("waiting for serial connection");
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("serial connected");

    while(ros::ok())
    {
        if(teleop_flag)
        {
            handAimer();
        }
        else
        {
            autoAimer();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }
}
