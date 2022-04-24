#include "rogilink_converter.h"

Rogilink_Converter::Rogilink_Converter(ros::NodeHandle &_nh,int _loop_rate, int _lost_time_threshold)
:nh(_nh),loop_rate(_loop_rate),lost_time_threshold(_lost_time_threshold)
{
    ROS_INFO("Creating rogilink converter");
    ROS_INFO_STREAM("loop rate [Hz]:" << loop_rate);
    ROS_INFO_STREAM("lost_time_threshold [ms]:" << lost_time_threshold);

    sub_conv = nh.subscribe("/send_serial", 100, &Rogilink_Converter::sub_conv_callback,this);
    pub_06 = nh.advertise<std_msgs::Float32MultiArray>("/control_06",100);
    pub_07 = nh.advertise<std_msgs::Float32MultiArray>("/control_07", 100);
    pub_0f = nh.advertise<std_msgs::Float32MultiArray>("/control_0f", 100);
    pub_10 = nh.advertise<std_msgs::Float32MultiArray>("/control_10", 100);
    pub_12 = nh.advertise<std_msgs::Float32MultiArray>("/control_12", 100);

    update();
}

void Rogilink_Converter::sub_conv_callback(rogi_link_msgs::RogiLink &msg)
{
    que.push(msg);
}

void Rogilink_Converter::pub_msgs()
{
    rogi_link_msgs::RogiLink msg;
    std_msgs::Float32MultiArray msg2;
    memcpy(&msg2.data[0], msg.data.begin(), msg.data.size());
    msg = que.front();
    que.pop();

    switch(msg.id >> 6)
    {
        case 0x06:
            pub_06.publish(msg2);
            break;
        case 0x07:
            pub_07.publish(msg2);
            break;
        case 0x0f:
            pub_0f.publish(msg2);
            break;
        case 0x10:
            pub_10.publish(msg2);
            break;
        case 0x12:
            pub_12.publish(msg2);
            break;
        default:
            break;
    }
}

void Rogilink_Converter::update()
{
    ros::Rate r(loop_rate);

    while(ros::ok())
    {
        if(!que.empty())
        {
            pub_msgs();
        }
        ros::spinOnce();
        r.sleep();
    }
}