#include "rogilink_converter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rogilink_converter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate = 30;
    int lost_time_threshold = 500;
    pnh.getParam("loop_rate", loop_rate);
    pnh.getParam("lost_time_threshold", lost_time_threshold);
    
    Rogilink_Converter rogi_conv(nh, loop_rate, lost_time_threshold);
}