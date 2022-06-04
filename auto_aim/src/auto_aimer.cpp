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