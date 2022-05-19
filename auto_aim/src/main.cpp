#include "BoH_indicator.h"

std::string node_name = "throw_ball";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate = 30;
    int lost_time_threshold = 500;
    float neck_length = 500;
    float limit_UD = 30;
    float limit_RL = 30;
    float rise_rate = 5;
    float roll_rate = 0.5;

    pnh.getParam("loop_rate", loop_rate);
    pnh.getParam("lost_time_threshold", lost_time_threshold);
    pnh.getParam("neck_length", neck_length);
    pnh.getParam("limit_UD", limit_UD);
    pnh.getParam("limit_RL", limit_RL);
    pnh.getParam("rise_rate", rise_rate);
    pnh.getParam("roll_rate", roll_rate);
    BoH_Indicator BoH_indicator;
}