#include "throw_ball_commander.h"

std::string node_name = "throw_ball";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate = 30;
    int lost_time_threshold = 500;
    float neck_length = 500;
    float rise_rate = 5;
    float motor_click = 36;//モーターのギア数
    float luck_click = 80;//ラックのギア数

    Throw_Ball_Commander commander(nh,loop_rate,lost_time_threshold,neck_length,rise_rate,motor_click,luck_click);
}