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
    float limit_UD = 30;
    float limit_RL = 30;
    float rise_rate = 5;
    float roll_rate = 0.5;
    float motor_click = 36; //モーターのギア数
    float luck_click = 80;//ラックのギア数

    Throw_Ball_Commander commander(nh, loop_rate, lost_time_threshold,
                                   limit_UD, 
                                   limit_RL, rise_rate, roll_rate, neck_length);
}