#pragma once

//include
#include <ros/ros.h>
#include <iostream>
#include <chrono>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "rogi_link_msgs/RogiLink.h"

class Throw_Ball_Commander
{
    public:
    Throw_Ball_Commander();
    ~Throw_Ball_Commander(){};
    
    private:
}