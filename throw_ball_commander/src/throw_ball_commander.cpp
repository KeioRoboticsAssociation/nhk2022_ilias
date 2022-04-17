#include "throw_ball_commander.h"

Throw_Ball_Commander::Throw_Ball_Commander(ros::NodeHandle &_nh, int &_loop_rate, int &_lost_time_threshold,
                                           float &_limit_UD, float &_limit_RL, float &_rise_rate, float &_roll_rate
                                           ,float &_neck_length)
                                           :nh(_nh),loop_rate(_loop_rate),lost_time_threshold(_lost_time_threshold),
                                           limit_UD(_limit_UD),limit_RL(_limit_RL),rise_rate(_rise_rate),roll_rate(_roll_rate)
                                           ,neck_length(_neck_length)
{
    ROS_INFO("Creating throw_ball_commander");
    ROS_INFO_STREAM("loop_rate [Hz]: " << loop_rate);
    ROS_INFO_STREAM("lost_time_threshold [ms]: " << lost_time_threshold);
    ROS_INFO_STREAM("limit_UD [deg]: " << limit_UD);
    ROS_INFO_STREAM("limit_RL [deg]: " << limit_RL);
    ROS_INFO_STREAM("rise_rate [mm / deg]: " << rise_rate);
    ROS_INFO_STREAM("roll_rate [deg / deg]: " << roll_rate);

    pub_ctrl = nh.advertise<rogi_link_msgs::RogiLink>("send_serial", 100);
    // init subscriver
    sub_emergence = nh.subscribe("/emergency_stop_flag", 1, &Throw_Ball_Commander::emergence_callback, this);
    sub_connection = nh.subscribe("/connection_status", 1, &Throw_Ball_Commander::connection_callback, this);
    sub_target = nh.subscribe("/target_location",1,&Throw_Ball_Commander::target_callback, this);
    sub_shot = nh.subscribe("/shot_flag",1,&Throw_Ball_Commander::shot_callback, this);
    sub_cocking = nh.subscribe("/rcv_serial_0E", 1, &Throw_Ball_Commander::cocking_callback, this);
    sub_reload = nh.subscribe("/bullet_flag", 1, &Throw_Ball_Commander::reload_callback, this);
    sub_limitSW = nh.subscribe("/", 1, &Throw_Ball_Commander::limitSW_callback, this);
    sub_RL_location = nh.subscribe("/rcv_serial_10", 1, &Throw_Ball_Commander::RL_location_callback, this);

    last_sub_vel_time = std::chrono::system_clock::now();

    init_drivers();
    init_variables();
    init_flags();

    update();
}

// initializers
void Throw_Ball_Commander::init_drivers()
{
    rogi_link_msgs::RogiLink init_msg;
    
    //RightRoller
    init_msg.id = RRMD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    // LeftRoller
    init_msg.id = LRMD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    // NeckUpDown
    init_msg.id = NKUD << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    // NeckRightLeft
    init_msg.id = NKRL << 6 | 0x02;
    init_msg.data[0] = 1;
    pub_ctrl.publish(init_msg);

    // MagazineServo
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

    cmd_RL = 0;
    cmd_UD = 0;
    remaining_bullets = 0;
}

void Throw_Ball_Commander::init_flags()
{
    init_flag = true;

    shot_flag = false;
    reloading_flag = false;
    reloaded_flag = true;

    send_emergence_flag = false;
}

// callbacks
void Throw_Ball_Commander::emergence_callback(const std_msgs::Empty::ConstPtr &msg)
{
    emergency_stop_flag = !emergency_stop_flag;
}

void Throw_Ball_Commander::connection_callback(const std_msgs::Bool::ConstPtr &msg)
{
    connection_flag = msg->data;
}

void Throw_Ball_Commander::target_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    target_x = msg->linear.x;
    target_y = msg->linear.y;

    target_distance = sqrt(target_x * target_x + target_y * target_y);
    target_theta = atan(target_y / target_x);

    last_sub_vel_time = std::chrono::system_clock::now();

    if(emergence_callback)
        ROS_ERROR_ONCE("Error: Emergency Stopping...");
}

void Throw_Ball_Commander::shot_callback(const std_msgs::Bool::ConstPtr &msg)
{
    shot_flag = msg->data;
}

void Throw_Ball_Commander::cocking_callback(const std_msgs::Float32MultiArray &msg)
{
    cocked_flag = true;
}

void Throw_Ball_Commander::reload_callback(const std_msgs::Bool::ConstPtr &msg)
{
    reloaded_flag = true;
    reloading_flag = false;
    remaining_bullets = 3;
}

void Throw_Ball_Commander::limitSW_callback(const std_msgs::Float32MultiArray &msg)
{
    if(init_flag)
    {
        if(msg.data[0])
        {
            limit_UD_flag = true;
        }
        if(msg.data[1])
        {
            limit_RL_flag = true;
        }
        if(limit_RL_flag && limit_UD_flag)
        {
            init_flag = false;
            limit_RL_flag = false;
            limit_UD_flag = false;
        }
    } else
    {
        send_emergence_flag = true;
    }
}

void Throw_Ball_Commander::RL_location_callback(const std_msgs::Float32MultiArray &msg)
{
    RL_location = msg.data[0];
}

// others
bool Throw_Ball_Commander::isSubscribed()
{
    auto current_time = std::chrono::system_clock::now();
    const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 current_time - last_sub_vel_time)
                                 .count();

    if (vel_elapsed < lost_time_threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Throw_Ball_Commander::publishMsg()
{
    rogi_link_msgs::RogiLink cmd_msg;

    if (init_flag)
    {
        cmd_msg.id = RRMD << 6 | 0x04;
        *(float *)(&cmd_msg.data[0]) = 0;
        pub_ctrl.publish(cmd_msg);

        cmd_msg.id = LRMD << 6 | 0x04;
        *(float *)(&cmd_msg.data[0]) = 0;
        pub_ctrl.publish(cmd_msg);

        if (!limit_RL_flag)
        {
            // NKRL publish
            cmd_msg.id = NKRL << 6 | 0x03;
            *(float *)(&cmd_msg.data[0]) = cmd_RL;
            pub_ctrl.publish(cmd_msg);
        }else{
            cmd_msg.id = NKRL << 6 | 0x01;
            *(float *)(&cmd_msg.data[0]) = 0;
            pub_ctrl.publish(cmd_msg);
        }
        if(!limit_UD_flag)
        {
            // NKUD publish
            cmd_msg.id = NKUD << 6 | 0x03;
            *(float *)(&cmd_msg.data[0]) = cmd_UD;
            pub_ctrl.publish(cmd_msg);
        }else{
            cmd_msg.id = NKUD << 6 | 0x01;
            *(float *)(&cmd_msg.data[0]) = 0;
            pub_ctrl.publish(cmd_msg);
        }
        if(limit_RL_flag&&limit_UD_flag)
        {
            init_flag = false;
        }
    }
    else
    {
        if(remaining_bullets > 0)
        {
            if (shot_flag && cocked_flag && reloaded_flag)
            {
                cmd_msg.id = MZSV << 6 | 0x08;
                *(float *)(&cmd_msg.data[0]) = 0;
                pub_ctrl.publish(cmd_msg);
                remaining_bullets--;
            }
            cmd_msg.id = RRMD << 6 | 0x04;
            *(float *)(&cmd_msg.data[0]) = 5;
            pub_ctrl.publish(cmd_msg);

            cmd_msg.id = LRMD << 6 | 0x04;
            *(float *)(&cmd_msg.data[0]) = 5;
            pub_ctrl.publish(cmd_msg);

            cmd_msg.id = NKUD << 6 | 0x03;
            *(float *)(&cmd_msg.data[0]) = cmd_UD;
            pub_ctrl.publish(cmd_msg);

            cmd_msg.id = NKRL << 6 | 0x03;
            *(float *)(&cmd_msg.data[0]) = cmd_RL;
            pub_ctrl.publish(cmd_msg);
        }
        else{
            cmd_msg.id = RRMD << 6 | 0x04;
            *(float *)(&cmd_msg.data[0]) = 0;
            pub_ctrl.publish(cmd_msg);

            cmd_msg.id = LRMD << 6 | 0x04;
            *(float *)(&cmd_msg.data[0]) = 0;
            pub_ctrl.publish(cmd_msg);

            cmd_msg.id = NKRL << 6 | 0x03;
            *(float *)(&cmd_msg.data[0]) = 0;
            pub_ctrl.publish(cmd_msg);
            if(RL_location<0.05 && RL_location > -0.05)
            {
                cmd_msg.id = MZDC << 6 | 0x10;
                *(float *)(&cmd_msg.data[0]) = 0;
                pub_ctrl.publish(cmd_msg);
            }
        }
    }
}

void Throw_Ball_Commander::main_cal()
{
    cmd_UD = neck_length * tan(0.5 * asin(target_distance * 9.8 / 900)) / rise_rate;
    cmd_RL = target_theta * roll_rate;
}

void Throw_Ball_Commander::reset()
{
    ROS_ERROR_ONCE("throw_ball_commander: unable to subscribe topics. Reset variables...");
    init_flag = true;
    limit_RL_flag = false;
    limit_UD_flag = false;
}

void Throw_Ball_Commander::update()
{
    ros::Rate r(loop_rate);

    while(ros::ok() && connection_flag == false)
    {
        ROS_WARN_ONCE("waiting for serial connection");
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("serial connected");

    while (ros::ok() && init_flag)
    {
        if(!isSubscribed())
        {
            reset();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }

    while(ros::ok())
    {
        if(isSubscribed())
        {
            main_cal();
        }else{
            reset();
        }
        publishMsg();
        ros::spinOnce();
        r.sleep();
    }
}

// #include "throw_ball_commander.h"

// Throw_Ball_Commander::Throw_Ball_Commander(ros::NodeHandle &_nh, int &_loop_rate, int &_lost_time_threshold,
//                                            float &_neck_length, float &_rise_rate, float &_motor_click, float &_luck_click)
//     : nh(_nh), loop_rate(_loop_rate), lost_time_threshold(_lost_time_threshold),
//       neck_length(_neck_length), rise_rate(_rise_rate), motor_click(_motor_click), luck_click(_luck_click)
// {
//     ROS_INFO("Crating Throw_Ball_commander");
//     ROS_INFO_STREAM("loop_rate: " << loop_rate);
//     ROS_INFO_STREAM("lost_time_threshold: " << lost_time_threshold);
//     ROS_INFO_STREAM("neck_length: " << neck_length);
//     ROS_INFO_STREAM("rise_rate: " << rise_rate);
//     ROS_INFO_STREAM("motor_click: " << motor_click);
//     ROS_INFO_STREAM("luck_click: " << luck_click);

//     sub_target = nh.subscribe("/target", 1, &Throw_Ball_Commander::target_sub_callback, this);
//     sub_shot = nh.subscribe("/shot", 1, &Throw_Ball_Commander::shot_flag_callback, this);
//     sub_emergency_stop = nh.subscribe("/emergency_stop_flag", 1, &Throw_Ball_Commander::emergency_stop_callback, this);
//     last_sub_vel_time = std::chrono::system_clock::now();

//     update();
// }

// // initializers
// void Throw_Ball_Commander::init_drivers()
// {
//     rogi_link_msgs::RogiLink init_msg;

//     init_msg.id = LRMD << 6 | 0x02;
//     init_msg.data[0] = 1;
//     pub_ctrl.publish(init_msg);

//     init_msg.id = RRMD << 6 | 0x02;
//     init_msg.data[0] = 1;
//     pub_ctrl.publish(init_msg);

//     init_msg.id = NKUD << 6 | 0x02;
//     init_msg.data[0] = 1;
//     pub_ctrl.publish(init_msg);

//     init_msg.id = NKRL << 6 | 0x02;
//     init_msg.data[0] = 1;
//     pub_ctrl.publish(init_msg);

//     init_msg.id = MZSV << 6 | INIT_CMD;
//     init_msg.data[0] = 1;
//     pub_ctrl.publish(init_msg);
// }

// void Throw_Ball_Commander::init_variables()
// {
//     target_x = 0;
//     target_y = 0;
//     target_distance = 0;
//     target_theta = 0;

//     roller_duty = 0;
//     neckUD_cmd = 0;
//     neckRL_cmd = 0;

//     emergency_stop_flag = false;
//     connection_flag = false;
//     neckUD_init_flag = true;
//     neckRL_init_flag = true;
//     fire_flag = false;
// }

// // publishers
// void Throw_Ball_Commander::aim_commander()
// {
//     rogi_link_msgs::RogiLink aimer;
//     aimer.id = RRMD << 6 | 0x04;
//     *(float *)(&aimer.data[0]) = roller_duty;
//     pub_ctrl.publish(aimer);

//     aimer.id = LRMD << 6 | 0x04;
//     *(float *)(&aimer.data[0]) = roller_duty;
//     pub_ctrl.publish(aimer);

//     aimer.id = NKUD << 6 | 0x03;
//     *(float *)(&aimer.data[0]) = neckUD_cmd;

//     aimer.id = NKRL << 6 | 0x03;
//     *(float *)(&aimer.data[0]) = neckRL_cmd;
// }

// void Throw_Ball_Commander::shot_commander()
// {
//     rogi_link_msgs::RogiLink shoter;
//     shoter.id = MZSV << 6 | SHOT_CMD;
//     shoter.data[0] = 1;
//     pub_ctrl.publish(shoter);
// }

// // subscribers
// void Throw_Ball_Commander::target_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_target)
// {
//     if (!emergency_stop_flag)
//     {
//         ROS_DEBUG("cmd_target recieved");
//         target_x = cmd_target->linear.x;
//         target_y = cmd_target->linear.y;
//         target_distance = target_x * target_x + target_y * target_y;
//         target_theta = atan(target_y / target_x);

//         last_sub_vel_time = std::chrono::system_clock::now();
//     }
//     else
//     {
//         target_x = 0;
//         target_y = 0;
//         target_distance = 0;
//         target_theta = 0;

//         last_sub_vel_time = std::chrono::system_clock::now();
//         ROS_ERROR_ONCE("throw_ball: emergency stopping...");
//     }
// }

// void Throw_Ball_Commander::shot_flag_callback(const std_msgs::Bool::ConstPtr &msg)
// {
//     fire_flag = msg->data;
// }

// void Throw_Ball_Commander::emergency_stop_callback(const std_msgs::Empty::ConstPtr &msg)
// {
//     emergency_stop_flag = !emergency_stop_flag;
// }

// void Throw_Ball_Commander::ConnectionFlagCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//     connection_flag = msg->data;
// }
// // caliculators
// void Throw_Ball_Commander::converter()
// {
// }

// void Throw_Ball_Commander::cal_aimming()
// {
//     roller_duty = 5;
//     neckUD_cmd = neck_length * tan(0.5 * asin(target_distance * 9.8 / 900)) / rise_rate;
//     neckRL_cmd = target_theta * luck_click / motor_click;
// }

// bool Throw_Ball_Commander::isSubscribed()
// {
//     auto current_time = std::chrono::system_clock::now();
//     const auto vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
//                                  current_time - last_sub_vel_time)
//                                  .count();

//     if (vel_elapsed < lost_time_threshold)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// void Throw_Ball_Commander::reset()
// {
//     ROS_ERROR_ONCE("throw_ball_commander: unable to subscribe topics. Reset variables...");

//     target_x = 0;
//     target_y = 0;
//     target_distance = 0;
//     target_theta = 0;
// }

// void Throw_Ball_Commander::update()
// {
//     ros::Rate r(loop_rate);

//     while (ros::ok() && connection_flag == false)
//     {
//         ROS_WARN_ONCE("waiting for serial connection");
//         ros::spinOnce();
//         r.sleep();
//     }

//     ROS_INFO("serial connected");
//     init_drivers();
//     init_variables();

//     while (ros::ok())
//     {
//         if (isSubscribed())
//         {

//         }
//         else
//         {
//             reset();
//         }
//         aim_commander();
//         if (fire_flag) shot_commander();
//         ros::spinOnce();
//         r.sleep();
//     }
// }
