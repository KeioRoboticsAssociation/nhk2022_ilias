#! /usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class Accessories_Controller():
    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.ball_catcher_vertical= rospy.Publisher('/ball_catcher_vertical', Bool ,queue_size=1)
        self.ball_catcher_catch=rospy.Publisher('/ball_catcher', Bool ,queue_size=1)
        self.ball_catcher_vertical_flag=False
        self.ball_catcher_catch_flag=False

    def Joycallback(self, msg):
        if msg.buttons[6]:#LT
            self.ball_catcher_vertical_flag=~self.ball_catcher_vertical_flag
            self.ball_catcher_vertical.publish(self.ball_catcher_vertical_flag)
        
        elif msg.buttons[2]:#B
            self.ball_catcher_catch_flag=~self.ball_catcher_catch_flag
            self.ball_catcher_catch.publish(self.ball_catcher_catch_flag)


if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        accessories_controller = Accessories_Controller()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
