#! /usr/bin/env python3

# from click import command
import rospy

from sensor_msgs.msg import Joy
# from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class Accessories_Controller():
    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.accessories_controler_pub= rospy.Publisher('/accessories_controler_commands', Float32MultiArray ,queue_size=1)
        self.ball_catcher_vertical_flag=False
        self.ball_catcher_catch_flag=False
        self.pub_commands= Float32MultiArray()

    def Joycallback(self, msg):
        if msg.buttons[6]:#LT
            self.ball_catcher_vertical_flag= not self.ball_catcher_vertical_flag
            rospy.loginfo("ball catcher hight changed")
        
        elif msg.buttons[2]:#B
            self.ball_catcher_catch_flag= not self.ball_catcher_catch_flag
            rospy.loginfo("ball catcher clip changed")
        
        self.pub_commands.data = [float(self.ball_catcher_catch_flag), float(self.ball_catcher_vertical_flag)]
        self.accessories_controler_pub.publish(self.pub_commands)


if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        accessories_controller = Accessories_Controller()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
