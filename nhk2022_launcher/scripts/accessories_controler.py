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
        # self.elevator_controler_pub= rospy.Publisher('/elevator_controler_commands', Float32MultiArray ,queue_size=1)
        # self.table_controler_pub= rospy.Publisher('/table_controler_commands', Float32MultiArray ,queue_size=1)
        self.ball_catcher_vertical_flag=False
        self.ball_catcher_catch_flag=False
        self.lagori_gripper_catch_flag=False
        self.elevator_flag=0
        self.controler_id=0
        # self.table_flag=0
        self.accessories_pub_commands= Float32MultiArray()
        # self.elevator_pub_commands= Float32MultiArray()
        # self.table_pub_commands= Float32MultiArray()

    def Joycallback(self, msg):
        if msg.buttons[6]:#LT
            self.ball_catcher_vertical_flag= not self.ball_catcher_vertical_flag
            # self.controler_id=1
            self.accessories_pub_commands.data = [float(1), float(self.ball_catcher_vertical_flag)]
            self.accessories_controler_pub.publish(self.accessories_pub_commands)
            rospy.loginfo("ball catcher hight changed")
        
        elif msg.buttons[2]:#B
            self.ball_catcher_catch_flag= not self.ball_catcher_catch_flag
            self.accessories_pub_commands.data = [float(2), float(self.ball_catcher_catch_flag)]
            # self.controler_id=2
            rospy.loginfo("ball catcher clip changed")

        elif msg.buttons[1]:#A
            self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")
        
        elif self.elevator_flag!=msg.axes[5]:
            # self.controler_id=4
            self.elevator_flag=msg.axes[5]
            self.accessories_pub_commands.data = [float(4), float(self.elevator_flag)]


        # if self.table_flag!=msg.axes[4]:
            # self.table_flag=msg.axes[4]

        # rospy.loginfo("elevator moving")


        # self.accessories_pub_commands.data = [float(self.ball_catcher_catch_flag), float(self.ball_catcher_vertical_flag)]
        # self.elevator_pub_commands.data = [float(self.elevator_flag),250]
        # self.table_pub_commands.data = [float(self.table_flag),0]
        self.accessories_controler_pub.publish(self.accessories_pub_commands)
        # self.elevator_controler_pub.publish(self.elevator_pub_commands)
        # self.table_controler_pub.publish(self.table_pub_commands)


if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        accessories_controller = Accessories_Controller()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")