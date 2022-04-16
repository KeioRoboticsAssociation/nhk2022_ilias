#! /usr/bin/env python3

# from click import command
from enum import Enum
from operator import mod
import rospy

from sensor_msgs.msg import Joy
from rogi_link_msgs.msg import RogiLink
# from std_msgs.msg import Bool
# from std_msgs.msg import Float32MultiArray

class HardId(Enum):
    EMGC_STOP = 0x00
    MAIN_BOARD = 0x01
    RFMD = 0x02
    LFMD = 0x03
    LBMD = 0x04
    RBMD = 0x05
    L_LAGORI = 0x06
    R_LAGORI = 0x07



class Rosconnector():

    publish_command = RogiLink()

    def generator(self,hardid,commandid,data):
        self.publish_command.id=hardid << 6 | commandid
        self.publish_command.data=data

    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.serial_pub= rospy.Publisher("sending_data", RogiLink ,queue_size=1)

    def generator(self,hardid,commandid,data):
        self.publish_command.id=hardid << 6 | commandid
        self.publish_command.data=data

    def Joycallback(self, msg):
        if msg.buttons[0]:#X
            self.generator()
            # self.ball_catcher_vertical_flag= not self.ball_catcher_vertical_flag
            # self.controler_id=1
            # self.accessories_pub_commands.data = [float(1), float(self.ball_catcher_vertical_flag)]
            # self.accessories_controler_pub.publish(self.accessories_pub_commands)

            rospy.loginfo("ball catcher hight changed")

        elif msg.buttons[1]:#O
            # self.ball_catcher_catch_flag= not self.ball_catcher_catch_flag
            # self.accessories_pub_commands.data = [float(2), float(self.ball_catcher_catch_flag)]
            # self.controler_id=2
            rospy.loginfo("ball catcher clip changed")


        elif msg.buttons[2]:#<|
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[3]:#<>
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[4]:#L1
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[5]:#R1
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[6]:#L2
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[7]:#R2
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[8]:#Share
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[9]:#Options
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[10]:#PS
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[11]:#Leftpush
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        elif msg.buttons[12]:#Rightpush
            # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            # self.controler_id=3
            rospy.loginfo("lagori catcher changed")

        # elif self.elevator_flag!=msg.axes[5]:
        #     # self.controler_id=4
        #     # self.elevator_flag=msg.axes[5]
        #     # self.accessories_pub_commands.data = [float(4), float(self.elevator_flag)]
        #     rospy.loginfo("lagori catcher changed")

        # if self.table_flag!=msg.axes[4]:
            # self.table_flag=msg.axes[4]

        # rospy.loginfo("elevator moving")


        # self.accessories_pub_commands.data = [float(self.ball_catcher_catch_flag), float(self.ball_catcher_vertical_flag)]
        # self.elevator_pub_commands.data = [float(self.elevator_flag),250]
        # self.table_pub_commands.data = [float(self.table_flag),0]
        # self.accessories_controler_pub.publish(self.accessories_pub_commands)
        # self.elevator_controler_pub.publish(self.elevator_pub_commands)
        # self.table_controler_pub.publish(self.table_pub_commands)


if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        Rosconnector = Rosconnector()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
