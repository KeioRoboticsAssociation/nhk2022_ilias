#! /usr/bin/env python3

# from click import command
import rospy
from enum import IntEnum

from std_msgs.msg import Empty
from struct import *
from sensor_msgs.msg import Joy
from rogi_link_msgs.msg import RogiLink
# from std_msgs.msg import Bool
# from std_msgs.msg import Float32MultiArray

class HardId(IntEnum):
    EMGC_STOP = 0x00
    MAIN_BOARD = 0x01
    RFMD = 0x02
    LFMD = 0x03
    LBMD = 0x04
    RBMD = 0x05
    L_LAGORI = 0x06
    R_LAGORI = 0x07
    LAGORI_E_MOTOR = 0x08
    AIR = 0x09
    LAGORI_SERVO = 0x0A
    LAGORI_G_MOTOR = 0x0B
    X_FEINT = 0x0C
    Y_FEINT = 0x0D
    SHOT_SERVO = 0x0E
    ELEVATION_ANGLE = 0x0F
    TURNE_ANGLE = 0x10
    SENSOR = 0x11
    BALL_E_MOTOR = 0x12


class Rosconnector():

    publish_command = RogiLink()
    ball_catcher_hight : bool = False
    ball_catcher_grab : bool = False
    elevator_position : float = 0
    grab_position : float = 0

    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.serial_pub= rospy.Publisher("send_serial", RogiLink ,queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)

    def send_rogilink(self,hardid,commandid,data_0,data_1):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('ff',data_0,data_1)
        self.serial_pub.publish(self.publish_command)

    def Joycallback(self, msg):
        if msg.buttons[0]:#X
            self.ball_catcher_hight= not self.ball_catcher_hight
            if self.ball_catcher_hight:self.send_rogilink(HardId.AIR,0x01,0,0)
            else: self.send_rogilink(HardId.AIR,0x02,0,0)
            rospy.loginfo("ball catcher hight changed")

        elif msg.buttons[1]:#O
            self.ball_catcher_grab= not self.ball_catcher_grab
            if self.ball_catcher_grab:self.send_rogilink(HardId.AIR,0x03,0,0)
            else: self.send_rogilink(HardId.AIR,0x04,0,0)
            rospy.loginfo("ball catcher grab changed")


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

        elif msg.buttons[8]:#Share back
            emergency_msg = Empty()
            self.emergency_stop_pub.publish(emergency_msg)
            rospy.logwarn("EMERGENCY STOP")

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

        # elif msg.buttons[12]:#Rightpush
        #     # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
        #     # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
        #     # self.controler_id=3
        #     rospy.loginfo("lagori catcher changed")

        elif self.elevator_flag!=msg.axes[5]:
            if(self.elevator_position>0):
                self.elevator_position = self.elevator_position + msg.axes[5]
                self.send_rogilink(HardId.LAGORI_E_MOTOR,0x03,self.elevator_position)
            else: rospy.loginfo("elevator too low")
        #     # self.controler_id=4
        #     # self.elevator_flag=msg.axes[5]
        #     # self.accessories_pub_commands.data = [float(4), float(self.elevator_flag)]
            rospy.loginfo("move elevator")

        if self.table_flag!=msg.axes[4]:
            if(self.grab_position>0):
                self.grab_position = self.grab_position + msg.axes[5]
                self.send_rogilink(HardId.LAGORI_G_MOTOR,0x03,self.grab_position)
            else: rospy.loginfo("grabing too much")

            rospy.loginfo("move gripper")


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
