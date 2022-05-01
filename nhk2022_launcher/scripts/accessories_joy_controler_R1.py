#! /usr/bin/env python3

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
    L_BALL = 0x06
    R_BALL = 0x07
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
    elevator_flag : bool = False
    elevator_position : float = 0
    ball_elevator_position : float = 0
    angle_position : float = 0
    prev_msg = Joy()
    roller_speed : float = 0

    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.serial_pub= rospy.Publisher("send_serial", RogiLink ,queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)

    def send_rogilink(self,hardid,commandid,data_0,data_1):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('ff',data_0,data_1)
        self.serial_pub.publish(self.publish_command)

    def Joycallback(self, msg):
        if msg.buttons != self.prev_msg.buttons:
            self.prev_msg = msg #saving prev message

            if msg.buttons[0]:#X
                self.ball_catcher_hight:self.send_rogilink(HardId.SHOT_SERVO,0x04,0,0)
                rospy.loginfo("reload")

            if msg.buttons[1]:#O
                self.ball_catcher_grab:self.send_rogilink(HardId.SHOT_SERVO,0x08,0,0)
                rospy.loginfo("shoot")


            if msg.buttons[2]:#<|
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            if msg.buttons[3]:#<>
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")


            if msg.buttons[5]:#R1
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")



            if msg.buttons[7]:#R2
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            if msg.buttons[8]:#Share back
                emergency_msg = Empty()
                self.emergency_stop_pub.publish(emergency_msg)
                rospy.logwarn("EMERGENCY STOP")

            if msg.buttons[9]:#Options
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            # if msg.buttons[10]:#PS
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            # if msg.buttons[11]:#Leftpush
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            # if msg.buttons[12]:#Rightpush
            #     # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
            #     # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
            #     # self.controler_id=3
                rospy.loginfo("lagori catcher changed")


            if msg.buttons[5]:#L1
                self.send_rogilink(HardId.BALL_E_MOTOR.value,0x03,-13.5,0)

                rospy.loginfo("move ball elevator")

            if msg.buttons[7]:#L2
                self.send_rogilink(HardId.BALL_E_MOTOR.value,0x03,0,0)

                rospy.loginfo("move ball elevator")


        if msg.axes[5]:
            if(self.elevator_position>=0):
                self.elevator_position = self.elevator_position + msg.axes[5] / 100
                self.send_rogilink(HardId.ELEVATION_ANGLE.value,0x03,self.angle_position,self.elevator_position)
            else:
                self.elevator_position = 0
                rospy.loginfo("elevating too much")

            rospy.loginfo("move elevation angle")


        if msg.axes[4]:
            if(self.angle_position>=0):
                self.angle_position = self.angle_position - msg.axes[4] / 100
                self.send_rogilink(HardId.TURNE_ANGLE.value,0x03,self.angle_position,self.elevator_position)
            else:
                self.angle_position = 0
                rospy.loginfo("turning too much")


            rospy.loginfo("move turn table angle")
            rospy.loginfo("%f",self.angle_position)


        if msg.axes[1]:
            self.roller_speed = msg.axes[1]*100
            self.send_rogilink(HardId.R_BALL.value,0x05,self.roller_speed,-self.roller_speed)
            self.send_rogilink(HardId.L_BALL.value,0x05,self.roller_speed,-self.roller_speed)

        #     rospy.loginfo("move elevation angle")



if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        Rosconnector = Rosconnector()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")