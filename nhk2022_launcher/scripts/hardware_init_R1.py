#! /usr/bin/env python3

# from click import command
from asyncio import wait_for
from numpy import std
import rospy
from enum import IntEnum

from std_msgs.msg import Empty
from struct import *
from sensor_msgs.msg import Joy
from rogi_link_msgs.msg import RogiLink
from std_msgs.msg import Bool
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
    TURN_ANGLE = 0x10
    SENSOR = 0x11
    BALL_E_MOTOR = 0x12


class Rosconnector():

    publish_command = RogiLink()

    def __init__(self):
        self.serial_pub= rospy.Publisher("send_serial", RogiLink ,queue_size=100)
        # self.connection_sub = rospy.Subscriber("connection_status", Bool ,self.connection_sub_callback)
        # self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)

    def connection_sub_callback(self, msg):
        self.connections_flag = msg.data

    def send_rogilink(self,hardid,commandid,data_0,data_1):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('ff',data_0,data_1)
        self.serial_pub.publish(self.publish_command)

    def send_rogilink_b(self,hardid,commandid,data_0):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('b',data_0)
        self.serial_pub.publish(self.publish_command)

    def hardware_initialize(self):
        # self.send_rogilink_b(HardId.RFMD.value,0x02,1)
        # self.send_rogilink_b(HardId.LFMD.value,0x02,1)
        # self.send_rogilink_b(HardId.LBMD.value,0x02,1)
        # self.send_rogilink_b(HardId.RBMD.value,0x02,1)
        # self.send_rogilinb(HardId.L_BALL,0x02,1)
        # self.send_rogilinb(HardId.R_BALL,0x02,1)
        self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,0)
        self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x02,0)
        # self.send_rogilink(HardId.ELEVATION_ANGLE,0x02,0)
        # self.send_rogilink(HardId.TURN_ANGLE,0x02,0)
        # self.send_rogilink(HardId.BALL_E_MOTOR,0x02,0)

        rospy.loginfo("hardware initialization for R2 is complete")




if __name__ == '__main__':
    try:
        rospy.init_node("hardware_init_R2")
        rospy.loginfo("initializing hardware")
        Rosconnector = Rosconnector()

        r = rospy.Rate(1) # 10hz

        while not rospy.is_shutdown():
            connection_status = rospy.wait_for_message("connection_status", Bool)

            if connection_status.data == True:
                rospy.loginfo("serial connected")
                break

            rospy.loginfo_once("initialization waiting for serial connection")
            r.sleep()

        Rosconnector.hardware_initialize()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
