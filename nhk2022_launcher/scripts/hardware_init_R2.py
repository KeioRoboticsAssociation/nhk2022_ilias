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
        self.serial_pub= rospy.Publisher("send_serial", RogiLink ,queue_size=1)
        self.hardware_initialize()
        # self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)

    def send_rogilink(self,hardid,commandid,data_0,data_1):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('ff',data_0,data_1)
        self.serial_pub.publish(self.publish_command)

    def send_rogilink(self,hardid,commandid,data_0):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('c',data_0)
        self.serial_pub.publish(self.publish_command)

    def hardware_initialize(self):
        self.send_rogilink(HardId.RFMD,0x02,1)
        self.send_rogilink(HardId.LFMD,0x02,1)
        self.send_rogilink(HardId.LBMD,0x02,1)
        self.send_rogilink(HardId.RBMD,0x02,1)
        # self.send_rogilink(HardId.L_BALL,0x02,1)
        # self.send_rogilink(HardId.R_BALL,0x02,1)
        self.send_rogilink(HardId.LAGORI_E_MOTOR,0x02,0)
        self.send_rogilink(HardId.LAGORI_G_MOTOR,0x02,0)
        self.send_rogilink(HardId.ELEVATION_ANGLE,0x02,0)
        self.send_rogilink(HardId.TURN_ANGLE,0x02,0)
        self.send_rogilink(HardId.BALL_E_MOTOR,0x02,0)

        rospy.loginfo("hardware initialization for R2 is complete")




if __name__ == '__main__':
    try:
        rospy.init_node('hardware_init_R2')
        rospy.loginfo("initializing hardware")

        Rosconnector = Rosconnector()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
