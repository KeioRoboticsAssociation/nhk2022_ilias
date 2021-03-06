#! /usr/bin/env python3

from queue import Empty
import rospy
from enum import IntEnum

from struct import *
from rogi_link_msgs.msg import RogiLink
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
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
    limit_switch_array = [0] *8
    current_command_position = [0]*2 #1:angle 2:elevation

    def __init__(self):
        self.init_sub = rospy.Subscriber("hard_init", Empty, self.hardinit_callback)
        self.serial_pub = rospy.Publisher("send_serial", RogiLink ,queue_size=100)
        self.limit_pub = rospy.Subscriber("rcv_serial_11", Float32MultiArray, self.limit_switch_callback)
        self.current_sub = rospy.Subscriber("current_command", Float32MultiArray, self.current_sub_callback)
        # self.connection_sub = rospy.Subscriber("connection_status", Bool ,self.connection_sub_callback)
        # self.emergency_stop_pub = rospy.Publisher('/emergency_stop_flag', Empty, queue_size=1)

    def connection_sub_callback(self, msg):
        self.connections_flag = msg.data

    def current_sub_callback(self,msg):
        for i in range(2):
            self.current_command_position[i] = msg.data[i]

    def send_rogilink(self,hardid,commandid,data_0,data_1):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('ff',data_0,data_1)
        self.serial_pub.publish(self.publish_command)

    def send_rogilink_b(self,hardid,commandid,data_0):
        self.publish_command.id=int(hardid) << 6 | commandid
        self.publish_command.data=pack('b',data_0)
        self.serial_pub.publish(self.publish_command)

    def hardware_initialize(self):
        self.send_rogilink_b(HardId.RFMD.value,0x02,1)
        self.send_rogilink_b(HardId.LFMD.value,0x02,1)
        self.send_rogilink_b(HardId.LBMD.value,0x02,1)
        self.send_rogilink_b(HardId.RBMD.value,0x02,1)
        # self.send_rogilinb(HardId.L_BALL,0x02,1)
        # self.send_rogilinb(HardId.R_BALL,0x02,1)
        self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,0)
        self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x02,0)
        # self.send_rogilink(HardId.ELEVATION_ANGLE,0x02,0)
        # self.send_rogilink(HardId.TURN_ANGLE,0x02,0)
        # self.send_rogilink(HardId.BALL_E_MOTOR,0x02,0)
        self.send_rogilink_b(HardId.X_FEINT.value,0x02,0)

        rospy.loginfo("hardware initialization for R2 is complete")


    def limit_switch_callback(self,msg):
        if msg.data[0] == 0:
            if msg.data[1] == 0:
                rospy.logerr("lagori elevator 0 pushed")
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x01,0)
                rospy.sleep(0.1) #???????????????????????????????????????????????????????????????????????????
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,3)
                self.send_rogilink(HardId.LAGORI_E_MOTOR.value,0x06,0.1,0)
                #?????????????????????????????????????????????
                rospy.sleep(0.5)
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x01,0)
                rospy.sleep(0.2)
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,0)
                # self.send_rogilink(HardId.LAGORI_E_MOTOR.value,0x09,0,0)
            elif msg.data[1] == 1:
                rospy.logerr("lagori elevator high pushed")
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x01,0)
                rospy.sleep(0.1)
                self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,0)
                self.send_rogilink(HardId.LAGORI_E_MOTOR.value,0x09,self.current_command_position[0],0)
            elif msg.data[1] == 2:
                rospy.logerr("lagori hand 0 pushed")
                self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x01,0)
                rospy.sleep(0.1)
                self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x02,0)
                self.send_rogilink(HardId.LAGORI_G_MOTOR.value,0x09,0,0)
            elif msg.data[1] == 3:
                rospy.logerr("lagori hand high pushed")
                self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x01,0)
                rospy.sleep(0.1)
                self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x02,0)
                self.send_rogilink(HardId.LAGORI_G_MOTOR.value,0x09,self.current_command_position[1],0)

            elif msg.data[1] == 4:
                rospy.logerr("lagori hand high pushed")
                self.send_rogilink_b(HardId.X_FEINT.value,0x01,0)
                rospy.sleep(0.1)
                self.send_rogilink_b(HardId.X_FEINT.value,0x02,0)
                self.send_rogilink(HardId.X_FEINT.value,0x09,0,0)

            else:
                rospy.logerr("unknown limit switch pushed")
        else:
            for i in range(8):
                self.limit_switch_array[i] = 0x00000001 & int(msg.data[1]) >> i

    def hardinit_callback(self, msg):

        rospy.logwarn("hard init command")
        rospy.loginfo("%d,%d,%d,%d,%d,%d,%d,%d",self.limit_switch_array[0],self.limit_switch_array[1],self.limit_switch_array[2],self.limit_switch_array[3],self.limit_switch_array[4],self.limit_switch_array[5],self.limit_switch_array[6],self.limit_switch_array[7])

        self.send_rogilink_b(HardId.RFMD.value,0x01,0)
        self.send_rogilink_b(HardId.LFMD.value,0x01,0)
        self.send_rogilink_b(HardId.LBMD.value,0x01,0)
        self.send_rogilink_b(HardId.RBMD.value,0x01,0)
        rospy.sleep(0.1)
        self.send_rogilink_b(HardId.RFMD.value,0x02,1)
        self.send_rogilink_b(HardId.LFMD.value,0x02,1)
        self.send_rogilink_b(HardId.LBMD.value,0x02,1)
        self.send_rogilink_b(HardId.RBMD.value,0x02,1)

        if self.limit_switch_array[0]==1 and self.limit_switch_array[1]==1:
            rospy.logwarn("elevation")
            self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x01,0)
            rospy.sleep(0.1) #???????????????????????????????????????????????????????????????????????????
            self.send_rogilink_b(HardId.LAGORI_E_MOTOR.value,0x02,3)
            self.send_rogilink(HardId.LAGORI_E_MOTOR.value,0x06,-0.3,0)

        if self.limit_switch_array[2]==1:
            self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x01,0)
            rospy.sleep(0.1) #???????????????????????????????????????????????????????????????????????????
            self.send_rogilink_b(HardId.LAGORI_G_MOTOR.value,0x02,3)
            self.send_rogilink(HardId.LAGORI_G_MOTOR.value,0x06,0.3,0)

        if self.limit_switch_array[4]==1:
            self.send_rogilink_b(HardId.X_FEINT.value,0x01,0)
            rospy.sleep(0.1)
            self.send_rogilink_b(HardId.X_FEINT.value,0x02,3)
            self.send_rogilink(HardId.X_FEINT.value,0x06,0.1,0)




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

        while not rospy.is_shutdown():
            Rosconnector.hardware_initialize()

            rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
