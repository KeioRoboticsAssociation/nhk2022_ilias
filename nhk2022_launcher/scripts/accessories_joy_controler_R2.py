#! /usr/bin/env python3

from asyncio import current_task
import rospy
from enum import IntEnum
from math import pi

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from struct import *
from sensor_msgs.msg import Joy
from rogi_link_msgs.msg import RogiLink
from std_msgs.msg import Float32,UInt8MultiArray,UInt8
# from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


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


FLAT_POSITION = 0.4

class Rosconnector():

    publish_command = RogiLink()
    coordinate_angle = Float32()
    ball_catcher_hight: bool = False
    ball_catcher_grab: bool = False
    lagori_catcher_angle_flag: bool = False
    elevator_flag: bool = False
    elevator_position: float = 0
    grab_position: float = 0
    prev_msg = Joy()
    pile_status = [0]*5
    lagori_number = 0
    catch_flag=0
    pile_send = [0]*2 #0:hight 1:grab

    def __init__(self):
        self.joy_sub = rospy.Subscriber("joy", Joy, self.Joycallback)
        self.serial_pub = rospy.Publisher(
            "send_serial", RogiLink, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher(
            "emergency_stop_flag", Empty, queue_size=1)
        self.joy_angle_sub = rospy.Publisher(
            "joy_angle", Float32, queue_size=1)
        self.pile_status_sub = rospy.Subscriber(
            "pile_status", UInt8MultiArray, self.pile_status_callback)
        self.lagori_number_sub = rospy.Subscriber(
            "lagori_number", UInt8, self.lagori_number_callback)
        self.current_pub = rospy.Publisher("current_command", Float32MultiArray, queue_size=1)
        self.hardinit_flag_pub = rospy.Publisher("hard_init",Empty,queue_size=1)

    def send_rogilink(self, hardid, commandid, data_0, data_1):
        self.publish_command.id = int(hardid) << 6 | commandid
        self.publish_command.data = pack('ff', data_0, data_1)
        self.serial_pub.publish(self.publish_command)

    def send_rogilink_servo(self, hardid, commandid, data_0, data_1, data_2):
        self.publish_command.id = int(hardid) << 6 | commandid
        self.publish_command.data = pack('bbbbb', 0, 0, data_0, data_1, data_2)
        self.serial_pub.publish(self.publish_command)

    def pile_status_callback(self, msg):
        self.pile_status = msg.data

    def lagori_number_callback(self, msg):
        rospy.loginfo("lagori number set %d",msg.data)
        self.lagori_number = msg.data
        # rospy.loginfo("lagori number set %d",self.lagori_number)
        # if self.lagori_number == 1:
        #     self.rogi_sender(4.5,-8)
        # elif self.lagori_number == 2:
        #     self.rogi_sender(6.2,-8.1)
        # elif self.lagori_number == 3:
        #     self.rogi_sender(7.4,-9.4)
        # elif self.lagori_number == 4:
        #     self.rogi_sender(8.8,-10)
        # elif self.lagori_number == 5:
        #     self.rogi_sender(12,-11)
        # else:
        #     self.rogi_sender(0,0)

    def pile_commander(self, msg):
        rospy.loginfo("pile")
        if msg=="catch":
            self.catch_flag = not self.catch_flag

            if self.catch_flag==0:#closed
                if self.lagori_number==1:
                    self.pile_send[1]=-3.1

                elif self.lagori_number==2:
                    self.pile_send[1]=-4.65

                elif self.lagori_number==3:
                    self.pile_send[1]=-6.1

                elif self.lagori_number==4:
                    self.pile_send[1]=-7.8

                elif self.lagori_number==5:
                    self.pile_send[1]=-9.36

                else:
                    self.pile_send[1]=0

            else:
                if self.lagori_number==1:
                    self.pile_send[1]=-8

                elif self.lagori_number==2:
                    self.pile_send[1]=-8.2

                elif self.lagori_number==3:
                    self.pile_send[1]=-9.4

                elif self.lagori_number==4:
                    self.pile_send[1]=-10

                elif self.lagori_number==5:
                    self.pile_send[1]=-11

                else:
                    self.pile_send[1]=0

        elif msg=="low":
            if self.lagori_number==1:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=0

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==2:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=0

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==3:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=0

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==4:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=0.74

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=1

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==5:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=0

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=1.76

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=1.52

                else:
                    self.pile_send[0]=0

            else:
                self.pile_send[0]=0

        elif msg=="high":
            if self.lagori_number==1:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=5.06

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=5.06

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=5.06

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==2:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=17.54

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=17.54

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=17.54

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==3:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=13.46

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=13.46

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=13.46

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==4:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=9.72

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=9.72

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=9.72

                else:
                    self.pile_send[0]=0

            elif self.lagori_number==5:
                if self.pile_status[self.lagori_number-1]==0:#flat
                    self.pile_send[0]=5.06

                elif self.pile_status[self.lagori_number-1]==1:#angle
                    self.pile_send[0]=5.06

                elif self.pile_status[self.lagori_number-1]==2:#perpendicular
                    self.pile_send[0]=5.06

                else:
                    self.pile_send[0]=0

            else:
                self.pile_send[0]=0

        rospy.loginfo("command sent%f %f",self.pile_send[0],self.pile_send[1])
        self.rogi_sender(self.pile_send[0],self.pile_send[1])

        array=[]
        for p in self.pile_send:
            array.append(p)
        publish_buffer = Float32MultiArray(data=array)
        self.current_pub.publish(publish_buffer)


    def rogi_sender(self, elevator_command, grab_command):
        self.elevator_position = elevator_command
        self.grab_position = grab_command
        self.send_rogilink(HardId.LAGORI_E_MOTOR.value,
                            0x03, self.elevator_position, 0)
        self.send_rogilink(HardId.LAGORI_G_MOTOR.value,
                            0x03, self.grab_position, 0)

    def Joycallback(self, msg):
        if msg.buttons != self.prev_msg.buttons:
            self.prev_msg = msg  # saving prev message

            if msg.buttons[0]:  # X
                self.ball_catcher_hight = not self.ball_catcher_hight
                if self.ball_catcher_hight:
                    self.send_rogilink(HardId.AIR.value, 0x01, 0, 0)
                else:
                    self.send_rogilink(HardId.AIR, 0x02, 0, 0)
                rospy.loginfo("ball catcher hight changed")

            if msg.buttons[1]:  # O
                self.ball_catcher_grab = not self.ball_catcher_grab
                if self.ball_catcher_grab:
                    self.send_rogilink(HardId.AIR.value, 0x03, 0, 0)
                else:
                    self.send_rogilink(HardId.AIR, 0x04, 0, 0)
                rospy.loginfo("ball catcher grab changed")

            if msg.buttons[2]:  # <|
                self.lagori_catcher_angle_flag = not self.lagori_catcher_angle_flag
                if self.lagori_catcher_angle_flag:
                    self.send_rogilink_servo(
                        HardId.LAGORI_SERVO.value, 0x03, 0, 0, 0)
                else:
                    self.send_rogilink_servo(
                        HardId.LAGORI_SERVO.value, 0x03, 0, 9, 0)
                rospy.loginfo("lagori catcher angle changed")

            if msg.buttons[3]:  # <>
                # self.lagori_gripper_catch_flag = not self.lagori_gripper_catch_flag
                # self.accessories_pub_commands.data = [float(3), float(self.lagori_gripper_catch_flag)]
                # self.controler_id=3
                rospy.loginfo("lagori catcher changed")

            if msg.buttons[4]:  # L1
                # self.pile_commander("high")
                rospy.loginfo("lagori high")

            if msg.buttons[5]:  # R1
                self.pile_commander("high")
                rospy.loginfo("lagori high")

            if msg.buttons[6]:  # L2
                self.pile_commander("catch")
                rospy.loginfo("lagori catch")

            if msg.buttons[7]:  # R2
                self.pile_commander("low")
                rospy.loginfo("lagori low")

            if msg.buttons[8]:  # Share back
                emergency_msg = Empty()
                self.emergency_stop_pub.publish(emergency_msg)
                rospy.logwarn("EMERGENCY STOP")

            if msg.buttons[9]:  # Options
                # rospy.loginfo("teleop_mode")
                # # self.client.cancel_goal()
                # teleop_mode = Bool()
                # teleop_mode.data = False
                # self.teleopflag_pub.publish(teleop_mode)
                self.hardinit_flag_pub.publish()
                rospy.logwarn("hard init")

            if msg.buttons[10]:#PS
                self.coordinate_angle.data = pi
                self.joy_angle_sub.publish(self.coordinate_angle)
                rospy.loginfo("coordinate angle pi")

            if msg.buttons[11]:  # Leftpush
                self.coordinate_angle.data = 0
                self.joy_angle_sub.publish(self.coordinate_angle)
                rospy.loginfo("coordinate angle 0")

            if msg.buttons[12]:  # Rightpush
                self.coordinate_angle.data = pi/4
                self.joy_angle_sub.publish(self.coordinate_angle)
                rospy.loginfo("coordinate angle pi/4")

        if msg.axes[7]:
            if(self.elevator_position >= 0):
                self.elevator_position = self.elevator_position + msg.axes[7] / 50
            else:
                self.elevator_position = 0
                rospy.loginfo("elevator too low")
            self.send_rogilink(HardId.LAGORI_E_MOTOR.value,0x03, self.elevator_position, 0)
            rospy.loginfo("move elevator %f",self.elevator_position)

        if msg.axes[6]:
            if(self.grab_position <= 0):
                self.grab_position = self.grab_position - msg.axes[6] / 50
            else:
                self.grab_position = 0
                rospy.loginfo("grabing too much")
            self.send_rogilink(HardId.LAGORI_G_MOTOR.value,0x03, self.grab_position, 0)
            rospy.loginfo("move gripper %f",self.grab_position)

        list=[]
        for p in range(2):
            list.append(p)
        list = [self.elevator_position,self.grab_position]
        buf_pub = Float32MultiArray(data=list)
        self.current_pub.publish(buf_pub)



if __name__ == '__main__':
    try:
        rospy.init_node('accessories_controler')
        rospy.loginfo("create accessories controler")

        Rosconnector = Rosconnector()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
