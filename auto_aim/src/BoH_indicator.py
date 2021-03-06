#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class BohIndicator:
    def __init__(self,loop_rate,max_range,limit_angle):
        # init handles

        rospy.logwarn([loop_rate, max_range, limit_angle])
        rospy.logwarn("enter init")
        rospy.init_node("BoH_Indicator")
        self.Boh_pub = rospy.Publisher("/BoH_location", Float32MultiArray, queue_size=100)
        rospy.Subscriber("/scan",LaserScan,self.LiDARCallback,queue_size=100)
        # rospy.logwarn("nya")

        # params
        self.myrate = rospy.Rate(loop_rate)
        # rospy.logwarn("bya")
        self.MAX_RANGE = max_range
        # rospy.logwarn("bya")
        self.LIMIT_ANGLE = limit_angle
        # rospy.logwarn("nya2")

        # variables
        self.isCaptured = False
        self.x = list()
        self.y = list()
        self.past_x = 100
        self.past_y = 100
        self.pp_x = 0
        self.pp_y = 0
        # rospy.logwarn("nya3")

        # methods
        self.update()

    def circleFitting(self,x,y):
        sumx = sum(x)
        sumy = sum(y)
        if not (sumx == 0 or sumy == 0):
            # rospy.logwarn("enter fitting")
            sumx2 = sum([ix ** 2 for ix in x])
            sumy2 = sum([iy ** 2 for iy in y])
            sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])
            # rospy.logwarn("pipi")
            F = np.array([[sumx2, sumxy, sumx],
                            [sumxy, sumy2, sumy],
                            [sumx, sumy, len(x)]])
            # rospy.logwarn("pipi")

            G = np.array([[-sum([ix ** 3 + ix*iy ** 2 for (ix, iy) in zip(x, y)])],
                            [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                            [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])
            # rospy.logwarn("pipi")
            if np.linalg.det(F) == 0 :
                # rospy.logwarn("nyuuu")
                return (0,0)
            else:
                I = np.linalg.inv(F)
                # rospy.logwarn("nya")
                T = np.dot(I,G)
                # rospy.logwarn("pipi")

                cxe = float(T[0]/-2)
                # rospy.logwarn("nya")
                cye = float(T[1]/-2)
                # rospy.logwarn("nya")
                # rospy.logwarn("pipi")
                # if cxe**2+cye**2-T[2] >= 0 :
                # re = math.sqrt(cxe**2+cye**2-T[2])
                # rospy.logwarn("nya")
                # rospy.logwarn("pipi")

                return (cxe, cye)
        else :
            return (0,0)


    def LiDARCallback(self,msg):
        self.x.clear()
        self.y.clear()
        group_A_x = list()
        group_A_y = list()
        group_A_r = list()
        group_B_x = list()
        group_B_y = list()
        group_B_r = list()
        rospy.logwarn("rcv msg")
        for num in range(len(msg.ranges)) :
            theta = msg.angle_min + num*msg.angle_increment
            r = msg.ranges[num]
            if r > 0.2 and r < self.MAX_RANGE and theta > -1 * self.LIMIT_ANGLE and theta < self.LIMIT_ANGLE:
                if not self.isCaptured:
                    group_A_x.append(r * math.cos(theta))
                    group_A_y.append(r * math.sin(theta))
                    group_A_r.append(r)
                else:
                    if abs(np.mean(group_A_r)-r) < 0.2:
                        group_A_x.append(r * math.cos(theta))
                        group_A_y.append(r * math.sin(theta))
                        group_A_r.append(r)
                    elif not group_B_x:
                        group_B_x.append(r * math.cos(theta))
                        group_B_y.append(r * math.sin(theta))
                        group_B_r.append(r)
                    else:
                        if abs(np.mean(group_A_r)-r) < abs(np.mean(group_B_r)-r):
                            group_A_x.append(r * math.cos(theta))
                            group_A_y.append(r * math.sin(theta))
                            group_A_r.append(r)
                        else :
                            group_B_x.append(r * math.cos(theta))
                            group_B_y.append(r * math.sin(theta))
                            group_B_r.append(r)

                self.isCaptured = True
        if len(group_A_r)>len(group_B_r):
            self.x = group_A_x
            self.y = group_A_y
        else :
            self.x = group_B_x
            self.y = group_B_y


    def sendMsg(self,pub_x,pub_y):
        array=[]

        if self.past_x == 100 or self.past_y == 100:
            self.past_x = pub_x
            self.past_y = pub_y
            self.pp_x = pub_x
            self.pp_y = pub_y
        elif abs(pub_x - self.past_x)>1 or abs(pub_y - self.past_y) > 1:
            if abs(self.pp_x - pub_x) < 0.5 and abs(self.pp_y - pub_y) < 0.5 :
                self.past_x = pub_x
                self.past_y = pub_y
                self.pp_x = pub_x
                self.pp_y = pub_y
            else:
                self.pp_x = pub_x
                self.pp_y = pub_y
                pub_x = self.past_x
                pub_y = self.past_y
        else :
            self.past_x = pub_x
            self.past_y = pub_y
            self.pp_x = pub_x
            self.pp_y = pub_y

        # rospy.logwarn("gya")
        if pub_x == 0:
            # rospy.logwarn("gya0")
            array.append(0)
            array.append(0)
        else :
            # rospy.logwarn("gya1")
            array.append(pub_x + 0.260)
            # rospy.logwarn("gyagya")
            array.append(pub_y + 0.3405)

        my_msg = Float32MultiArray(data=array)

        # rospy.logwarn("gya1")
        self.Boh_pub.publish(my_msg)
        rospy.loginfo(my_msg.data)

    def update(self):
        rospy.logwarn("enter update")
        while not rospy.is_shutdown():
            # rospy.logwarn("enter while")
            if self.isCaptured:
                # rospy.logwarn("enter if")
                cxe,cye = self.circleFitting(self.x,self.y)
                # rospy.logwarn("bya")
                self.x.clear()
                # rospy.logwarn("bya")
                self.y.clear()
                # rospy.logwarn("bya")
                self.isCaptured = False
                # rospy.logwarn("bya")
                self.sendMsg(cxe,cye)
                # rospy.logwarn("bya")

            self.myrate.sleep()


if __name__ == "__main__" :
    rospy.logwarn("start!")
    try:
        # rospy.logwarn("wei")
        loop_rate = rospy.get_param("/BoH_indicator/loop_rate")
        # rospy.logwarn("wei")
        max_range = rospy.get_param("/BoH_indicator/max_range")
        # rospy.logwarn("wei")
        limit_angle = rospy.get_param("/BoH_indicator/limit_angle")
        func = BohIndicator(loop_rate, max_range, limit_angle)

    except:
        rospy.logwarn("BoH_Indicator : something wrong")

    finally:
        rospy.logwarn("end process")


# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# # @brief ???????????????????????????????????????????????????????????????
# # @author: Atsushi Sakai


# def CircleFitting(x, y):
#     """???????????????????????????????????????????????????????????????
#         input: x,y ????????????????????????????????????

#         output  cxe ??????x??????
#                 cye ??????y??????
#                 re  ??????

#         ??????
#         ????????????????????????????????????????????????????????????????????????????????????????????????
#         http://imagingsolution.blog107.fc2.com/blog-entry-16.html
#     """

#     sumx = sum(x)
#     sumy = sum(y)
#     sumx2 = sum([ix ** 2 for ix in x])
#     sumy2 = sum([iy ** 2 for iy in y])
#     sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

#     F = np.array([[sumx2, sumxy, sumx],
#                   [sumxy, sumy2, sumy],
#                   [sumx, sumy, len(x)]])

#     G = np.array([[-sum([ix ** 3 + ix*iy ** 2 for (ix, iy) in zip(x, y)])],
#                   [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
#                   [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])

#     T = np.linalg.inv(F).dot(G)

#     cxe = float(T[0]/-2)
#     cye = float(T[1]/-2)
#     re = math.sqrt(cxe**2+cye**2-T[2])
#     #print (cxe,cye,re)
#     return (cxe, cye, re)


# if __name__ == '__main__':
#     import matplotlib.pyplot as plt
#     """Unit Test"""
#     #??????????????????????????????
#     cx = 4  # ??????x
#     cy = 10  # ??????y
#     r = 30  # ??????

#     #???????????????????????????
#     plt.figure()
#     x = range(-10, 10)
#     y = []
#     for xt in x:
#         y.append(cy+math.sqrt(r**2-(xt-cx)**2))

#     #????????????????????????
#     (cxe, cye, re) = CircleFitting(x, y)

#     #?????????
#     theta = np.arange(0, 2*math.pi, 0.1)
#     xe = []
#     ye = []
#     for itheta in theta:
#         xe.append(re*math.cos(itheta)+cxe)
#         ye.append(re*math.sin(itheta)+cye)
#     xe.append(xe[0])
#     ye.append(ye[0])

#     plt.plot(x, y, "ob", label="raw data")
#     plt.plot(xe, ye, "-r", label="estimated")
#     plt.plot(cx, cy, "xb", label="center")
#     plt.axis("equal")
#     plt.grid(True)
#     plt.legend()
#     plt.show()
