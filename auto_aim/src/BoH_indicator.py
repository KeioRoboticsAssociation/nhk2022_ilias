#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class BohIndicator:
    def __init__(self,_loop_rate,_max_range,_limit_angle):
        # init handles
        rospy.init_node("BoH_Indicator")
        self.Boh_pub = rospy.Publisher("Boh_location", Float32MultiArray, queue_size=100)
        self.Boh_sub = rospy.Subscriber("/scan",LaserScan,self.LiDAR_callback,queue_size=100)
        
        # params
        self.loop_rate = rospy.Rate(_loop_rate)
        self.MAX_RANGE = _max_range
        self.LIMIT_ANGLE = _limit_angle
        
        # variables
        self.isCaptured = False
        self.x = list()
        self.y = list()
        
        # methods
        self.update()
    
    def circleFitting(self,x,y):
        sumx = sum(x)
        sumy = sum(y)
        sumx2 = sum([ix ** 2 for ix in x])
        sumy2 = sum([iy ** 2 for iy in y])
        sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

        F = np.array([[sumx2, sumxy, sumx],
                        [sumxy, sumy2, sumy],
                        [sumx, sumy, len(x)]])

        G = np.array([[-sum([ix ** 3 + ix*iy ** 2 for (ix, iy) in zip(x, y)])],
                        [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                        [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])

        T = np.linalg.inv(F).dot(G)

        cxe = float(T[0]/-2)
        cye = float(T[1]/-2)
        re = math.sqrt(cxe**2+cye**2-T[2])

        return (cxe, cye, re)
      
    def LiDARCallback(self,msg):
        for num in range(len(msg.ranges)) :
            theta = msg.angle_min + num*msg.angle_increment
            r = msg.ranges[num]
            if r < self.MAX_RANGE and theta > -1 * self.LIMIT_ANGLE and theta < self.LIMIT_ANGLE:
                self.isCaptured = True
                x_num = msg.ranges[num] * math.cos(msg.angle_min + num*msg.angle_increment)
                y_num = msg.ranges[num] * math.sin(msg.angle_min + num*msg.angle_increment)
                self.x.append(x_num)
                self.y.append(y_num)

    def sendMsg(self,pub_x,pub_y):
        my_msg = Float32MultiArray()
        if pub_x == 0:
            my_msg.data[0] = 0
            my_msg.data[1] = 0
        else :
            my_msg.data[0] = math.sqrt(pub_x*pub_x+pub_y+pub_y)
            my_msg.data[1] = math.atan(pub_y/pub_x)
        self.Boh_pub.publish(my_msg)
        rospy.loginfo(my_msg.data)                  
    
    def update(self):
        while not rospy.is_shutdown():
            if self.isCaptured:
                cxe,cye,re = self.circleFitting(self.x,self.y)
                self.x.clear()
                self.y.clear()
                self.isCaptured = False
                self.sendMsg(cxe,cye)
            rospy.sleep(self.loop_rate)
    

def main():
    try:
        loop_rate = rospy.get_param("loop_rate","default")
        max_range = rospy.get_param("max_range","default")
        limit_angle = rospy.get_param("limit_angle","default")
        func = BohIndicator(loop_rate,max_range,limit_angle)
        
    except:
        rospy.loginfo("BoH_Indicator : something wrong")
        
    finally:
        rospy.loginfo("end process")
        

# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# # @brief 最小二乗法による円フィッティングモジュール
# # @author: Atsushi Sakai


# def CircleFitting(x, y):
#     """最小二乗法による円フィッティングをする関数
#         input: x,y 円フィッティングする点群

#         output  cxe 中心x座標
#                 cye 中心y座標
#                 re  半径

#         参考
#         一般式による最小二乗法（円の最小二乗法）　画像処理ソリューション
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
#     #推定する円パラメータ
#     cx = 4  # 中心x
#     cy = 10  # 中心y
#     r = 30  # 半径

#     #円の点群の擬似情報
#     plt.figure()
#     x = range(-10, 10)
#     y = []
#     for xt in x:
#         y.append(cy+math.sqrt(r**2-(xt-cx)**2))

#     #円フィッティング
#     (cxe, cye, re) = CircleFitting(x, y)

#     #円描画
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
