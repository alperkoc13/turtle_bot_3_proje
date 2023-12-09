#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec  9 20:21:50 2023

@author: alper
"""

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class NesneTespitiEngelKontrol:
    def __init__(self):
        rospy.init_node('nesne_tespiti_engel_kontrol_node', anonymous=True)

        self.image_abone = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.laser_abone = rospy.Subscriber('/scan', LaserScan, self.obstacle_callback)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def image_callback(self, data):
        try:
            cv_goruntu = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        islenmis_goruntu, agirlik_merkezi = self.goruntu_isleme(cv_goruntu)

        cv2.circle(islenmis_goruntu, agirlik_merkezi, 5, (0, 255, 0), -1)

        cv2.imshow('Nesne Tespiti', islenmis_goruntu)
        cv2.waitKey(1)

    def goruntu_isleme(self, goruntu):
        alt_mavi = np.array([100, 50, 50])
        ust_mavi = np.array([140, 255, 255])
        mask = cv2.inRange(goruntu, alt_mavi, ust_mavi)
        sonuc = cv2.bitwise_and(goruntu, goruntu, mask=mask)

        M = cv2.moments(mask)
        if M["m00"] != 0:
            agirlik_merkezi = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            agirlik_merkezi = (0, 0)

        return sonuc, agirlik_merkezi

    def obstacle_callback(self, bilgi):
        algilama_bilgisi = bilgi.ranges

        if min(algilama_bilgisi) < 1.0:
            dur = Twist()
            dur.linear.x = 0.0
            dur.angular.z = 0.0
            self.pub.publish(dur)
        else:
            yurume_hiz = Twist()
            yurume_hiz.linear.x = 0.2
            self.pub.publish(yurume_hiz)

    def calistir(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        nesne_engel_kontrol = NesneTespitiEngelKontrol()
        nesne_engel_kontrol.calistir()
    except rospy.ROSInterruptException:
        pass
