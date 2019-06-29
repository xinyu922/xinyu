#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np
import cv2
from PIL import Image, ImageFont, ImageDraw

import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as sensorImg



class ERRIMG:
    def __init__(self):
        self.bridge_ = CvBridge()
        self.amcl_err_ = 0
        self.current_speed_x_ = 0
        self.current_speed_th_ = 0
        self.vo_err_ = 0
        self.vo_max_err_ = 0
        self.vo_min_err_ = 0
        self.vo_mean_err_ = 0
        self.husky_err_sub_ = rospy.Subscriber("/husky_err", Float32MultiArray, self.huskyErrCb)
        self.vo_err_sub_ = rospy.Subscriber("/vo_err", Float32MultiArray, self.slamTecErrCb)
        self.husky_img_pub_ = rospy.Publisher("husky_img", sensorImg, queue_size=1)
        self.vo_img_pub_ = rospy.Publisher("vo_img", sensorImg, queue_size=1)

    def huskyErrCb(self, msg):
        self.amcl_err_ = msg.data[0]
        self.current_speed_x_ = msg.data[1]
        self.current_speed_th_ = msg.data[2]
        errImg.showImg()

    def slamTecErrCb(self, msg):
        self.vo_err_ = msg.data[0]
        self.vo_max_err_ = msg.data[1]
        self.vo_min_err_ = msg.data[2]
        self.vo_mean_err_ = msg.data[3]

    def showImg(self):
        husky_img_title = "导航定位实时测试系统"
        amcl_err = "实时定位误差: {0:.2f}".format(self.amcl_err_) + " m"
        current_speed_x = "前进速度: {0:.2f}".format(self.current_speed_x_) + " m/s"
        current_speed_th = "转向速度: {0:.2f}".format(self.current_speed_th_) + " m/s"

        vo_title = "视觉定位实时测试系统"
        vo_err = "实时误差: {0:.2f}".format(self.vo_err_) + " m"
        vo_max_err = "最大误差: {0:.2f}".format(self.vo_max_err_) + " m"
        vo_min_err = "最小误差: {0:.2f}".format(self.vo_min_err_) + " m"
        vo_mean_err = "平均误差: {0:.2f}".format(self.vo_mean_err_) + " m"

        husky_img = Image.new(mode="RGB", size=(300, 120))
        title_font = ImageFont.truetype('/home/ubuntu/ws/husky_demo_ws/src/vis_marker/scripts/simsun.ttc', size=20)
        font = ImageFont.truetype('/home/ubuntu/ws/husky_demo_ws/src/vis_marker/scripts/simsun.ttc', size=18)
        draw = ImageDraw.Draw(husky_img)
        draw.text((10, 5), unicode(husky_img_title, 'UTF-8'), font=title_font, fill=(255, 255, 255, 0))
        draw.text((10, 50), unicode(amcl_err, 'UTF-8'), font=font, fill=(0, 255, 255, 0))
        draw.text((10, 70), unicode(current_speed_x, 'UTF-8'), font=font, fill=(255, 255, 255, 0))
        draw.text((10, 90), unicode(current_speed_th, 'UTF-8'), font=font, fill=(255, 255, 255, 0))
        img = np.array(husky_img)

        vo_img = Image.new(mode="RGB", size=(300, 150))
        draw = ImageDraw.Draw(vo_img)
        draw.text((10, 5), unicode(vo_title, 'UTF-8'), font=title_font, fill=(255, 255, 255, 0))
        draw.text((10, 50), unicode(vo_err, 'UTF-8'), font=font, fill=(0, 255, 255, 0))
        draw.text((10, 70), unicode(vo_max_err, 'UTF-8'), font=font, fill=(255, 255, 255, 0))
        draw.text((10, 90), unicode(vo_min_err, 'UTF-8'), font=font, fill=(255, 255, 255, 0))
        draw.text((10, 110), unicode(vo_mean_err, 'UTF-8'), font=font, fill=(255, 255, 255, 0))
        img2 = np.array(vo_img)

        try :
            self.husky_img_pub_.publish(self.bridge_.cv2_to_imgmsg(img, 'bgr8'))
            self.vo_img_pub_.publish(self.bridge_.cv2_to_imgmsg(img2, 'bgr8'))
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    rospy.init_node("err_msg_display", anonymous=True)
    errImg = ERRIMG()
    while not rospy.is_shutdown():
        rospy.spin()
