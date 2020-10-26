#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import curses

count = 0
 
cam_path  = '/home/husky/Desktop/usb_cam/'    # 已经建立好的存储cam0 文件的目录
# cam1_path  = '/home/hltt3838/my_c++/VINS_test/BUAA_robot/cam1/'

 
def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    timestr = "%.6f" %  data.header.stamp.to_sec()
            #%.6f表示小数点后带有6位，可根据精确度需要修改；

    cv2.imshow("frame" , cv_img)

    char = screen.getch()

    if char in [32, 83, 115]:
        image_name = str(count)+ ".jpg" #图像命名：时间戳.jpg
        cv2.imwrite(cam_path + image_name, cv_img)  #保存；
        count +=1

    cv2.waitKey(3)  

 
def displayWebcam():
    rospy.init_node('webcam_display', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    bridge = CvBridge()

    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':

    screen = curses.initscr()
    curses.noecho()
    curses.cbreak()
    screen.keypad(1)
    screen.nodelay(1)

    displayWebcam()