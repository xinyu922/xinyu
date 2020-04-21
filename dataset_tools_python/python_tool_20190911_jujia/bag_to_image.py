# coding:utf-8
#!/usr/bin/python
 
# Extract images from a bag file.
 
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
 
# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys
 
left_path = '/home/bdi048/Documents/20190911_jujia/jujia_robot_1/image_file/left/'
right_path = '/home/bdi048/Documents/20190911_jujia/jujia_robot_1/image_file/right/'
depth_path = '/home/bdi048/Documents/20190911_jujia/jujia_robot_1/image_file/depth/'
 
class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('/home/bdi048/Documents/20190911_jujia/jujia_robot_1/jujia_robot_1_2019-09-11-13-40-53.bag', 'r') as bag:  #要读取的bag文件；
            for topic,msg,t in bag.read_messages():
                if topic == "/mynteye/left/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print(e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(left_path + image_name, cv_image)  #保存；
                elif topic == "/mynteye/right/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print(e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(right_path + image_name, cv_image)  #保存；
                elif topic == "/mynteye/depth/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"mono16")
                        except CvBridgeError as e:
                            print(e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(depth_path + image_name, cv_image)  #保存；
 
if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
