#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import cv2
import curses

count = 0
bridge = CvBridge()

# rospy.init_node('UndistortImage',anonymous = True)
pub = rospy.Publisher('/image_undistort', Image, queue_size = 1)
 
# cam_path  = '/home/husky/Desktop/usb_cam/'    # 已经建立好的存储cam0 文件的目录
# cam1_path  = '/home/hltt3838/my_c++/VINS_test/BUAA_robot/cam1/'


def image_undistort(img):
    fx=1.97562480e+03
    fy=1.97424657e+03
    cx=9.74580525e+02
    cy=4.24990872e+02
    distcoeffs = np.array([-0.55736824, 0.36627329, 0.00116797, -0.00214494, -0.28372033])
    camera_matrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
    # distcoeffs=np.array([-0.61758395,-0.12767886,-0.00095697,0.00142091,0])

    optimalMat, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distcoeffs, (1920,1080), 1)
    newimage = np.array([])
    try:
        newimage=cv2.undistort(img,camera_matrix,distcoeffs,newimage,optimalMat)
    except:
        print("error!!!!")
    return newimage

def publish_image(imgdata):
    image_temp = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height=1080
    image_temp.width=1920
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tostring()
    # image_temp.is_bigendian=True
    image_temp.header=header
    image_temp.step=1920*3
    pub.publish(image_temp)


def callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5  

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    img_undistort = image_undistort(cv_img)
    img_undistort = cv2.cvtColor(img_undistort, cv2.COLOR_BGR2RGB)
    publish_image(img_undistort)
    # cv2.imshow("undistort_image" , img_undistort)

    # cv2.waitKey(3)  

 
def displayWebcam():
    rospy.init_node('UndistortImage',anonymous = True)
    # make a video_object and init the video object
    
    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    rospy.spin()
 
if __name__ == '__main__':

    displayWebcam()
