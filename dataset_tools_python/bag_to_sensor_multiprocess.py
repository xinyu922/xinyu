# coding:utf-8
# !/usr/bin/python

# Extract images from a bag file.

# PKG = 'beginner_tutorials'
import roslib;  # roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import sys
import time
import multiprocessing as mp


reload(sys)
sys.setdefaultencoding('utf8')

rosbag_file = '/media/dzsb078/D82A47592A4733B2/00_Dataset/无人车_数据集/rosbag/out2/out2_2019-09-26-15-49-35.bag'
data_save_path = '/media/dzsb078/D82A47592A4733B2/00_Dataset/无人车_数据集/rosbag/out2/out2_dataset/'


class GetSensorData():
    def __init__(self):
        self.mkdir()
        # self.get_image_left()
        # self.get_image_right()
        # self.get_imu()
        self.multicore()

    def multicore(self):
        t1 = mp.Process(target=self.get_image_left, args=())
        t2 = mp.Process(target=self.get_image_right, args=())
        t3 = mp.Process(target=self.get_imu, args=())
        t1.start()
        t2.start()
        t3.start()
        t1.join()
        t2.join()
        t3.join()


    def mkdir(self):

        folder_images = os.path.exists(data_save_path + 'images')
        folder_imu = os.path.exists(data_save_path + 'imu')

        folder_images_left_path = data_save_path + 'images/left'
        folder_images_right_path = data_save_path + 'images/right'
        folder_imu_path = data_save_path + 'imu'

        # folder_images = os.path.exists(data_save_path + 'images')
        if not folder_images and not folder_imu:
            os.makedirs(folder_images_left_path)
            os.makedirs(folder_images_right_path)
            os.makedirs(folder_imu_path)
        else:
            pass


    def get_image_left(self):
        self.bridge = CvBridge()
        with rosbag.Bag(rosbag_file, 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                if topic == "/image_raw1":  # 图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print(e)
                    # timestr = "%.6f" %  msg.header.stamp.to_sec()
                    timestr = msg.header.stamp.to_nsec()

                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = str(timestr) + ".png"  # 图像命名：时间戳.png
                    left_path = data_save_path + 'images/left/'
                    cv2.imwrite(left_path + image_name, cv_image)  # 保存；

    def get_image_right(self):
        self.bridge = CvBridge()
        with rosbag.Bag(rosbag_file, 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                if topic == "/image_raw2":  # 图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = msg.header.stamp.to_nsec()
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；
                    image_name = str(timestr) + ".png"  # 图像命名：时间戳.png
                    right_path = data_save_path + 'images/right/'
                    cv2.imwrite(right_path + image_name, cv_image)  # 保存；

    def get_imu(self):
        with rosbag.Bag(rosbag_file, 'r') as bag:  # 要读取的bag文件；
            imu_path = data_save_path + 'imu/imu.txt'
            f_imu = open(imu_path, 'w')
            f_imu.write('# timestamp ang_v_x ang_v_y ang_v_z lin_acc_x lin_acc_y lin_acc_z qx qy qz qw\n')
            for topic, msg, t in bag.read_messages():
                if topic == "/imu/data": #图像的topic；
                    f_imu.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' % (
                             msg.header.stamp.to_sec(),
                             msg.angular_velocity.x,
                             msg.angular_velocity.y,
                             msg.angular_velocity.z,

                             msg.linear_acceleration.x,
                             msg.linear_acceleration.y,
                             msg.linear_acceleration.z,

                             msg.orientation.x,
                             msg.orientation.y,
                             msg.orientation.z,
                             msg.orientation.w))
            f_imu.close()


if __name__ == '__main__':
    try:
        st = time.time()
        getsensordata = GetSensorData()
        st1 = time.time()
        print('mulitiprocessing time:', st1 - st)
    except rospy.ROSInterruptException:
        pass
