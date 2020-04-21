#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry

import os
import argparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pitch = yaw = roll =  0.0
f = open('./vin_camera_pose.csv', 'w')
f.write('timestamp,x,y,z,pitch,roll,yaw\n')

def extract():

    rospy.init_node('get_vins_path', anonymous=True)
    rospy.Subscriber('/vins_estimator/camera_pose', Odometry, callback)
    rospy.spin()

def callback(data):

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    f.write('%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f\n' %
            (
            data.header.stamp.to_sec(),
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            roll,
			pitch,            
			yaw 
			
            
            )
            )


if __name__ == '__main__':

    extract()
