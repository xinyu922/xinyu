#!/usr/bin/env python2
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import os
import argparse
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Extract():

    def __init__(self, ):
        rospy.init_node('get_vrpn_pose', anonymous=True)
        self.slamtec_path_pub = rospy.Publisher('/slamtec_path', Path, queue_size=10)
        self.Oxgd_path_pub = rospy.Publisher('/Oxgd_path', Path, queue_size=10)
        self.sub_vrpn_slamtec = rospy.Subscriber('/vrpn_client_node/slamtec/pose', PoseStamped, self.callbackslamtec)
        self.sub_vrpn_Oxgd = rospy.Subscriber('/vrpn_client_node/Oxgd/pose', PoseStamped, self.callbackOxgd)
        self.slamtec_path = Path()
        self.Oxgd_path = Path()

    
    def callbackslamtec(self, data):
        
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = data.pose.position.x  # 减去初始时刻的位置[10.0, 0.0]
        pose.pose.position.y = data.pose.position.y
        pose.pose.position.z = 0
        pose.pose.orientation.x = data.pose.orientation.x
        pose.pose.orientation.y = data.pose.orientation.y
        pose.pose.orientation.z = data.pose.orientation.z
        pose.pose.orientation.w = data.pose.orientation.w
        self.slamtec_path.header.frame_id = "world"
        self.slamtec_path.header.stamp = rospy.Time.now()
        pose.header.stamp = self.slamtec_path.header.stamp
        self.slamtec_path.poses.append(pose)

            
    def callbackOxgd(self, data):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = data.pose.position.x  # 减去初始时刻的位置[10.0, 0.0]
        pose.pose.position.y = data.pose.position.y
        pose.pose.position.z = 0
        pose.pose.orientation.x = data.pose.orientation.x
        pose.pose.orientation.y = data.pose.orientation.y
        pose.pose.orientation.z = data.pose.orientation.z
        pose.pose.orientation.w = data.pose.orientation.w
        self.Oxgd_path.header.frame_id = "world"
        self.Oxgd_path.header.stamp = rospy.Time.now()
        pose.header.stamp = self.Oxgd_path.header.stamp
        self.Oxgd_path.poses.append(pose)
        
    def run(self):
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            self.slamtec_path_pub.publish(self.slamtec_path)
            self.Oxgd_path_pub.publish(self.Oxgd_path)
            rate.sleep()
        rospy.spin()

if __name__ == '__main__':

    extract = Extract()
    # time.sleep(1)
    extract.run()

