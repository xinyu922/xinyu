#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv

roll = pitch = yaw = 0.0



def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    
    with open('time-rpy.txt', 'a') as rpy_file:
        rpy_file.write("{0} ".format(msg.header.stamp) +  "{0} ".format(roll) + "{0} ".format(pitch) + "{0} ".format(yaw) + "\n")
        print msg.header.stamp , roll, pitch, yaw
        rpy_file.close()
    


rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/vrpn_client_node/husky/pose', PoseStamped, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():    
    quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    r.sleep()
    
