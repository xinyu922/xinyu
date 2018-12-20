#!/usr/bin/env python  
import rospy
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

quaternion = (
    -0.00346357583061,
    0.00427715047416,
    0.803023547028,
    -0.595921895588)


euler = euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print roll , pitch , yaw
