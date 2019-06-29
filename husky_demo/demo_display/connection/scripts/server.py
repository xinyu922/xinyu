#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import socket
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 60000
s.bind(("192.168.1.114", port))
print("UDP bound on port %s..." % port)


def main():
    rospy.init_node("server", anonymous=True)
    vo_pub = rospy.Publisher("vo_err", Float32MultiArray, queue_size=1)
    husky_pub = rospy.Publisher("husky_err", Float32MultiArray, queue_size=1)
    husky_path_pub = rospy.Publisher("husky_path", Path, queue_size=1)

    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        data, addr = s.recvfrom(65536)
	print data
        print("Receive from %s:%s" % addr)
        msg = Float32MultiArray()
        temp_data = data.replace("(", "").replace(")", "").split(",")
        if temp_data[0] == "vo":
            msg.data = list(map(float, temp_data[1:]) )
            vo_pub.publish(msg)
        if temp_data[0] == "husky":
            msg.data = list(map(float, temp_data[1:]) )
            husky_pub.publish(msg)
        if temp_data[0] == "path":
            temp_data = data.replace(",[[", ";").replace("]]", "").replace("], [", ";").split(";")
            temp = temp_data[1:]
            temp = [data.split(",") for data in temp]
            print temp[0]
            msg = Path()
            odom = TransformStamped()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "world"
            odom.child_frame_id = "odom"
            odom.transform.translation.x = float(temp[0][0].strip())
            odom.transform.translation.y = float(temp[0][1].strip())
            odom.transform.translation.z = float(temp[0][2].strip())
            odom.transform.rotation.x = float(temp[0][3].strip())
            odom.transform.rotation.y = float(temp[0][4].strip())
            odom.transform.rotation.z = float(temp[0][5].strip())
            odom.transform.rotation.w = float(temp[0][6].strip())
            for pose in temp[1:]:
                if len(pose) == 7:
                    poseStamp = PoseStamped()
                    poseStamp.pose.position.x = float(pose[0])
                    poseStamp.pose.position.y = float(pose[1])
                    poseStamp.pose.position.z = float(pose[2])
                    poseStamp.pose.orientation.x = float(pose[3])
                    poseStamp.pose.orientation.y = float(pose[4])
                    poseStamp.pose.orientation.z = float(pose[5])
                    poseStamp.pose.orientation.w = float(pose[6])
                    msg.poses.append(poseStamp)
            br.sendTransform(odom)
            msg.header.frame_id = "odom"
            husky_path_pub.publish(msg)




if __name__ == '__main__':
    main()
