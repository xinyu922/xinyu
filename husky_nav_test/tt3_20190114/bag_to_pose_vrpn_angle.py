#!/usr/bin/env python2

import os
import argparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rosbag

roll = pitch = yaw = 0.0

def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('timestamp,x,y,z,roll,pitch,yaw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "geometry_msgs/PoseStamped":
                orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                f.write('%.12f, %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         msg.pose.position.x, 
                         msg.pose.position.y,
                         msg.pose.position.z,
                         roll,
                         pitch,
                         yaw
                         )
                        )
            else:
                assert False, "Unknown message type"
            n += 1
    print('wrote ' + str(n) + ' husky pose messages to the file: ' + out_filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('--msg_type', default='geometry_msgs/PoseStamped',
                        help='message type')
    parser.add_argument('--output', default='turtlebot3_poses.csv',
                        help='output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    print('Saving to file '+out_fn)
    extract(args.bag, args.topic, args.msg_type, out_fn)
