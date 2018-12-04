#!/usr/bin/env python2

import os
import argparse

import rosbag


def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp x y z qx qy qz qw l_x l_y l_z a_x a_y a_z\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "nav_msgs/Odometry":
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         msg.pose.pose.position.x,
			 msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w,

			 msg.twist.twist.linear.x,
			 msg.twist.twist.linear.y,
			 msg.twist.twist.linear.z,

			 msg.twist.twist.angular.x,
			 msg.twist.twist.angular.y,
			 msg.twist.twist.angular.z,


))
            else:
                assert False, "Unknown message type"
            n += 1
    print('wrote ' + str(n) + ' imu messages to the file: ' + out_filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('--msg_type', default='nav_msgs/Odometry',
                        help='message type')
    parser.add_argument('--output', default='odom_poses.txt',
                        help='output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    print('Saving to file '+out_fn)
    extract(args.bag, args.topic, args.msg_type, out_fn)
