#!/usr/bin/env python2

import os
import argparse

import rosbag


def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('#timestamp,ang_v_x,ang_v_y,ang_v_z,lin_acc_x,lin_acc_y,lin_acc_z,qx,qy,qz,qw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "sensor_msgs/Imu":
                f.write('%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f\n' %
                        (msg.header.stamp.to_sec(),
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
    parser.add_argument('--msg_type', default='sensor_msgs/Imu',
                        help='message type')
    parser.add_argument('--output', default='imu.csv',
                        help='output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    print('Saving to file '+out_fn)
    extract(args.bag, args.topic, args.msg_type, out_fn)
