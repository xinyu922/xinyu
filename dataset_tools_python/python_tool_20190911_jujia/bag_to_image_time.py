#!/usr/bin/env python2

import os
import argparse

import rosbag


def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# frame, timestamp\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "sensor_msgs/Image":
                f.write('%d %s %.12f\n' %
                        (
                            n,
                            ',',
                            msg.header.stamp.to_sec()
       
                         )
                        )
            else:
                assert False, "Unknown message type"
            n += 1
    print('wrote ' + str(n) + ' messages to the file: ' + out_filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('--msg_type', default='sensor_msgs/Image',
                        help='message type')
    parser.add_argument('--output', default='image_time.csv',
                        help='output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print('Extract pose from bag '+args.bag+' in topic ' + args.topic)
    print('Saving to file '+out_fn)
    extract(args.bag, args.topic, args.msg_type, out_fn)
