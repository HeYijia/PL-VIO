#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    #print "Extract images from %s on topic %s into %s" % (args.bag_file, args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    #log = open('/media/hyj/Elements/slam_dataset/timestamp.txt','w')
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	#ts = msg.header.stamp
    imgname = "frame%06i.png" % count
    #log.write("{:10.6f}".format(ts.to_sec())+ ' '+imgname + '\n')
	#cv2.imwrite(os.path.join(args.output_dir, imgname), cv_img)
    cv2.imshow("frame",cv_img)
    cv2.waitKey(1)
    #print "time %.6f, Wrote image %i" % (ts.to_sec() ,count)

    count += 1

    bag.close()
    #log.close()

    return

if __name__ == '__main__':
    main()
