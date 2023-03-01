#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import cv2
import rosbag
from cv_bridge import CvBridge


def main():
    """Extract a folder of images from a rosbag.
    """

    bag_file = '/home/velasale/gripper_ws/src/Suction-Experiment/data/data_with_video/PUTAS.bag'
    image_topic = '/usb_cam/image_raw/'
    output_dir = '/home/velasale/gripper_ws/src/Suction-Experiment/data/data_with_video/videos/'

    bag = rosbag.Bag(bag_file, "r")
    info = bag.get_type_and_topic_info()
    print(info)
    bridge = CvBridge()
    count = 0

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % count), cv_img)
        print("Wrote image %i" % count)

        count += 1

    bag.close()

    return


if __name__ == '__main__':
    main()
