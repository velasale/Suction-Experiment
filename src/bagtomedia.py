#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import cv2
import rosbag
from cv_bridge import CvBridge
from bagpy import bagreader


def bagToPng(input_dir, bag_file, output_dir, cam_topic):
    """Method to extract images from bagfile"""

    print(bag_file)
    bag = rosbag.Bag(input_dir + bag_file, "r")
    bridge = CvBridge()
    count = 0

    for topic, msg, t in bag.read_messages(topics=[cam_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % count), cv_img)
        print("Wrote image %i" % count)

        count += 1

    bag.close()


def bagToVideo(input_dir, bag_file, output_dir, cam_topic, counter):
    """Method to extract video from bagfile"""

    bag = rosbag.Bag(input_dir + bag_file)
    bridge = CvBridge()

    out = None
    for topic, msg, t in bag.read_messages(topics=[cam_topic]):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = img.shape

        if out is None:
            fps = bag.get_type_and_topic_info()[1][cam_topic][3]
            out = cv2.VideoWriter(output_dir + str(counter) + '.avi', cv2.VideoWriter_fourcc(*'MP4V'), fps, (w, h))
        out.write(img)

    bag.close()
    out.release()


def main():
    """Extract media from camera topic in rosbag.
    """

    # Define folders
    location = os.path.dirname(os.getcwd())
    in_folder = '/data/data_with_video/'
    out_folder = '/data/data_with_video/media/'
    image_topic = '/usb_cam/image_raw'

    input_folder = location + in_folder
    output_folder = location + out_folder

    # Sweep folder location with bagfiles
    counter = 0
    for file in os.listdir(input_folder):
        if file.endswith(".bag"):
            output_dir = output_folder + "pngs" + str(counter) + "/"
            os.makedirs(output_dir)
            # bagToPng(input_folder, file, output_dir, image_topic)
            bagToVideo(input_folder, file, output_dir, image_topic, counter)

            counter += 1


if __name__ == '__main__':
    main()



