#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import cv2
import rosbag
from cv_bridge import CvBridge


def bagToPng(input_dir, bag_file, output_dir, cam_topic):
    """Method to extract images from bagfile"""

    print(bag_file)
    bag = rosbag.Bag(input_dir + bag_file, "r")
    bridge = CvBridge()
    count = 0
    initial_time_stamp = 0.0

    for topic, msg, t in bag.read_messages(topics=[cam_topic]):

        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Add elapsed time as text
        if count == 0:
            initial_time_stamp = t
        elapsed_time = round((t.to_sec() - initial_time_stamp.to_sec()), 3)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_img, 'time [sec]: ' + str(elapsed_time), (int(cv_img.shape[0] * 0.05), int(cv_img.shape[1] * 0.73)), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # Save file
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
    # location = os.path.dirname(os.getcwd())
    location = "/home/alejo/Documents"
    in_folder = '/data/samples_with_camera/'
    out_folder = '/data/samples_with_camera/media/'
    image_topic = '/usb_cam/image_raw'

    input_folder = location + in_folder
    output_folder = location + out_folder

    # Sweep folder location with bagfiles
    counter = 0
    for file in sorted(os.listdir(input_folder)):
        if file.endswith(".bag"):
            # output_dir = output_folder + "pngs" + str(counter) + "/"
            only_filename = file.split(".bag")[0]
            output_dir = output_folder + only_filename + "/pngs/"
            os.makedirs(output_dir)

            # Uncomment for PNGs
            bagToPng(input_folder, file, output_dir, image_topic)

            # Uncomment for AVIs
            # bagToVideo(input_folder, file, output_dir, image_topic, counter)

            counter += 1


if __name__ == '__main__':
    main()



