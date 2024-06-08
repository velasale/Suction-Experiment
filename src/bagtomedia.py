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
        cv2.imwrite(os.path.join(output_dir, str(int(elapsed_time*1000)) + ".png"), cv_img)
        print("Wrote image %i" % count)

        count += 1

    bag.close()


def bagToVideo(input_dir, bag_file, output_dir, cam_topic, only_filename):
    """Method to extract video from bagfile"""

    bag = rosbag.Bag(input_dir + bag_file)
    bridge = CvBridge()

    out = None
    for topic, msg, t in bag.read_messages(topics=[cam_topic]):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = img.shape

        if out is None:
            fps = bag.get_type_and_topic_info()[1][cam_topic][3]
            out = cv2.VideoWriter(output_dir + only_filename + '.avi', cv2.VideoWriter_fourcc(*'MP4V'), fps, (w, h))
        out.write(img)

    bag.close()
    out.release()


def main():
    """Extract media from camera topic in rosbag.
    """

    # Define folders
    # location = os.path.dirname(os.getcwd())
    # location = "/home/alejo/Documents"
    # in_folder = '/data/samples_with_camera/'
    # out_folder = '/data/samples_with_camera/'
    # image_topic = '/usb_cam/image_raw'
    # image_topic = '/camera/color/image_raw'
    # image_topic = '/camera/aligned_depth_to_color/image_raw'
    image_topic = '/camera/depth/camera_info'
    image_topic = '/device_0/sensor_1/Color_0/image/data'

    #
    # input_folder = location + in_folder
    # output_folder = location + out_folder

    # # Sweep folder location with bagfiles
    # for file in sorted(os.listdir(input_folder)):
    #     if file.endswith(".bag"):
    #
    #         only_filename = file.split(".bag")[0]
    #         output_dir = output_folder + only_filename + "/pngs/"
    #
    #         # Create Dir if it doesnt exists
    #         if not os.path.exists(output_dir):
    #             os.makedirs(output_dir)
    #
    #         # Uncomment for PNGs
    #         bagToPng(input_folder, file, output_dir, image_topic)
    #
    #         # Uncomment for AVIs
    #         bagToVideo(input_folder, file, output_dir, image_topic, only_filename)



    # For a single file
    input_folder = '/home/alejo/Documents/temporal/'
    # file = '20230724_proxy_sample_0_yaw_0_rep_0_stiff_high_force_low.bag'
    file = 'papa.bag'
    only_filename = file.split(".bag")[0]
    output_dir = input_folder + only_filename + "/pngs/"

    # Create Dir if it doesnt exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Uncomment for PNGs
    bagToPng(input_folder, file, output_dir, image_topic)


if __name__ == '__main__':
    main()



