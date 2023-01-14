## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import numpy as np
from numpy import pi, cos, sin, arccos, arange
import os
from random import random
import rospy
import time
import statistics as st
import subprocess, shlex, psutil
import sys
import rosbag

## --- Related 3rd party imports
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from std_msgs.msg import String, Int32
# import sympy as sym
import tf
from tf.transformations import euler_from_quaternion, quaternion_about_axis, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterpart in another list
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

def terminate_saving_rosbag(cmd, process):
    """
    Method to finish saving RosBag file
    """
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(cmd[2:]).issubset(proc.cmdline()):
            proc.send_signal(subprocess.signal.SIGINT)
    process.send_signal(subprocess.signal.SIGINT)


class SuctionExperiment():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('suction_experiment', anonymous=True)

        # ---- 1 - Initial setup
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        success_or_failure_publisher = rospy.Publisher('/success_or_failure', String, queue_size=20)
        event_publisher = rospy.Publisher('/experiments_steps', Int32, queue_size=20)

        # ---- 2 - Display Basic Information in the command line

        # ---- 3 - Variables
        self.robot = robot
        self.move_group = move_group

        self.markerTextPublisher = rospy.Publisher('captions', Marker, queue_size=1000)

    def go_preliminary_position(self):
        """ This function is to avoid the robot from travelling around weird points"""

        # # Place a marker for the apple
        # self.place_marker_sphere(1, 0, 0, 1.0, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, 0.08)
        # # Place a marker for the sampling sphere
        # self.place_marker_sphere(0, 1, 0, 0.2, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, self.sphereRadius * 2)

        # --- Place a marker for the text
        self.place_marker_text(0, 0, 0 + 0.5, 0.1,
                               "Going to Preliminary Starting Position")

        # --- Initiate object joint goal
        joint_goal = self.move_group.get_current_joint_values()

        # --- Preliminary position joint values
        joint_goal[0] = -  11 * pi / 180
        joint_goal[1] = - 100 * pi / 180
        joint_goal[2] = - 139 * pi / 180
        joint_goal[3] = - 100 * pi / 180
        joint_goal[4] = -  90 * pi / 180
        joint_goal[5] = 0

        # --- Move to the joint goal
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # --- Compare the joint goal with the current joint state
        current_joints = self.move_group.get_current_joint_values()
        # Print for debugging:
        # print("Final Joints State: ", current_joints)

        return all_close(joint_goal, current_joints, 0.01)

    def place_marker_text(self, x, y, z, scale, text):
        """
    Creates a text as a Marker
    @ r,g,b: Indexes of the color in rgb format
    @ a: Alpha value - from 0 (invisible) to 1 (opaque)
    @ x,y,z: coordinates of the marker
    @ scale
    @ text: Text to display in RVIZ
    """
        # Create a marker.  Markers of all shapes share a common type.
        caption = Marker()

        # Set the frame ID and type.  The frame ID is the frame in which the position of the marker
        # is specified.  The type is the shape of the marker, detailed on the wiki page.
        caption.header.frame_id = "/world"
        caption.type = caption.TEXT_VIEW_FACING

        # Each marker has a unique ID number.  If you have more than one marker that you want displayed at a
        # given time, then each needs to have a unique ID number.  If you publish a new marker with the same
        # ID number and an existing marker, it will replace the existing marker with that ID number.
        caption.id = 0

        # Set the action.  We can add, delete, or modify markers.
        caption.action = caption.ADD

        # These are the size parameters for the marker.  The effect of these on the marker will vary by shape,
        # but, basically, they specify how big the marker along each of the axes of the coordinate frame named
        # in frame_id.
        caption.scale.x = scale
        caption.scale.y = scale
        caption.scale.z = scale

        # Color, as an RGB triple, from 0 to 1.
        caption.color.r = 1
        caption.color.g = 1
        caption.color.b = 1
        caption.color.a = 1

        caption.text = text

        # Specify the pose of the marker.  Since spheres are rotationally invarient, we're only going to specify
        # the positional elements.  As usual, these are in the coordinate frame named in frame_id.  Every time the
        # marker is displayed in rviz, ROS will use tf to determine where the marker should appear in the scene.
        # in this case, the position will always be directly above the robot, and will move with it.
        caption.pose.position.x = x
        caption.pose.position.y = y
        caption.pose.position.z = z

        # Set up a publisher.  We're going to publish on a topic called balloon.
        self.markerTextPublisher.publish(caption)

        # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
        rate = rospy.Rate(10)


def main():
    # Step 1: Place robot at starting position
    suction_experiment = SuctionExperiment()
    # suction_experiment.go_preliminary_position()

    # Step 2: Add noise

    # Step 3: Apply vacuum

    # Step  : Record data

    # Start Recording Rosbag file
    filename = "trial"
    topics = "pressure"
    command = "rosbag record -O " + filename + " " + topics
    command = shlex.split(command)
    rosbag_process = subprocess.Popen(command)

    time.sleep(10)
    terminate_saving_rosbag(command, rosbag_process)

    # Step 4: Approach cup towards surface

    # Step 5: Retrieve until cup detaches from surface

    # Go back to step 2 and repeat


if __name__ == '__main__':
    main()
