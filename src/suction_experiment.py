## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import numpy as np
from numpy import pi, cos, sin, arccos, arange
import os
import psutil
from random import random
import rospy
import time
import shlex
import statistics as st
import subprocess
import sys

## --- Related 3rd party imports
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from std_msgs.msg import String, Int32
import sympy as sym
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


def main():
    # Step 1: Place robot at starting position
    suction_experiment = SuctionExperiment()

    # Step 2: Add noise

    # Step 3: Apply vacuum

    # Step 4: Approach cup towards surface

    # Step 5: Retrieve until cup detaches from surface

    # Go back to step 2 and repeat


if __name__ == '__main__':
    main()
