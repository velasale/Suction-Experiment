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
import json
import datetime

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


def start_saving_rosbag(name="trial"):
    
    filename = name
    topics = "/gripper/pressure" \
                + " wrench" \
                + " joint_states" \
                + " /camera/image_raw"
    command = "rosbag record -O " + filename + " " + topics    
    command = shlex.split(command)
    
    return command, subprocess.Popen(command)


def terminate_saving_rosbag(cmd, process):
    """
    Method to finish saving RosBag file
    """
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(cmd[2:]).issubset(proc.cmdline()):
            proc.send_signal(subprocess.signal.SIGINT)
    process.send_signal(subprocess.signal.SIGINT)


def service_call(service):
    """Method to call service from the command line.
    Note: The services are described in the Arduino file"""
    text = "rosservice call " + service
    os.system(text)


class SuctionExperiment():

    def __init__(self):
        super(SuctionExperiment, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('suction_experiment', anonymous=True)

        # ---- 1 - Initial setup
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("suction_cup")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        success_or_failure_publisher = rospy.Publisher('/success_or_failure', String, queue_size=20)
        event_publisher = rospy.Publisher('/experiments_steps', Int32, queue_size=20)

        # ---- 2 - Display Basic Information in the command line
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # ---- 3 - Variables
        # Misc Variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.success_or_failure_publisher = success_or_failure_publisher
        self.event_publisher = event_publisher
        self.planning_frame = planning_frame

        ## Variables for the markers (sampling sphere, apple, ideal starting points)
        self.marker_id = 1
        self.proxy_markers = MarkerArray()
        self.markerPublisher = rospy.Publisher('balloons', MarkerArray, queue_size=1000)
        self.markerTextPublisher = rospy.Publisher('captions', Marker, queue_size=1000)

        wiper = Marker()
        wiper.id = 0
        wiper.action = wiper.DELETEALL
        self.proxy_markers.markers.append(wiper)
        self.markerPublisher.publish(self.proxy_markers)
        self.markerTextPublisher.publish(wiper)

        self.noise_z = 0

        self.previous_pose = tf2_geometry_msgs.PoseStamped()
        
        ## Experiment Parameters
        self.ROBOT_NAME = "ur5e"
        self.experiment_type = "vertical"
        self.pressureAtCompressor = 100
        self.pressureAtValve = 60
        
        self.SUCTION_CUP_NAME = "Suction cup F-BX20 Silicone"
        self.SUCTION_CUP_SPEC = 0.0122
        self.OFFSET = 0.02
        self.SPHERE_RADIUS = 0.075/2
        self.SURFACE = "3DprintedPLA"

    def go_preliminary_position(self):
        """ This function is to avoid the robot from travelling around weird points"""

        # # Place a marker for the apple
        # self.place_marker_sphere(1, 0, 0, 1.0, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, 0.08)
        # # Place a marker for the sampling sphere
        # self.place_marker_sphere(0, 1, 0, 0.2, self.apple_pos_x, self.apple_pos_y, self.apple_pos_z, self.sphereRadius * 2)

        # --- Place a marker with text in RVIZ
        text = "Going to an Prelim Position"
        self.place_marker_text(0, 0, 1.5, 0.1, text)

        print("SHOULD APPEAR")

        # --- Initiate object joint goal
        joint_goal = self.move_group.get_current_joint_values()

        # --- Preliminary position joint values
        joint_goal[0] = + 180 * pi / 180
        joint_goal[1] = - 65 * pi / 180
        joint_goal[2] = - 100 * pi / 180
        joint_goal[3] = - 90 * pi / 180
        joint_goal[4] = + 90 * pi / 180
        joint_goal[5] = +  0 * pi / 180

        # --- Move to the joint goal
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        # --- Compare the joint goal with the current joint state
        current_joints = self.move_group.get_current_joint_values()
        # Print for debugging:
        # print("Final Joints State: ", current_joints)

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_starting_position(self):
        """ This function takes the gripper to the IDEAL starting position, before adding noise.
        * Places the gripper in a position on the surface of the sphere (center at the apple's)
        * Orients the gripper so that its Z-axis points towards the center of the apple.
    """

        # --- Place a marker with text in RVIZ
        text = "Going to an IDEAL Starting Position"
        self.place_marker_text(0, 0, 1.5, 0.1, text)

        # --- ROS tool for transformations among c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)


        # --- Step 1: Set Goal Pose in the intuitive/easy c-frame
        goal_pose = tf2_geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = "sphere"
        goal_pose.header.stamp = rospy.Time(0)

        goal_pose.pose.position.x = 0.115/2
        goal_pose.pose.position.y = 0.115/2
        goal_pose.pose.position.z = - self.OFFSET
       

        roll = 0
        pitch = 0
        yaw = 0
        q = quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # ---- Step 2: Transform Goal Pose into the planning_frame
        try:
            goal_pose_pframe = tf_buffer.transform(goal_pose, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

        # --- Step 3: Move to the goal pose
        self.move_group.set_pose_target(goal_pose_pframe.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()

        # --- Step 4: Compare the goal with the current state
        current_pose = self.move_group.get_current_pose().pose
        self.previous_pose = current_pose
        success = all_close(goal_pose_pframe.pose, current_pose, 0.01)

        return success

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
        caption.header.frame_id = "world"
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
        caption.color.r = 0
        caption.color.g = 1
        caption.color.b = 0
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

    def add_cartesian_noise(self, x_noise, y_noise, z_noise):

        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)    

        # ---- Step 1: Read the current pose in the planning frame
        cur_pose_pframe = tf2_geometry_msgs.PoseStamped()
        cur_pose_pframe.pose = self.move_group.get_current_pose().pose
        cur_pose_pframe.header.frame_id = self.planning_frame
        cur_pose_pframe.header.stamp = rospy.Time(0)

        # ---- Step 2: Transform current pose into the intutitive/easy frame and add noise
        try: 
            cur_pose_ezframe = tf_buffer.transform(cur_pose_pframe, "sphere", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cur_pose_ezframe.pose.position.x += x_noise
        cur_pose_ezframe.pose.position.y += y_noise
        cur_pose_ezframe.pose.position.z += z_noise
        cur_pose_ezframe.header.stamp = rospy.Time(0)

        # ---- Step 3: Transform again the goal pose into the planning frame
        try: 
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Move to the new pose
        self.move_group.set_pose_target(goal_pose_pframe.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # --- Step 5: Compare poses
        cur_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, cur_pose, 0.01)
        
        return success

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
        caption.header.frame_id = "world"
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

    def move_in_z(self, z):

        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)    

        # ---- Step 1: Read the current pose in the planning frame
        cur_pose_pframe = tf2_geometry_msgs.PoseStamped()
        cur_pose_pframe.pose = self.move_group.get_current_pose().pose
        cur_pose_pframe.header.frame_id = self.planning_frame
        cur_pose_pframe.header.stamp = rospy.Time(0)

        # ---- Step 2: Transform current pose into the intutitive/easy frame and add noise
        try: 
            cur_pose_ezframe = tf_buffer.transform(cur_pose_pframe, "sphere", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cur_pose_ezframe.pose.position.z += z
        cur_pose_ezframe.header.stamp = rospy.Time(0)

        # ---- Step 3: Transform again the goal pose into the planning frame
        try: 
            goal_pose_pframe = tf_buffer.transform(cur_pose_ezframe, self.planning_frame, rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        # --- Step 4: Move to the new pose        
        self.move_group.set_pose_target(goal_pose_pframe.pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # --- Step 5: Compare poses
        cur_pose = self.move_group.get_current_pose().pose
        success = all_close(goal_pose_pframe.pose, cur_pose, 0.01)
        
        return success

    def save_metadata(self, filename):
        """
        Create json file with metadata
        """
        
        experiment_info = {
            "generalInfo": {
                "date": str(datetime.datetime.now()),
                "experimentType": "vertical"
            },
            "robotInfo": {
                "robot": self.ROBOT_NAME,
                "noise [mm]": self.noise_z
            },
            "gripperInfo": {
                "Suction Cup": self.SUCTION_CUP_NAME,
                "pressureAtCompressor": self.pressureAtCompressor,
                "pressureAtValve": self.pressureAtValve
            },
            "surfaceInfo": {
                "type": self.SURFACE,
                "radius [m]": self.SPHERE_RADIUS
            }
        }

        filename = filename + ".json"
        with open(filename, "w") as outfile:
            json.dump(experiment_info, outfile, indent=4)
        

    
def main():
    # Step 1: Place robot at starting position
    suction_experiment = SuctionExperiment()
    # suction_experiment.go_preliminary_position
    
    suction_experiment.go_to_starting_position()

    steps = 10
    # noise = suction_experiment.SPHERE_RADIUS / steps
    noise_res = suction_experiment.SUCTION_CUP_SPEC / steps

    # Step 2: Add noise
    for step in range(steps):

        noise = - 1 * noise_res
        print("Noise added [mm]: %.2f" %(noise * 1000 * step))
        
        suction_experiment.noise_z = round(1000*noise*step,2)

        # a. Start Recording Rosbag file
        location = os.path.dirname(os.getcwd())        
        foldername = "/data/"
        name = suction_experiment.experiment_type \
                + "_#" + str(step) \
                + "_pres_" + str(suction_experiment.pressureAtValve) \
                + "_surface_" + suction_experiment.SURFACE \
                + "_radius_" + str(suction_experiment.SPHERE_RADIUS) \
                + "_noise_" + str(suction_experiment.noise_z)    

        filename = location + foldername + name    
        command, rosbag_process = start_saving_rosbag(filename)        

        # c. Apply vacuum
        time.sleep(0.5)
        service_call("openValve")
        time.sleep(0.01)

        # d. Approach the surface
        move1 = suction_experiment.move_in_z(suction_experiment.OFFSET + suction_experiment.SUCTION_CUP_SPEC)
        print("\nMove 1:",move1)

        # e. Retrieve from surface
        move2 = suction_experiment.move_in_z( - suction_experiment.OFFSET - suction_experiment.SUCTION_CUP_SPEC)
        print("\nMove 2:",move2)

        # f. Stop vacuum
        time.sleep(0.01)
        service_call("closeValve")
        time.sleep(0.01)

        # g. Stop recording
        terminate_saving_rosbag(command, rosbag_process)

        # h. And finally save the metadata
        suction_experiment.save_metadata(filename)

        # i. Add noise to the suction cup's location               
        #suction_experiment.add_cartesian_noise(noise* step, 0, 0)   # --> This one is CARTESIAN IN X
        move3 = suction_experiment.add_cartesian_noise(0, 0, noise)
        print("\nMove 3:",move3)
        


if __name__ == '__main__':
    main()
