## --- Standard Library Imports
import copy
import csv
import math
import matplotlib.pyplot as pp
import numpy as np
from numpy import pi, cos, sin, arccos, arange
import os
import keyboard
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
from moveit_msgs.msg import Constraints, JointConstraint

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
    """Start saving a bagfile"""
    
    filename = name

    topics = "/gripper/pressure" \
                + " wrench" \
                + " joint_states" \
                + " /usb_cam/image_raw" \
                + " experiment_steps"


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


def z_noise_experiment(suction_experiment):   

    steps = 12
    reps_at_each_step = 1
    noise_res = 1.3 * suction_experiment.SUCTION_CUP_SPEC * math.cos(suction_experiment.pitch) / steps

    # Step 2: Add noise
    for step in range(steps):

        suction_experiment.step = step

        print("\n **** Step %d of %d ****" %(step, steps))

        suction_experiment.noise_z_command = -1 * noise_res * step
        noise_for_filename = round(suction_experiment.noise_z_command * 1000, 2)
        pitch_for_filename = round(math.degrees(suction_experiment.pitch), 2)

        for rep in range(reps_at_each_step):

            suction_experiment.repetition = rep + 1

            # --- Move to Starting position
            print("Moving to starting position")
            suction_experiment.publish_event("Start")
            # move1 = False
            # while not move1:
            move1 = suction_experiment.go_to_starting_position()
            print("Move 1:", move1)
            # time.sleep(0.01)

            # --- Start Recording Rosbag file
            location = os.path.dirname(os.getcwd())
            foldername = "/data/"
            name = suction_experiment.experiment_type \
                    + "_#" + str(step) \
                    + "_pres_" + str(suction_experiment.pressureAtValve) \
                    + "_surface_" + suction_experiment.SURFACE \
                    + "_radius_" + str(suction_experiment.SPHERE_RADIUS) \
                    + "_noise_" + str(noise_for_filename) \
                    + "_pitch_" + str(pitch_for_filename) \
                    + "_rep_" + str(suction_experiment.repetition)

            filename = location + foldername + name
            command, rosbag_process = start_saving_rosbag(filename)
            print("Start recording Rosbag")
            time.sleep(0.1)

            # --- Add noise to the starting position
            print("Adding cartesian noise of %.2f [mm] in z" % (suction_experiment.noise_z_command * 1000))
            suction_experiment.publish_event("Noise")
            move2 = suction_experiment.add_cartesian_noise(0, 0, suction_experiment.noise_z_command)
            print("Move 2:",move2)
            # time.sleep(0.1)

            # --- Apply vacuum
            print("Applying vacuum")
            suction_experiment.publish_event("Vacuum On")
            service_call("openValve")
            # time.sleep(0.1)

            # --- Approach the surface
            print("Approaching surface")
            suction_experiment.publish_event("Approach")
            move3 = suction_experiment.move_in_z(suction_experiment.OFFSET + suction_experiment.SUCTION_CUP_SPEC * math.cos(suction_experiment.pitch))
            print("Move 3:",move3)

            # Wait some time to have a steady state
            suction_experiment.publish_event("Steady")
            time.sleep(2)

            # --- Retrieve from surface
            print("Retreieving from surface")
            suction_experiment.publish_event("Retrieve")
            move4 = suction_experiment.move_in_z( - suction_experiment.OFFSET - suction_experiment.SUCTION_CUP_SPEC * math.cos(suction_experiment.pitch))
            print("Move 4:",move4)
            # time.sleep(0.1)

            # --- Stop vacuum
            print("Stop vacuum")
            suction_experiment.publish_event("Vacuum Off")
            service_call("closeValve")
            # time.sleep(0.05)

            # --- Stop recording
            terminate_saving_rosbag(command, rosbag_process)
            print("Stop recording Rosbag")
            time.sleep(0.1)

            # --- Finally save the metadata
            suction_experiment.save_metadata(filename)
            print("Saving Metadata")


def x_noise_experiment(suction_experiment):

    steps = 10
    reps_at_each_step = 1
    noise_res = 1.4 * (suction_experiment.SPHERE_RADIUS - suction_experiment.SUCTION_CUP_RADIUS) / steps

    # Step 2: Add noise
    for step in range(steps):

        suction_experiment.step = step

        print("\n **** Step %d of %d ****" %(step, steps))

        suction_experiment.noise_x_command = 1 * noise_res * step
        noise_for_filename = round(suction_experiment.noise_x_command * 1000, 2)
        suction_experiment.noise_z_command = suction_experiment.calc_vertical_noise()
        pitch_for_filename = round(math.degrees(suction_experiment.pitch), 2)

        for rep in range(0, reps_at_each_step):

            suction_experiment.repetition = rep + 1

            # --- Move to Starting position
            print("Moving to starting position")
            suction_experiment.publish_event("Start")
            move1 = suction_experiment.go_to_starting_position()
            print("Move 1:", move1)
            # time.sleep(0.01)

            # --- Start Recording Rosbag file
            location = os.path.dirname(os.getcwd())
            foldername = "/data/"
            name = suction_experiment.experiment_type \
                   + "_#" + str(step) \
                   + "_pres_" + str(suction_experiment.pressureAtValve) \
                   + "_surface_" + suction_experiment.SURFACE \
                   + "_radius_" + str(suction_experiment.SPHERE_RADIUS) \
                   + "_noise_" + str(noise_for_filename) \
                   + "_pitch_" + str(pitch_for_filename) \
                   + "_rep_" + str(suction_experiment.repetition)

            filename = location + foldername + name
            command, rosbag_process = start_saving_rosbag(filename)
            print("Start recording Rosbag")
            time.sleep(0.1)

            # --- Add noise to the starting position
            print("Adding cartesian noise of %.2f [mm] in z" % (suction_experiment.noise_x_command * 1000))
            suction_experiment.publish_event("Noise")
            # Note: made the x noise negative, to avoid the robot doing weird movements
            move2 = suction_experiment.add_cartesian_noise(-1 * suction_experiment.noise_x_command, 0, 0)
            move2 = suction_experiment.add_cartesian_noise(0, 0, suction_experiment.noise_z_command)
            print("Move 2:", move2)
            # time.sleep(0.1)

            # --- Apply vacuum
            print("Applying vacuum")
            suction_experiment.publish_event("Vacuum On")
            service_call("openValve")
            # time.sleep(0.1)

            # --- Approach the surface
            print("Approaching surface")
            suction_experiment.publish_event("Approach")
            move3 = suction_experiment.move_in_z(suction_experiment.OFFSET + suction_experiment.SUCTION_CUP_SPEC * math.cos(suction_experiment.pitch))
            print("Move 3:", move3)

            # Wait some time to have a steady state
            suction_experiment.publish_event("Steady")
            time.sleep(2)

            # --- Retrieve from surface
            print("Retreieving from surface")
            suction_experiment.publish_event("Retrieve")
            move4 = suction_experiment.move_in_z( - suction_experiment.OFFSET - suction_experiment.SUCTION_CUP_SPEC * 1.5) # a bit more to avoid hitting when going to starting point
            print("Move 4:",move4)
            # time.sleep(0.1)

            # --- Stop vacuum
            print("Stop vacuum")
            suction_experiment.publish_event("Vacuum Off")
            service_call("closeValve")
            time.sleep(0.05)

            # --- Stop recording
            terminate_saving_rosbag(command, rosbag_process)
            print("Stop recording Rosbag")
            time.sleep(1)

            # --- Finally save the metadata
            suction_experiment.save_metadata(filename)
            print("Saving Metadata")


def simple_cup_experiment(suction_experiment):   

    steps = 5
    # noise = suction_experiment.SPHERE_RADIUS / steps
    noise_res = suction_experiment.SUCTION_CUP_SPEC / steps

    # Step 2: Add noise
    for step in range(steps):

        print("\n **** Step %d of %d ****" %(step, steps))

        # suction_experiment.noise_z_command = -1 * noise_res * step
        # noise_for_filename = round(suction_experiment.noise_z_command * 1000, 2)
        
        # --- Start Recording Rosbag file
        location = os.path.dirname(os.getcwd())        
        foldername = "/data/"
        name = suction_experiment.experiment_type \
                + "_#" + str(step) \
                + "_pres_" + str(suction_experiment.pressureAtValve) \
                + "_surface_" + suction_experiment.SURFACE \
                + "_radius_" + str(suction_experiment.SPHERE_RADIUS)

        filename = location + foldername + name    
        command, rosbag_process = start_saving_rosbag(filename)     
        print("Start recording Rosbag")
        time.sleep(1)   

        # # --- Move to Starting position
        # print("Moving to starting position")
        suction_experiment.publish_event("Start")
        # move1 = suction_experiment.go_to_starting_position()
        # print("Move 1:", move1)
        time.sleep(1)

        # # --- Add noise to the starting position                
        # print("Adding cartesian noise of %.2f [mm] in z" % (suction_experiment.noise_z_command * 1000))
        # suction_experiment.publish_event("Noise")
        # time.sleep(0.001)
        # move2 = suction_experiment.add_cartesian_noise(0, 0, suction_experiment.noise_z_command)
        # print("Move 2:",move2)
        # time.sleep(0.1)

        # --- Apply vacuum
        print("Applying vacuum")
        suction_experiment.publish_event("Vacuum On")
        # time.sleep(0.001)
        service_call("openValve")
        time.sleep(1)

        # # --- Approach the surface
        # print("Approaching surface")
        # suction_experiment.publish_event("Approach")
        # time.sleep(0.001)
        # move3 = suction_experiment.move_in_z(suction_experiment.OFFSET + suction_experiment.SUCTION_CUP_SPEC)
        # print("Move 3:",move3)

        # Wait some time to have a steady state
        suction_experiment.publish_event("Steady")
        time.sleep(2)

        # # --- Retrieve from surface
        # print("Retreieving from surface")
        # suction_experiment.publish_event("Retreive")
        # time.sleep(0.001)
        # move4 = suction_experiment.move_in_z( - suction_experiment.OFFSET - suction_experiment.SUCTION_CUP_SPEC)
        # print("Move 4:",move4)
        # time.sleep(0.1)

        # --- Stop vacuum
        print("Stop vacuum")
        suction_experiment.publish_event("Vacuum Off")
        # time.sleep(0.01)
        service_call("closeValve")
        time.sleep(1)

        # --- Stop recording
        terminate_saving_rosbag(command, rosbag_process)
        print("Stop recording Rosbag")
        time.sleep(1)

        # ---- Ffinally save the metadata
        suction_experiment.save_metadata(filename)
        print("Saving Metadata")     


def calibrate_zero(suction_experiment):
    
    suction_experiment.go_to_starting_position()
    delta = 1/1000

    # ask user to hit one arrow, and the move
    print("Type 'w' = +x, 's' = -x, 'a' = -y, 'd' = +y, 'q' = +z, 'z' = -z, or 'exit'")
    
    dx = 0
    dy = 0
    dz = 0

    jog= 'q'
    while jog != 'exit':
        jog = input()

        if jog == 'w':
            "Move in +y"
            suction_experiment.add_cartesian_noise(0, +delta, 0)
            dy += 1            

        elif jog == 's':
            "Move in -y"
            suction_experiment.add_cartesian_noise(0, -delta, 0)
            dy -= 1
        
        elif jog =='d':
            "Move in -x"
            suction_experiment.add_cartesian_noise(-delta, 0, 0)
            dx += 1
        
        elif jog == 'a':
            "Move in +x"
            suction_experiment.add_cartesian_noise(+delta, 0, 0)
            dx -= 1

        elif jog =='q':
            "Move in -z"
            suction_experiment.add_cartesian_noise(0, 0, -delta)
            dz += 1
        
        elif jog == 'z':
            "Move in +z"
            suction_experiment.add_cartesian_noise(0, 0, +delta)
            dz -= 1
        
        print("dx= %.0d, dy= %.0d,  dz= %0.d" % (dx, dy, dz))
    
   
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
        event_publisher = rospy.Publisher('/experiment_steps', String, queue_size=20)
        markerTextPublisher = rospy.Publisher('captions', Marker, queue_size=1000, latch=True)

        # ---- 2 - Display Basic Information in the command line
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # ---- 3 - Variables
        # Misc Variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.event_publisher = event_publisher
        self.planning_frame = planning_frame

        ## Variables for the markers (sampling sphere, apple, ideal starting points)
        self.markerTextPublisher = markerTextPublisher
        wiper = Marker()
        wiper.id = 0
        wiper.action = Marker.DELETEALL
        self.markerTextPublisher.publish(wiper)
        self.event_publisher = event_publisher
                      
        ## Experiment Parameters
        self.ROBOT_NAME = "ur5e"
        self.experiment_type = "vertical"
        self.pressureAtCompressor = 100
        self.pressureAtValve = 50
        self.roll = math.radians(0)
        self.pitch = math.radians(0)
        self.repetition = 0
        
        # Source https://www.piab.com/inriverassociations/0206204/#specifications
        self.SUCTION_CUP_NAME = "Suction cup F-BX20 Silicone"
        self.SUCTION_CUP_SPEC = 0.010
        self.SUCTION_CUP_RADIUS = 0.021 / 2

        self.OFFSET = 0.015     # This should be checked with the calibration_zero function. Look at 'dz'
        self.SPHERE_RADIUS = 0.085/2
        self.SURFACE = "3DprintedPLA"

        ## Experiment Variables
        self.noise_z_command = 0
        self.noise_z_real = 0
        self.noise_x_command = 0
        self.noise_x_real = 0
        self.start_pose = tf2_geometry_msgs.PoseStamped()
        self.previous_pose = tf2_geometry_msgs.PoseStamped()
        self.step = 0

    # --- Moving Functions ---

    def go_preliminary_position(self):
        """ This function is to avoid the robot from travelling around weird points"""       

        # --- Place a marker with text in RVIZ
        text = "Going to an Preliminary Pose"
        self.place_marker_text(0, 0, 1.5, 0.1, text)
        
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
        text = "Going to an IDEAL Starting Pose"
        self.place_marker_text(0, 0, 1.5, 0.05, text)

        # --- ROS tool for transformations among c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # --- Step 1: Set Goal Pose in the intuitive/easy c-frame
        goal_pose = tf2_geometry_msgs.PoseStamped()
        goal_pose.header.frame_id = "sphere"
        goal_pose.header.stamp = rospy.Time(0)

        goal_pose.pose.position.x = 0.115/2
        goal_pose.pose.position.y = 0.115/2
        z_calibration = 1/1000      # This value comes after running the calibration method and measruing 'dz', which should be the ofsset value once the cup just touches the surface
        goal_pose.pose.position.z = - (self.OFFSET + (self.SPHERE_RADIUS - 0.075/2) + z_calibration)
        #goal_pose.pose.position.z = - self.OFFSET
       
        roll = 0
        pitch = self.pitch
        yaw = 0
        q = quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.start_pose = goal_pose

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

    def add_cartesian_noise(self, x_noise, y_noise, z_noise):
        
        # --- Place a marker with text in RVIZ
        text = "Step No " + str(self.step) + "\nAdding cartesian noise"
        self.place_marker_text(0, 0, 1.5, 0.05, text)

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
        
        self.check_real_noise()

        return success

    def move_in_z(self, z):

        # --- Place a marker with text in RVIZ
        text = "Moving in Z-Axis"
        self.place_marker_text(0, 0, 1.5, 0.05, text)

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

    # --- Additional functions ---
       
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
        caption.color.g = 0
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
    
    def check_real_noise(self):
        """Get the real noise by comparing the star pose with the robot's current pose"""

        start_z = self.start_pose.pose.position.z
        start_x = self.start_pose.pose.position.x

        # Read current pose and transform into the sphere cframe
        # --- ROS tool for transformation across c-frames
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)    

        current = self.move_group.get_current_pose()
        current.header.stamp = rospy.Time(0)

        # ---- Step 2: Transform current pose into the intutitive/easy frame and add noise
        try: 
            cur_pose_ezframe = tf_buffer.transform(current, "sphere", rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        self.noise_x_real = cur_pose_ezframe.pose.position.x - start_x
        self.noise_z_real = cur_pose_ezframe.pose.position.z - start_z
        
        print("Xaxis - Commanded noise: %.2f, and Real noise: %.2f" %(self.noise_x_command * 1000, self.noise_x_real * 1000))
        print("Zaxis - Commanded noise: %.2f, and Real noise: %.2f" %(self.noise_z_command * 1000, self.noise_z_real * 1000))

    def save_metadata(self, filename):
        """
        Create json file with metadata
        """
        
        experiment_info = {
            "generalInfo": {
                "date": str(datetime.datetime.now()),
                "experimentType": self.experiment_type,
                "repetition": self.repetition
            },
            "robotInfo": {
                "robot": self.ROBOT_NAME,
                "z noise command [m]": self.noise_z_command,
                "z noise real [m]": self.noise_z_real,
                "x noise command [m]": self.noise_x_command,
                "x noise real [m]": self.noise_x_real,
                "roll [rad]": self.roll,
                "pitch [rad]": self.pitch

            },
            "gripperInfo": {
                "Suction Cup": self.SUCTION_CUP_NAME,
                "pressureAtCompressor [PSI]": self.pressureAtCompressor,
                "pressureAtValve [PSI]": self.pressureAtValve
            },
            "surfaceInfo": {
                "type": self.SURFACE,
                "radius [m]": self.SPHERE_RADIUS
            }
        }

        filename = filename + ".json"
        with open(filename, "w") as outfile:
            json.dump(experiment_info, outfile, indent=4)
        
    def publish_event(self, event):
        self.event_publisher.publish(event)

    def calc_vertical_noise(self):
        """Calculate the amount of movement in z required to keep the suction-cup either tangent or
        edge-touching the sphere"""

        delta_x = self.noise_x_command

        x_tangent_threshold = self.SPHERE_RADIUS * math.sin(self.pitch)
        y_tangent_threshold = self.SPHERE_RADIUS * math.cos(self.pitch)

        x_part = self.SUCTION_CUP_RADIUS * math.cos(self.pitch)
        y_part = self.SUCTION_CUP_RADIUS * math.sin(self.pitch)

        x_right_shift = delta_x - x_part
        x_left_shift = delta_x + x_part

        if x_left_shift < x_tangent_threshold:
            print("First Part")
            # First part
            y_shift = self.SPHERE_RADIUS - math.sqrt(self.SPHERE_RADIUS ** 2 - x_left_shift ** 2)
            delta_z = - y_part + y_shift
        elif x_right_shift > x_tangent_threshold:
            print("Last Part")
            # Last part
            y_shift = self.SPHERE_RADIUS - math.sqrt(self.SPHERE_RADIUS ** 2 - x_right_shift ** 2)
            delta_z = + y_part + y_shift
        else:
            print("Middle Part")
            # Tangent part
            x_middle_shift = x_left_shift - x_tangent_threshold
            y_middle_shift = x_middle_shift * math.tan(self.pitch) + (self.SPHERE_RADIUS - y_tangent_threshold)
            delta_z = - y_part + y_middle_shift

        return delta_z

    
def main():

    # TODO Build ROS package
    # TODO Add a check that the solver found a solution, otherwise try again

    # Step 1: Place robot at preliminary position
    suction_experiment = SuctionExperiment()
    #suction_experiment.go_preliminary_position()

    # Step 2: Get some info from the user
    print("\n\n ***** Suction Cups Experiments *****")
    print("a. Type Experiment: 1 (zNoise), 2 (xNoise), 3 (simple suction), or 4 (calibrate zero)")
    experiment = ''
    while ((experiment != "1") and (experiment != "2") and (experiment != "3") and (experiment != "4")):
        experiment = input()        
    suction_experiment.experiment_type = str(experiment)

    print("b. Type of Surface:")
    suction_experiment.SURFACE = str(input())

    print("c. Pressure at the Valve [PSI]): ")
    print("Tip: If the desired pressure is lower than the current one (at the pressure regulator),\n then first pass that pressure and the go up to the desired pressure")
    suction_experiment.pressureAtValve = int(input())    

    print("d. Diameter of Surface [mm]:")
    suction_experiment.SPHERE_RADIUS = int(input()) / 2000

    print("e. Pitch angle [deg]:")
    suction_experiment.pitch = math.radians(int(input()))

    # Step 3: Run the experiment
    if experiment == "1":
        # Perform the z_noise experiment
        suction_experiment.experiment_type = "vertical"
        z_noise_experiment(suction_experiment)      
    
    elif experiment == "2":
        # Perform x noise experiment
        suction_experiment.experiment_type = "horizontal"
        x_noise_experiment(suction_experiment)    
    
    elif experiment == "3":
        # Perform x noise experiment
        suction_experiment.experiment_type = "simple_suction"
        simple_cup_experiment(suction_experiment) 
    
    elif experiment == "4":
        # Calibrate the zero location - upper quadrant of sphere"
        suction_experiment.experiment_type = experiment
        calibrate_zero(suction_experiment)


if __name__ == '__main__':
    main()
