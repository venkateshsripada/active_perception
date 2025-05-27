#!/usr/bin/env python

import os
import sys
import pdb
import copy
import pickle
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import moveit_commander
import moveit_msgs.msg

import config
import math
import numpy as np
import tf2_ros
import tf.transformations
from tf2_geometry_msgs import do_transform_pose

from actionlib_msgs.msg import GoalStatusArray
import actionlib

# import openai
# import call_vlm
import base64
# from openai import OpenAI
from rich import print
import tf
import time

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray

GPT_4okey = 'sk-proj-Qt5Xmbq40CEgfTU0wMVgT3BlbkFJi9JjJ140dihGT0a0LGT8'


class SimpleMoveit():
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.grid_image_sub = rospy.Subscriber("/aruco_simple/result", Image, self.aruco_callback)
        self.vertice_pose_x_sub = rospy.Subscriber("/aruco_simple/matrix_x", Float32MultiArray, self.matrix_x_callback)
        self.vertice_pose_y_sub = rospy.Subscriber("/aruco_simple/matrix_y", Float32MultiArray, self.matrix_y_callback)
        self.vertice_pose_z_sub = rospy.Subscriber("/aruco_simple/matrix_z", Float32MultiArray, self.matrix_z_callback)
        self.grid_visualised_sub = rospy.Subscriber("/aruco_simple/Grid_visualised", Bool, self.grid_visual_callback)
        self.pixel_to_grid_sub = rospy.Subscriber("/aruco_simple/pixel_to_grid", Image, self.pixel_to_grid_callback)
        self.moveit_reached_sub = rospy.Subscriber("/move_group/status", GoalStatusArray, self.moveit_reached_callback)

        self.velocity_scale = 0.10
        
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("detect_marker_motion")
        # self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group.set_max_velocity_scaling_factor(self.velocity_scale)

        self.move_group.set_pose_reference_frame("base_link")

        if use_franka_robot:
            self.recovery_client = actionlib.SimpleActionClient( "/franka_control/error_recovery", ErrorRecoveryAction )
            self.recovery_client.wait_for_server()
            self.recoverError = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=10)
            self.move_group.set_end_effector_link("panda_hand_tcp")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.listener = tf.TransformListener()

        self.saved_new_grid = False
        self.got_vertices_mapping = False
        self.start_time = None

        self.run_pipeline = True
        self.memory_all = {}

        self.count = 0
        self.angle = 35

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)

    def aruco_callback(self, data):
        try:
            self.aruco_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            self.aruco_image = self.bridge.imgmsg_to_cv2(data)
    
    def matrix_x_callback(self, msg):
        self.matrix_x = msg.data
    
    def matrix_y_callback(self, msg):
        self.matrix_y = msg.data

    def matrix_z_callback(self, msg):
        self.matrix_z = msg.data

    def grid_visual_callback(self, msg):
        self.grid_visualised = msg

    def pixel_to_grid_callback(self, data):
        self.pixel_grid_cb = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC2")

    def moveit_reached_callback(self, msg):
        self.moveit_reached_msg = msg

    def save_grid_image(self):
        try:
            if (self.grid_visualised.data):
                
                # Check if the 3D grid has been visible for 2 seconds continuously. 
                # This check is important as the cube keeps flickering if the marker is not correctly visible
                if self.start_time is None:
                    self.start_time = time.time()
                
                elif time.time() - self.start_time >= 2:
                    # print("Grid is visualised ")
                    if not self.got_vertices_mapping:
                        self.get_vertices_pose()
                    
                    pos = str(self.count)
                    # Save raw image
                    self.raw_image_name = self.save_image(pos, raw=True, aruco=False)
                    # Check if grid is formed and save grid image
                    self.aruco_image_name = self.save_image(pos, raw=False, aruco=True)

                    # if self.count < 2:
                    #     self.count = self.count + 1
                    self.saved_new_grid = True
            else:
                self.start_time = None
                                    
        except AttributeError:
            # print("Grid not visualised")
            self.start_time = None

    def round_half_up(self, n):
        return float(math.floor(n + 0.5) if n > 0 else math.ceil(n - 0.5))
    
    def get_vertices_pose(self):
        try:
            self.PoseMatrix_x = np.array(self.matrix_x).reshape(numDots, numDots)
            self.PoseMatrix_y = np.array(self.matrix_y).reshape(numDots, numDots)
            self.PoseMatrix_z = np.array(self.matrix_z).reshape(numDots, 1)

            # x,y pixel value of dots on the grid
            # self.pixel_grid = np.array(self.pixel_grid_cb).reshape(numDots, numDots, 2) 
            self.got_vertices_mapping = True
            # print(self.PoseMatrix_x)
            # print(self.PoseMatrix_y)
            # print(self.PoseMatrix_z)

            # for k in range(len(self.PoseMatrix_z)):
            #     for i in range(len(self.PoseMatrix_x)):
            #         for j in range(len(self.PoseMatrix_y)):
            #             print("Going to ({},{}) = ({},{},{})".format(i,j, self.PoseMatrix_x[i,j], self.PoseMatrix_y[i,j], self.PoseMatrix_z[k,0]))
            #             self.go_to_position(self.PoseMatrix_x[i,j], self.PoseMatrix_y[i,j], self.PoseMatrix_z[k,0])
            #         self.predefined_postions("center")
        except AttributeError:
            print("Did not get matrix information")

            
    
    def print_stuff(self):
        planning_frame = self.move_group.get_planning_frame()
        print("================== Planning Frame is: %s " % planning_frame)

        eef_link = self.move_group.get_end_effector_link()
        print("================== EEF link is %s" % eef_link )

        print("================== Available groups are ")
        group_names = self.robot.get_group_names()
        print(group_names)
        
        print("================== Current robot state is============")
        robot_state = self.robot.get_current_state()
        print(robot_state)
        
    def franka_pose_base_link(self):
        """
        To be called only when using Franka Panda robot. 
        """
        current_pose = self.move_group.get_current_pose().pose

        # Prepare the pose to transform
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.move_group.get_pose_reference_frame()
        pose_stamped.pose = current_pose

        self.listener.waitForTransform( "base_link","panda_link0",rospy.Time(), rospy.Duration(1.0))
        transform = self.tf_buffer.lookup_transform("base_link", "panda_link0",rospy.Time())

        pose_in_base_link = do_transform_pose(pose_stamped, transform)
        # print("Transformed pose in base_link frame:")
        # print(pose_in_base_link.pose)

        return pose_in_base_link
    
    def recover_error(self):
        """Quickly recover from a soft error (e.g. someone pushed the robot while in position control."""

        command = ErrorRecoveryActionGoal()
        self.recoverError.publish(command)
    
    def rotate_gripper(self, gripper_orientation):
        pose_goal = Pose()

        if use_franka_robot:
            cur_pose = self.franka_pose_base_link()
            x_val = cur_pose.pose.position.x
            y_val = cur_pose.pose.position.y
            z_val = cur_pose.pose.position.z
        pose_goal.position.x = x_val
        pose_goal.position.y = y_val
        pose_goal.position.z = z_val

        # Orientation: Align gripper with principal axis
        pose_goal.orientation.x = gripper_orientation[0]
        pose_goal.orientation.y = gripper_orientation[1]
        pose_goal.orientation.z = gripper_orientation[2]
        pose_goal.orientation.w = gripper_orientation[3]

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
    
    def go_to_position(self, x_pos, y_pos, z_pos, orient_x):

        pose_goal = Pose()
        if use_franka_robot:
            pose_goal.orientation.x = 0.7372128176402045
            pose_goal.orientation.y = 0.6752785098787415
            pose_goal.orientation.z = 0.022664939653652464
            pose_goal.orientation.w = 0.0015799093117836137
        else:
            pose_goal.orientation.x = -0.023273475657969504
            pose_goal.orientation.y = 0.9995675584715471
            pose_goal.orientation.z = 0.017924180374535442
            pose_goal.orientation.w = 0.001328585687693568
        pose_goal.position.x = x_pos - 0.015
        # print("Y-pos is: ", y_pos)
        if y_pos > 0.40:
            y_pos = y_pos - 0.080
        else:
            y_pos = y_pos - 0.060
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos 

        # self.listener.waitForTransform("base_link", "tool0",rospy.Time(), rospy.Duration(1.0))
        # (trans, rot) = self.listener.lookupTransform("base_link", "tool0", rospy.Time())
        # print("Currently at", trans)
        # print("----------------------------------------")

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        # Looks right or towards base
        if orient_x == 1:
            theta_radians = -math.radians(self.angle)
            # quaternion = tf.transformations.quaternion_from_euler(increment_roll, pitch, yaw)
        # Looks left or towards top
        elif orient_x == 2:
            theta_radians = math.radians(self.angle)
            # print("Quaternion is: ",quaternion)
        elif orient_x == 0:
            return

        # For orient_y uncomment the following
        q_z_rotation = tf.transformations.quaternion_about_axis(theta_radians, (0, 1, 0))
        # For orient_x uncomment the following
        # q_z_rotation = tf.transformations.quaternion_about_axis(theta_radians, (1, 0, 0))
        cur_pose = self.franka_pose_base_link()
        q_current = [cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z, cur_pose.pose.orientation.w]
        q_new = tf.transformations.quaternion_multiply(q_current, q_z_rotation)
        self.rotate_gripper(q_new)


    def save_image(self, pos, raw=True, aruco=False, second_cam=False):
        # Saving the current image after moving to desired position
        if raw and int(pos)==0:
            name = "saved_image_{}.jpg".format(pos)
            # When count == 1, we get the intial overhead view that goes into context always
            # if self.count == 0:
            #     cv2.imwrite(image_save_path + "/trial_{}/saved_image_{}.jpg" .format(trial, pos), self.cv_image)
            # elif self.count == 1 :     
            #     cv2.imwrite(image_save_path + "/trial_{}/saved_image_overhead.jpg" .format(trial, pos), self.cv_image)
            # else:
            cv2.imwrite(image_save_path + "/trial_{}/saved_image_{}.jpg" .format(trial, pos), self.cv_image)
            cv2.imwrite(image_save_path + "/trial_{}/saved_image.jpg" .format(trial), self.cv_image)

            self.memory_all[self.count]["image_path"] = "{}/trial_{}/saved_image.jpg".format(image_save_path, trial)
            # print("Saving {} to {}".format(name, image_save_path))
            return name

        elif aruco and int(pos)==0:
            name = "grid_image_{}.jpg".format(pos)
            # if self.count == 0:  
            #     cv2.imwrite(image_save_path + "/trial_{}/grid_image_{}.jpg" .format(trial, pos), self.aruco_image)
            # elif self.count == 1: 
            #     cv2.imwrite(image_save_path + "/trial_{}/grid_image_overhead.jpg" .format(trial, pos), self.aruco_image)                
            # else:
            cv2.imwrite(image_save_path + "/trial_{}/grid_image_{}.jpg" .format(trial, pos), self.aruco_image)
            cv2.imwrite(image_save_path + "/trial_{}/grid_image.jpg" .format(trial), self.aruco_image)
            # print("Saving grid {} to {}/trial_{}".format(name, image_save_path, trial))
            return name
        
        elif second_cam:
            name = "second_cam_rgb_{}.jpg".format(pos)
            cv2.imwrite(image_save_path + "/trial_{}/second_cam_rgb_{}.jpg" .format(trial, pos), self.cv_image)
            cv2.imwrite(image_save_path + "/trial_{}/second_cam_rgb.jpg" .format(trial), self.cv_image)
            # print("Saving grid {} to {}/trial_{}".format(name, image_save_path, trial))
            self.memory_all[self.count]["second_image_path"] = "{}/trial_{}/second_cam_rgb_{}.jpg".format(image_save_path, trial, str(self.count))
            return name

    def get_orient_value(self):
        with open(result_path, 'r') as file:
            for line in file:
                # Check if the line starts with "orient_x:"
                if line.strip().startswith("orient_x:"):
                    # Split the line by ':' and get the value after the colon
                    orient_x_value = line.split(":")
                    orient_x_value = orient_x_value[1].strip()
                    try:
                        orient_x_value = int(orient_x_value)
                        return orient_x_value   
                    except ValueError:
                        return 0

            return 0     

    def get_vlm_coords(self):

        summary_lines = []
        with open(result_path, 'r') as file:
            lines = file.readlines()

            inside_summary = False

            # Loop through each line to find and store the Summary section
            for line in lines:
                # Check for the beginning of the "Summary" section
                if "output" in line.lower():
                    inside_summary = True

                if inside_summary:
                    summary_lines.append(line.strip())  # Strip leading/trailing spaces
            

            # Join the summary lines into a single output line
            summary = " ".join(summary_lines)
            arr = summary.split("(")

            for i in range(len(arr)):
                if not len(arr[i]) == 0:
                    try:
                        coords = int(arr[i][0])
                        coords=  arr[i]
                        if coords[1] == ".":
                            coords_x = coords[0:3]
                            coords_y = coords[4:7]
                            if coords[3] == ",":
                                coords_z = coords[8:9]
                                #roll = coords[10:11]
                            else:
                                coords_z = coords[8:11]
                                #roll = coords[12:13]
                            return float(coords_x), float(coords_y), float(coords_z)#, int(roll)
                        else:
                            coords_x = coords[0:1]
                            coords_y = coords[2:3]
                            coords_z  = coords[4:5]
                            roll = coords[6:7]
                            return int(coords_x), int(coords_y), int(coords_z)#, int(roll)
                        
                        # print("array value is at i is ", i, coords)
                    except ValueError:
                        # print("Passing at i", i)
                        pass
            # if arr[1][0] == "s":
            #     print(arr[1])
            #     print("Stopping. Question answered!")
            #     self.run_pipeline = False

    def get_vlm_coords_new(self, phrase, alternate_phrase):
        out = self.get_result(phrase, alternate_phrase)
        if out:
            out = out.replace("(","").replace(")","")
            out = out.split(",")
            return [float(out[0]), float(out[1]), float(out[2])]
        else:
            return ""

    def get_target_object(self):
        # Open the file and read line by line
        with open(gpt_analyser_path, 'r') as file:
            for line in file:
                # Check if the line starts with "orient_x:"
                if line.strip().startswith("Target") or line.strip().startswith("- Target"):
                    # Split the line by ':' and get the value after the colon
                    target = line.split(":")
                    target = target[1].strip()
                    try:
                        target = str(target)
                        return target   
                    except ValueError:
                        print("No string output from Perception Analyser")
                        return 0
            return 0
        
    def get_perception_analyser_output(self):  
        summary_lines = []
        coordinate_pose = []

        with open(gpt_analyser_path, 'r') as file:
            lines = file.readlines()

            inside_summary = False

            # Loop through each line to find and store the Summary section
            for line in lines:
                # Check for the beginning of the "Summary" section
                if "output" in line.lower():
                    inside_summary = True

                if inside_summary:
                    summary_lines.append(line.strip())  # Strip leading/trailing spaces
                

            # Join the summary lines into a single output line
            summary = " ".join(summary_lines)
            arr = summary.split(":")

            for i in range(len(arr)):
                if not len(arr[i]) == 0:
                    try:
                        coords=  arr[i]
                        # print(coords)
                        if coords[0].lower() == "s":
                            return True
                        elif coords[0].lower() == "g":
                            return False
                        
                    except ValueError:
                        pass

    def get_perception_action(self, phrase):
        # Open the file and read line by line
        with open(gpt_analyser_path, 'r') as file:
            for line in file:
                # Check if the line starts with "orient_x:"
                if line.strip().startswith(phrase):
                    # Split the line by ':' and get the value after the colon
                    target = line.split(":")
                    if (not str(phrase) == "- Analysis of action") and (not str(phrase) == "- Thought"):
                        
                        target = target[1].replace(" ","")
                        target = target.strip()
                        target = target.lower()
                    # print("{} output from Perception Analyser is {}".format(phrase, target))
                    try:
                        target = str(target)
                        return target   
                    except ValueError:
                        print("Error from perception analyser output action suggestion")
                        return 0
            return 0

    def get_result(self, point, alternate_point=None):
        # Open the file and read line by line
        with open(result_path, 'r') as file:
            for line in file:
                # Check if the line starts with "orient_x:"
                if line.strip().startswith(point) or line.strip().startswith(alternate_point):
                    # Split the line by ':' and get the value after the colon
                    target = line.split(":")
                    target = target[1].strip()
                    # print("Keypoint for {} is: {}".format(point, target))
                    try:
                        target = str(target)
                        return target   
                    except ValueError:
                        print("Error")
                        return 0
            return 0
            

    def write_current_state(self, coord_x, coord_y, coord_z, orient_x):
        with open(current_position_path, "a") as current_pos_file:
            position_string = "Step_{} output by GPT-4o: ".format(self.count-2)
            current_pos_file.write(str(position_string) + "(" + str(coord_x) + ", " + str(coord_y) + ", " + str(coord_z) + ", " + str(orient_x) + ")\n")

    def AP_full_view(self):
        self.recover_error()

        plan = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if use_franka_robot:
            cur_pose = self.franka_pose_base_link()
            self.home_z = cur_pose.pose.position.z

    def get_xyz_move(self, coord_x, coord_y, coord_z, orient_x):
        # print("In get xyz_move() coord x,y,z are: ", coord_x, coord_y, coord_z)
        
        if type(coord_x) == float or type(coord_x) == int:
            if coord_x >= numDots-1:
                coord_x = coord_x-1.1
            coords_x = self.get_precise(float(coord_x), x_coord=True)
            
            if coord_y >= numDots-1:
                coord_y = coord_y-1.1
            coords_y = self.get_precise(float(coord_y), x_coord=False)
            
            print("Coordinates ({},{}) from grid frame correspond to ({},{}) and z is {} ".format(coord_x, coord_y, coords_x, coords_y, self.PoseMatrix_z[coord_z, 0]))              
            
            # self.go_to_position(coords_x, coords_y, self.PoseMatrix_z[coords_z, 0])
            # self.go_to_position(coords_x, coords_y, coords_z)
            self.moveit_position_loop(coords_x, coords_y, coord_z, orient_x, precise=True)
        else:
            print("The mapped Keypoints to 2D grid points need to be int or floats")

    def moveit_position_loop(self, coords_x, coords_y, coords_z, orient_x, precise):
        if precise:
            self.go_to_position(coords_x, coords_y, self.PoseMatrix_z[coords_z, 0], orient_x)
            if self.moveit_reached_msg.status_list[0].status != int(3):
                # print("Movegorup stratus: ", self.moveit_reached_msg.status_list[0].status)
                self.moveit_position_loop(coords_x, coords_y, 1, orient_x, precise)
        else:
            self.go_to_position(self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0], orient_x)

            if self.moveit_reached_msg.status_list[0].status != int(3):
                # print("Movegorup stratus: ", self.moveit_reached_msg.status_list[0].status)
                self.moveit_position_loop(coords_x, coords_y, 1, orient_x, precise)
    
    def split_pose_matrix(self, split_start, split_end, desired_digit):
        length = split_end - split_start
        interval = length / 10  # 10 parts between 2,2 and 2,3. 2nd part will be 2.2
        val = split_start + desired_digit * interval
        return val
        
    def get_precise(self, coords, x_coord):
        if int(coords) >= 10:
            decimal = 3
        else:
            decimal = 2
        coords = str(coords)

        if not x_coord:
            if int(coords[decimal]) < 5:
                coord_lower = int(self.round_half_up(float(coords)))
                coord_upper = coord_lower + 1
                coord_val = int(float(coords))
                coords_val = self.split_pose_matrix(self.PoseMatrix_y[coord_val, coord_lower], self.PoseMatrix_y[coord_val, coord_upper], int(coords[decimal])) # This will be 2,2 amd 2,3
            elif int(coords[decimal]) >= 5:
                coord_upper = int(self.round_half_up(float(coords)))
                coord_lower = coord_upper - 1
                coord_val = int(float(coords))
                coords_val = self.split_pose_matrix(self.PoseMatrix_y[coord_val, coord_lower], self.PoseMatrix_y[coord_val, coord_upper], int(coords[decimal])) # This will be 2,2 amd 2,3
        
        elif x_coord:
            if int(coords[decimal]) < 5:
                coord_lower = int(self.round_half_up(float(coords)))
                coord_upper = coord_lower + 1
                coord_val = int(float(coords))
                coords_val = self.split_pose_matrix(self.PoseMatrix_x[coord_lower, coord_val], self.PoseMatrix_x[coord_upper, coord_val], int(coords[decimal])) # This will be 2,2 amd 2,3
            elif int(coords[decimal]) >= 5:
                coord_upper = int(self.round_half_up(float(coords)))
                coord_lower = coord_upper - 1
                coord_val = int(float(coords))
                coords_val = self.split_pose_matrix(self.PoseMatrix_x[coord_lower, coord_val], self.PoseMatrix_x[coord_upper, coord_val], int(coords[decimal])) # This will be 2,2 amd 2,3

        return coords_val

    def get_total_dots(self, coord):
        if int(coord) >= 10:
            decimal = 3
        else:
            decimal = 2

        coord = str(coord)
        if int(coord[decimal]) < 5:
            lower = int(self.round_half_up(float(coord)))
            val = lower * divide + divide*float("0.{}".format(coord[decimal]))
        elif int(coord[decimal]) >= 5:
            upper = int(self.round_half_up(float(coord)))
            val = upper * divide - divide*float("0.{}".format(coord[decimal]))

        return val
    
    
    
    def pipeline(self):       

        if not self.saved_new_grid:
            self.save_grid_image()

            # Creating memory component
            if not self.count in self.memory_all.keys():
                self.memory_all[self.count] = copy.deepcopy(memory) 

        elif self.saved_new_grid: 
            with open(local_mem_path + "dict.pkl", "wb") as f:
                pickle.dump(self.memory_all, f)
                pdb.set_trace()

            # self.get_xyz_move(3.0,2.5,0,1)
            # pdb.set_trace()
            # GPT-4o callback
            print("")
            print("-------- Calling Perception Analyser - PA --------------")
            if use_gpt:
                os.system('python3 /home/venkatesh/franka_ip_ws/src/franka_ros/franka_control/scripts/AP_VLM/AP_call_perception_analyser.py')
            elif use_gemini:
                os.system('rosrun franka_control run_gemini_PA.sh')
            elif use_claude:
                os.system('python3 /home/venkatesh/franka_ip_ws/src/franka_ros/franka_control/scripts/AP_VLM/claude_PA.py')

            PA_output = self.get_perception_analyser_output()
            # print("PA output is: ", PA_output)
                
            if PA_output:
                print("Stopping as I got a conclusive answer")
                self.run_pipeline = False
                return
            elif not PA_output:
                # GPT-4o callback
                print("")
                print("[bold red]-------- Performing Active Perception - APP --------------[/bold red]")
                if use_gpt:
                    os.system('python3 /home/venkatesh/franka_ip_ws/src/franka_ros/franka_control/scripts/AP_VLM/AP_call_vlm.py')
                elif use_gemini:
                    os.system('rosrun franka_control run_gemini_call_VLM.sh')
                elif use_claude:
                    os.system('python3 /home/venkatesh/franka_ip_ws/src/franka_ros/franka_control/scripts/AP_VLM/claude_call_VLM.py')

                if self.get_vlm_coords_new("- Output:", "- **Output:**"):
                    policy_output = self.get_vlm_coords_new("- Output:", "- **Output:**")
                else:
                    policy_output = self.get_vlm_coords_new("Output:", "**Output:**")

                coord_x, coord_y, coord_z = policy_output[0], policy_output[1], policy_output[2]#self.get_vlm_coords()
                orient_x = self.get_orient_value()
            
            if type(coord_x) == str:
                print("Stopping")
                self.run_pipeline = False
            else:
            # elif type(coord_x) == float:
            #     if coord_y == float(3.0):
            #         coord_y = float(2.9)
            #     if coord_x == float(3.0):
            #         coord_x = float(2.9)
            #     if coord_x == float(2.5) and coord_y == float(2.5) and coord_z == float(2.0) and orient_x == int(2):
            #         coord_x = float(2.5)
            #         coord_y = float(2.0)
            #         coord_z = int(1)

                # mapped_coord_x = self.get_total_dots(coord_x)
                # mapped_coord_y = self.get_total_dots(coord_y)
                mapped_coord_x = float(coord_x)
                mapped_coord_y = float(coord_y)
                coords_z = int(coord_z)
                orient_x = int(orient_x) 

                # if orient_x == 1:
                #     print("Going to ({},{},{}) and looking up" .format(coord_x, coord_y, coord_z))
                # elif orient_x == 2:
                #     print("")
                #     print("Going to ({},{},{}) and looking down" .format(coord_x, coord_y, coord_z))
                # elif orient_x == 0:
                #     print("Going to ({},{},{})" .format(coord_x, coord_y, coord_z))

                
                # print("It corresponds to ", precise_coords_x, precise_coords_y, self.PoseMatrix_z[coords_z, 0])
                
                self.predefined_postions("center")

                self.get_xyz_move(mapped_coord_x, mapped_coord_y, coords_z, orient_x)
                # self.moveit_position_loop(precise_coords_x, precise_coords_y, coords_z, orient_x, precise=True)

                target_obj = self.get_target_object()
                thought = self.get_perception_action("- Thought:")

                policy_reasoning = self.get_result("- Action Output:", "- **Action Output:**")

                second_cam_rgb = self.save_image(self.count, raw=False, aruco=False, second_cam=True)
                
                self.memory_all[self.count]["target_object"] = target_obj
                self.memory_all[self.count]["perception_thought"]  = thought
                self.memory_all[self.count]["orientation_x"]  = orient_x
                self.memory_all[self.count]["output_coords"]  = policy_output
                self.memory_all[self.count]["policy_reason"]  = policy_reasoning           

            self.count = self.count + 1
            self.write_current_state(coord_x, coord_y, coord_z, orient_x)
            self.saved_new_grid = False
            self.start_time = None
        
        else:
            self.AP_full_view()
            self.saved_new_grid = False
            self.start_time = None
        
    
    def predefined_postions(self, pos):
        # Checked by Venkatesh
        
        if pos == "bottom_left":
            self.go_to_position(0.3, 0.7, 0.4, 0)      
        elif pos == "bottom_right":
            self.go_to_position(-0.3, 0.7, 0.4, 0)
        elif pos == "center":
            self.go_to_position(0.01, 0.4, 0.4, 0)
        elif pos == "top_left":
            self.go_to_position(0.3, 0.3, 0.4, 0)
        elif pos == "top_right":
            self.go_to_position(-0.3, 0.3, 0.5, 0)
        elif pos == "full_view":
            self.go_to_position(0.01, 0.45, 0.7, 0)
        elif pos == "custom":
            position_x = input("Enter the x position to go to")
            position_y = input("Enter the y position to go to")
            position_z = input("Enter the z position to go to")
            self.go_to_position(position_x, position_y, position_z, 0)


    def control_loop(self):
        while not rospy.is_shutdown():
            if self.run_pipeline:
                self.pipeline()  
            else:
                break


def main():
    rospy.init_node('broadcast_marker', anonymous=True)

    sm = SimpleMoveit()
    # sm.predefined_postions("top_right")   # Go to marker near base of robot
    sm.AP_full_view()
    sm.control_loop()
    
if __name__ == "__main__":
    # while not rospy.is_shutdown():
    # detect = DetectToMotion()
    scene = config.scene
    trial = config.trial
    scene_name = config.scene_name

    use_gpt = config.use_gpt
    use_gemini = config.use_gemini
    use_claude = config.use_claude

    numDots = 6
    use_franka_robot = True
    numDots_visualise = 6
    divide = numDots/(numDots_visualise-1)

    if use_franka_robot:
        group_name = "panda_arm"
        joint_goal = [-0.06154114919900649, -0.8294276117609257, -0.0842066666462831, -1.9992733160628395, -0.04568747991654608, 1.2866662808633769, 0.7166154399505457]
        second_cam_joint_goal = [0.12901441150556425, 0.015470819364514266, -0.16335267397813627, -2.337445184607255, -0.04538340679647799, 3.4267693100839822, 0.7957336327212887]
        # second_cam_joint_goal = [0.11506829440902666, -0.6945936535447347, -0.2962877743077195, -2.7382143919008812, -0.21470820336871677, 2.7731193219290864, 0.8344193054524561]
        z_camera_hand = 0.0308057   # z value obtained from camera to panda eye-in-hand calbration
        from franka_control.srv import HeatmapService
        from franka_msgs.msg import FrankaState, ErrorRecoveryAction, ErrorRecoveryActionGoal
        from franka_gripper.msg import MoveAction, MoveGoal, HomingAction, HomingGoal
    else:
        group_name = "manipulator"
        joint_goal = [-1.4043696562396448, -1.3513196150409144, -0.8806141058551233, 3.9225025177001953, -4.713055197392599, 3.307312488555908]
        from ur_robot_driver.srv import HeatmapService
    
    image_save_path = config.image_save_path
    video_save_path = config.video_save_path
    result_path = config.result_path
    gpt_analyser_path = config.gpt_analyser_path

    current_position_path = config.current_position_path
    target_state_path = config.target_state_path
    # result_path = "/home/ur_5/Documents/gpt_tests/Test_4_9_24/test_24/gpt_result_24.txt"
    local_mem_path = config.memory_dictionary_path

    # New Directory to log results
    test_directory = config.test_directory
    path = os.path.join(image_save_path, test_directory)
    os.mkdir(path)

    memory = {"action": 
                {"grasp": 
                    {"contact": [], 
                     "place": [], 
                     "curr_sequence":[],
                     "all_sequences":[],
                     }, 
                "push":{
                    "pre_contact": "",
                    "contact": "",
                    "post_contact":""
                    }
                },
            "goal_object": "",
            "target_object": "",
            "orientation_x": "",
            "orientation_y": "",
            "summary":[],
            "perception_thought": "",
            "policy_reason": "",
            "image_path": "",
            "second_image_path": "",
            "image": {"path": [],
                        "array": [],
                        },
            "output_coords": ""
            }


    main()
