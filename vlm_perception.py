#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import moveit_commander
import moveit_msgs.msg

# import openai
# import call_vlm
import base64
# from openai import OpenAI
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
        self.moveit_reached_sub = rospy.Subscriber("/move_group/status", GoalStatusArray, self.moveit_reached_callback)

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("detect_marker_motion")
        # self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.listener = tf.TransformListener()

        self.saved_new_grid = False
        self.got_vertices_mapping = False
        self.start_time = None

        self.run_pipeline = True

        self.count = 0

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

    def moveit_reached_callback(self, msg):
        self.moveit_reached_msg = msg

    def save_grid_image(self):
        try:
            if (self.grid_visualised.data):
                
                # Check if the 3D grid has been visible for 3 seconds continuously. 
                # This check is important as the cube keeps flickering if the marker is not correctly visible
                if self.start_time is None:
                    self.start_time = time.time()
                
                elif time.time() - self.start_time >= 2:
                    print("Grid is visualised ")
                    if not self.got_vertices_mapping:
                        self.get_vertices_pose()
                    
                    pos = str(self.count)
                    # Save raw image
                    self.raw_image_name = self.save_image(pos, raw=True, aruco=False)
                    # Check if grid is formed and save grid image
                    self.aruco_image_name = self.save_image(pos, raw=False, aruco=True)

                    self.count = self.count + 1
                    self.saved_new_grid = True
            else:
                self.start_time = None
                                    
        except AttributeError:
            # print("Grid not visualised")
            self.start_time = None

    def get_vertices_pose(self):
        try:
            self.PoseMatrix_x = np.array(self.matrix_x).reshape(numDots, numDots)
            self.PoseMatrix_y = np.array(self.matrix_y).reshape(numDots, numDots)
            self.PoseMatrix_z = np.array(self.matrix_z).reshape(numDots, 1)
            self.got_vertices_mapping = True
            # print(self.PoseMatrix_x)
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
        

    def go_to_position(self, x_pos, y_pos, z_pos):

        pose_goal = Pose()
        pose_goal.orientation.x = -0.023273475657969504
        pose_goal.orientation.y = 0.9995675584715471
        pose_goal.orientation.z = 0.017924180374535442
        pose_goal.orientation.w = 0.001328585687693568
        pose_goal.position.x = x_pos
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.trans, self.rot = self.pose_ee_link()


    def save_image(self, pos, raw=True, aruco=False):
        # Saving the current image after moving to desired position
        if raw:
            name = "saved_image_{}.jpg" .format(pos)
            cv2.imwrite("/home/ur_5/ur5_ws/saved_images/saved_image_{}.jpg" .format(pos), self.cv_image)
            print("Saving {} to {}".format(name, image_path))
            return name
        elif aruco:
            name = "grid_image_{}.jpg" .format(pos)
            cv2.imwrite("/home/ur_5/ur5_ws/saved_images/grid_image_{}.jpg" .format(pos), self.aruco_image)
            cv2.imwrite("grid_image.jpg", self.aruco_image)
            print("Saving grid {} to {}".format(name, image_path))
            return name


    def pose_ee_link(self):
        # print("Starting transformation")
        # transformed_marker_pose = self.transform_pose(self.aruco_msg, "camera_color_optical_frame", "base_link")
        self.listener.waitForTransform("base_link", "tool0",rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("base_link", "tool0", rospy.Time())
        # print("Transformed marker pose: ", trans)
        # print("Rotation is: ", rot)
        return trans, rot

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
                        coords_x = coords[0:1]
                        coords_y = coords[2:3]
                        coords_z  = coords[4:5]

                        return coords_x, coords_y, coords_z
                        # print("array value is at i is ", i, coords)
                    except ValueError:
                        # print("Passing at i", i)
                        pass
        # return "s","s","s"
            
    
    def AP_full_view(self):
        pose_goal = Pose()
        pose_goal.orientation.x = 0.040209717276491856
        pose_goal.orientation.y = -0.9967552503108692
        pose_goal.orientation.z =  -0.0663499022260687
        pose_goal.orientation.w = -0.021443882317635124
        pose_goal.position.x = -0.10604477511715267
        pose_goal.position.y =  0.3252246343845232
        pose_goal.position.z = 0.7823752459856232

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def moveit_position_loop(self, coords_x, coords_y, coords_z):
        self.go_to_position(self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0])
        if self.moveit_reached_msg.status_list[0].status != int(3):
            self.moveit_position_loop()
    
    
    def pipeline(self):       

        if not self.saved_new_grid:
            self.save_grid_image() 

        elif self.saved_new_grid and self.count >= 2: 
            self.count = self.count + 1
            # GPT-4o callback
            os.system('python3 /home/ur_5/ur5_ws/src/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/call_vlm.py')
            coord_x, coord_y, coord_z = self.get_vlm_coords()          
            # self.AP_full_view()     # Manually taking it to new position. TO be done by GPT 
            
        
            if coord_x == "s":
                print("Stopping")
                self.run_pipeline = False
            else:
                coords_x = int(coord_x)
                coords_y = int(coord_y)
                coords_z = int(coord_z)   
            
                print("Going to ({},{},{})" .format(coords_x, coords_y, coords_z))
                print("It corresponds to ", self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0])
                self.predefined_postions("center")
                self.moveit_position_loop(coords_x, coords_y, coords_z)
                # self.go_to_position(self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0])
                
                transl, rot = self.pose_ee_link()
                print("Currently at", transl)
                print("----------------------------------------")
            
                curr_trans, curr_rot = self.pose_ee_link()
                self.saved_new_grid = False
                self.start_time = None
        
        else:
            self.AP_full_view()
            self.saved_new_grid = False
            self.start_time = None
        # if abs(curr_trans[0] - self.trans[0]) > 1e-3: 
        #     print("Robot moved!")
        # else:
        #     self.run_pipeline = False
        


    
    def predefined_postions(self, pos):
        # Checked by Venkatesh
        
        if pos == "bottom_left":
            self.go_to_position(0.3, 0.7, 0.4)      
        elif pos == "bottom_right":
            self.go_to_position(-0.3, 0.7, 0.4)
        elif pos == "center":
            self.go_to_position(0.01, 0.5, 0.4)
        elif pos == "top_left":
            self.go_to_position(0.3, 0.3, 0.4)
        elif pos == "top_right":
            self.go_to_position(-0.3, 0.3, 0.5)
        elif pos == "full_view":
            self.go_to_position(0.01, 0.45, 0.7)
        elif pos == "custom":
            position_x = input("Enter the x position to go to")
            position_y = input("Enter the y position to go to")
            position_z = input("Enter the z position to go to")
            self.go_to_position(position_x, position_y, position_z)

        # self.print_stuff()
        # self.name = self.save_image(pos)
        # self.divide_grids(image_path, 3, 3)

        # GPT-4o callback
        # os.system('python3 /home/ur_5/ur5_ws/src/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/call_vlm.py')
        # coord_x, coord_y, coord_z = self.get_vlm_coords()
        # coord_x = float(coord_x)
        # coord_y = float(coord_y)
        # coord_z = float(coord_z)

        # if coord_x == 0.0:
        #     coord_x = 0.01
        # self.go_to_position(coord_x, coord_y, coord_z)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.run_pipeline:
                self.pipeline()  
            else:
                break


def main():
    rospy.init_node('broadcast_marker', anonymous=True)

    sm = SimpleMoveit()
    # divide_grids(image_path, 3,3)
    sm.predefined_postions("top_right")   # Go to marker near base of robot
    sm.control_loop()
    
    

if __name__ == "__main__":
    # while not rospy.is_shutdown():
    # detect = DetectToMotion()

    numDots = 4
    trial = 5
    image_path = "/home/ur_5/ur5_ws/"
    result_path = "/home/ur_5/Documents/gpt_tests/Test_29_8_4/gpt_result_{}.txt".format(trial)
    
    
    model = "gpt-4o"
    prompt = """
    
    You are an intelligent agent that can generate voxelized poses (x,y,z,theta, phi) given an input image and an input text question. The camera taking the image is attached to an end effector of a 6DOF robotic arm.

    The input image consists of an RGB image of a tabletop scene overlaid with grids in green. The grids have (x,y) values in blue that denote the (height, width) of the image. The z value is not seen in the image as it is fixed at 0.5. For rotation values (theta, phi), each grid cell along the x and y are incremented by 10 degrees. 
    For example (x,y) = (0.3,0.3) in image corresponds to (x,y,z,theta, phi) = (0.3, 0.3, 0.6, 10, 10), 
    (0.0, 0.3) in image corresponds to (x,y,z,theta, phi) = (0.01, 0.3, 0.6, 20, 10)
    (-0.3, 0.3) in image corresponds to (x,y,z,theta, phi) = (-0.3, 0.3, 0.6, 30, 10)
    (0.3, 0.5) in image corresponds to (x,y,z,theta, phi) = (0.3, 0.5, 0.6, 10, 20)
    (0.3, 0.7) in image corresponds to (x,y,z,theta, phi) = (0.3, 0.7, 0.6, 10, 30)
    and so on ...

    Remember to not give 0.0 for x,y, or z. Instead give me 0.01

    The input text question asks you steps you need to take to move the robot's end effector so that the desired output is produced.

    Hence, as output I want you to think about what the answer to the input question is and give me the pose of the end effector in camera frame i.e (x,y,z, theta, phi). Here are a couple of examples for context

    Input Image: Image with green grids and (x,y) text
    Input text: What is the best way know if <there is something near the book>?
    Output: The best way to know if <there is something near the book>  is to go to <(0.3, 0.3, 0.6, 10, 10)> and look again. This is so that <we get an unoccluded view of the objects near the book>

    Input Image: Image with green grids and (x,y) text
    Input text: Is <there is an object behind the laptop>?
    Output: The best way to know if <there is an object behind the laptop>  is to go to <(0.01, 0.5, 0.6, 20, 20)> and look again. This is so that <we can see objects behind the laptop>

    Input image:
    Input text: How do I analyse the allen keys?
    Output:"""

    main()
