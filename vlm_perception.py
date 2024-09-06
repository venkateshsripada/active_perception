#!/usr/bin/env python

import os
import sys
import pdb
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

        self.velocity_scale = 0.15
        
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("detect_marker_motion")
        # self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group.set_max_velocity_scaling_factor(self.velocity_scale)

        self.listener = tf.TransformListener()

        self.saved_new_grid = False
        self.got_vertices_mapping = False
        self.start_time = None

        self.run_pipeline = True

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
        

    def go_to_position(self, x_pos, y_pos, z_pos, orient_x):

        pose_goal = Pose()
        pose_goal.orientation.x = -0.023273475657969504
        pose_goal.orientation.y = 0.9995675584715471
        pose_goal.orientation.z = 0.017924180374535442
        pose_goal.orientation.w = 0.001328585687693568
        pose_goal.position.x = x_pos
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos

        self.listener.waitForTransform("base_link", "tool0",rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform("base_link", "tool0", rospy.Time())
        print("Currently at", trans)
        print("----------------------------------------")

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        change = (np.pi * self.angle)/180
        increment = roll - change
        decrement = roll + change

        if orient_x == 1:
            quaternion = tf.transformations.quaternion_from_euler(increment, pitch, yaw)
        elif orient_x == 2:
            quaternion = tf.transformations.quaternion_from_euler(decrement, pitch, yaw)
        elif orient_x == 0:
            return

        pose_goal = Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = x_pos
        pose_goal.position.y = y_pos
        pose_goal.position.z = z_pos

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()



    def save_image(self, pos, raw=True, aruco=False):
        # Saving the current image after moving to desired position
        if raw:
            name = "saved_image_{}.jpg" .format(pos)
            # When count == 1, we get the intial overhead view that goes into context always
            if self.count == 1 :     
                cv2.imwrite(image_save_path + "/test_{}/saved_image_overhead.jpg" .format(trial, pos), self.cv_image)
            else:
                cv2.imwrite(image_save_path + "/test_{}/saved_image_{}.jpg" .format(trial, pos), self.cv_image)
            print("Saving {} to {}".format(name, image_save_path))
            return name

        elif aruco:
            name = "grid_image_{}.jpg" .format(pos)
            if self.count == 0:  
                cv2.imwrite(image_save_path + "/test_{}/grid_image_{}.jpg" .format(trial, pos), self.aruco_image)
            elif self.count == 1: 
                cv2.imwrite(image_save_path + "/test_{}/grid_image_overhead.jpg" .format(trial, pos), self.aruco_image)                
            else:
                cv2.imwrite(image_save_path + "/test_{}/grid_image_{}.jpg" .format(trial, pos), self.aruco_image)
                cv2.imwrite(image_save_path + "/test_{}/grid_image.jpg" .format(trial), self.aruco_image)
            print("Saving grid {} to {}/test_{}".format(name, image_save_path, trial))
            return name



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
                            if coords[9] == ",":
                                coords_z = coords[8:9]
                                roll = coords[10:11]
                            else:
                                coords_z = coords[8:11]
                                roll = coords[12:13]
                            return float(coords_x), float(coords_y), float(coords_z), int(roll)
                        else:
                            coords_x = coords[0:1]
                            coords_y = coords[2:3]
                            coords_z  = coords[4:5]
                            roll = coords[6:7]
                            return int(coords_x), int(coords_y), int(coords_z), int(roll)
                        
                        # print("array value is at i is ", i, coords)
                    except ValueError:
                        # print("Passing at i", i)
                        pass
            # if arr[1][0] == "s":
            #     print(arr[1])
            #     print("Stopping. Question answered!")
            #     self.run_pipeline = False

        
    def get_init_target_state(self):  
        summary_lines = []
        coordinate_pose = []

        with open(result_path, 'r') as file:
            lines = file.readlines()

            inside_summary = False

                # Loop through each line to find and store the Summary section
            for line in lines:
                # Check for the beginning of the "Summary" section
                if "vertex_x" in line.lower():
                    inside_summary = True

                if inside_summary:
                    summary_lines.append(line.strip())  # Strip leading/trailing spaces
                

            # Join the summary lines into a single output line
            summary = " ".join(summary_lines)
            arr = summary.split(":")

            for i in range(len(arr)):
                if not len(arr[i]) == 0:
                    try:
                        coords = int(arr[i][1])
                        coords=  arr[i]
                        print(coords)
                        if coords[2] == ".":
                            coordinate_pose.append(float(coords[1:4]))
                        else:
                            coordinate_pose.append(int(coords[1]))
                    except ValueError:
                        pass

        return coordinate_pose[0], coordinate_pose[1], coordinate_pose[2], coordinate_pose[3]

    def write_target_state(self):
        vertex_x, vertex_y, vertex_z, orientation_x = self.get_init_target_state()
        with open(target_state_path, "w") as target_pos_file:
            target_pos_file.write("(" + str(vertex_x) + ", " + str(vertex_y) + ", " + str(vertex_z) + ", " + str(orientation_x) + ")\n")

    def write_current_state(self, coord_x, coord_y, coord_z, orient_x):
        with open(current_position_path, "w") as current_pos_file:
            current_pos_file.write("(" + str(coord_x) + ", " + str(coord_y) + ", " + str(coord_z) + ", " + str(orient_x) + ")\n")

    
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

    def moveit_position_loop(self, coords_x, coords_y, coords_z, orient_x, precise):
        if precise:
            self.go_to_position(coords_x, coords_y, self.PoseMatrix_z[coords_z, 0], orient_x)
            if self.moveit_reached_msg.status_list[0].status != int(3):
                print("Movegorup stratus: ", self.moveit_reached_msg.status_list[0].status)
                self.moveit_position_loop(coords_x, coords_y, 1, orient_x, precise)
        else:
            self.go_to_position(self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0], orient_x)

            if self.moveit_reached_msg.status_list[0].status != int(3):
                print("Movegorup stratus: ", self.moveit_reached_msg.status_list[0].status)
                self.moveit_position_loop(coords_x, coords_y, 1, orient_x, precise)
    
    def split_pose_matrix(self, split_start, split_end, desired_digit):
        length = split_end - split_start
        interval = length / 10  # 10 parts between 2,2 and 2,3. 2nd part will be 2.2
        val = split_start + desired_digit * interval
        return val
        
    def get_precise(self, coords, x_coord):
        coords = str(coords)

        if not x_coord:
            if int(coords[2]) < 5:
                coord_lower = int(round(float(coords)))
                coord_upper = coord_lower + 1
                coord_val = int(coords[0])
                coords_val = self.split_pose_matrix(self.PoseMatrix_y[coord_val, coord_lower], self.PoseMatrix_y[coord_val, coord_upper], int(coords[2])) # This will be 2,2 amd 2,3
            elif int(coords[2]) >= 5:
                coord_upper = int(round(float(coords)))
                coord_lower = coord_upper - 1
                coord_val = int(coords[0])
                coords_val = self.split_pose_matrix(self.PoseMatrix_y[coord_val, coord_lower], self.PoseMatrix_y[coord_val, coord_upper], int(coords[2])) # This will be 2,2 amd 2,3
        
        elif x_coord:
            if int(coords[2]) < 5:
                coord_lower = int(round(float(coords)))
                # pdb.set_trace()
                coord_upper = coord_lower + 1
                coord_val = int(coords[0])
                coords_val = self.split_pose_matrix(self.PoseMatrix_x[coord_lower, coord_val], self.PoseMatrix_x[coord_upper, coord_val], int(coords[2])) # This will be 2,2 amd 2,3
            elif int(coords[2]) >= 5:
                coord_upper = int(round(float(coords)))
                coord_lower = coord_upper - 1
                coord_val = int(coords[0])
                coords_val = self.split_pose_matrix(self.PoseMatrix_x[coord_lower, coord_val], self.PoseMatrix_x[coord_upper, coord_val], int(coords[2])) # This will be 2,2 amd 2,3

        return coords_val

    def pipeline(self):       

        if not self.saved_new_grid:
            self.save_grid_image() 

        elif self.saved_new_grid and self.count >= 2: 
            self.count = self.count + 1
            print("Count is: ", self.count)

            # GPT-4o callback
            os.system('python3 /home/ur_5/ur5_ws/src/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/call_vlm.py')

            if self.count == 3:
                self.write_target_state()   # Should get GPT-4o output for analyser prompt
                coord_x, coord_y, coord_z, orient_x = self.get_init_target_state()
                
            if self.count > 3:
                coord_x, coord_y, coord_z, orient_x = self.get_vlm_coords()
            
            if type(coord_x) == str:
                print("Stopping")
                self.run_pipeline = False
            elif type(coord_x) == float:
                precise_coords_x = self.get_precise(coord_x, x_coord=True)
                precise_coords_y = self.get_precise(coord_y, x_coord=False)
                coords_z = int(coord_z)
                orient_x = int(orient_x) 

                if orient_x == 1:
                    print("Going to ({},{},{}) and looking up" .format(coord_x, coord_y, coord_z))
                elif orient_x == 2:
                    print("Going to ({},{},{}) and looking down" .format(coord_x, coord_y, coord_z))
                elif orient_x == 0:
                    print("Going to ({},{},{})" .format(coord_x, coord_y, coord_z))
                print("It corresponds to ", precise_coords_x, precise_coords_y, self.PoseMatrix_z[coords_z, 0])
                
                self.predefined_postions("center")
                self.moveit_position_loop(precise_coords_x, precise_coords_y, coords_z, orient_x, precise=True)
            else:
                coords_x = int(coord_x)
                coords_y = int(coord_y)
                coords_z = int(coord_z)
                orient_x = int(orient_x)   
            
                if orient_x == 1:
                    print("Going to ({},{},{}) and looking up" .format(coords_x, coords_y, coords_z))
                elif orient_x == 2:
                    print("Going to ({},{},{}) and looking down" .format(coords_x, coords_y, coords_z))
                elif orient_x == 0:
                    print("Going to ({},{},{})" .format(coords_x, coords_y, coords_z))

                print("It corresponds to ", self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0])
                self.predefined_postions("center")
                self.moveit_position_loop(coords_x, coords_y, coords_z, orient_x, precise=False)
            # self.go_to_position(self.PoseMatrix_x[coords_x, coords_y], self.PoseMatrix_y[coords_x, coords_y], self.PoseMatrix_z[coords_z, 0])               

            self.write_current_state(coord_x, coord_y, coord_z, orient_x)
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
            self.go_to_position(0.3, 0.7, 0.4, 0)      
        elif pos == "bottom_right":
            self.go_to_position(-0.3, 0.7, 0.4, 0)
        elif pos == "center":
            self.go_to_position(0.01, 0.5, 0.4, 0)
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
    trial = 11
    # image_path = "/home/ur_5/ur5_ws/"
    image_save_path = "/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt"
    result_path = "/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{}/gpt_result_{}.txt".format(trial, trial)
    
    current_position_path = "/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{}/current_position_{}.txt".format(trial, trial)
    target_state_path = "/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{}/target_position_{}.txt".format(trial, trial)
    # result_path = "/home/ur_5/Documents/gpt_tests/Test_4_9_24/test_24/gpt_result_24.txt"

    # New Directory to log results
    test_directory = "test_{}".format(trial)
    path = os.path.join(image_save_path, test_directory)
    os.mkdir(path)
    
    
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
