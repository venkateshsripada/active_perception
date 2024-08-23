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
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

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

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("detect_marker_motion")
        # self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

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

    def get_vertices_pose(self):
        try:
            if (self.grid_visualised.data):
                print("Grid is visualised ")
            else:
                print("No  Grid is visualised ")
        except AttributeError:
            print("No Grid is visualised ")
            
    
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


    def save_image(self, pos):
        # Saving the current image after moving to desired position
        name = "saved_image_{}.jpg" .format(pos)
        cv2.imwrite("saved_image_{}.jpg" .format(pos), self.cv_image)
        print("Saving {} to {}".format(name, image_path))
        return name

    def divide_grids(self, image1_path, grids_x, grids_y):

        image = cv2.imread(image1_path + self.name)    

        total_height = image.shape[0]
        total_width = image.shape[1]

        annotate_height_start, annotate_height_end, annotate_width_start, annotate_width_end = self.annotate_vals()

        annotate_y = np.linspace(annotate_height_start, annotate_height_end, grids_x)
        annotate_x = np.linspace(annotate_width_start, annotate_width_end, grids_y)

        # To avoid code breaking in the for loops below, we add a dummy element (0 here) to the array
        annotate_x = np.append(annotate_x, 0)
        annotate_y = np.append(annotate_y, 0)
        print("Will annotate x axis {} without 0".format(annotate_x))
        print("Will annotate y axis {} without 0".format(annotate_y))

        divide_x = total_width // grids_x
        divide_y = total_height// grids_y

        print("Total height of image is {} and width is {}".format(total_height, total_width))

        i = 0
        for y in range(0, total_height, divide_y):
            j = 0
            for x in range(0, total_width, divide_x):
                y1 = y + divide_y
                x1 = x + divide_x

                # Calculate the center of the current grid cell
                center_y = y + divide_y // 2
                center_x = x + divide_x // 2
                
                cv2.rectangle(image, (x, y), (x1, y1), color=(0, 255, 0), thickness=4)

                print_annotate_x = round(annotate_x[i], 2)
                print_annotate_y = round(annotate_y[j], 2)
                j = j + 1

                text = str("({}, {})".format(print_annotate_y, print_annotate_x))

                text_size = cv2.getTextSize(text, font, font_scale, thickness=4)[0]
                text_x = center_x - text_size[0] // 2
                text_y = center_y + text_size[1] // 2

                # Add the text to the image
                cv2.putText(image, text, (text_x, text_y), font, font_scale, (255, 0, 0), 8)
            i = i + 1
        cv2.imwrite("annotated_image_" + self.name, image)

    def generate_3d_grid(self):
        print("Length of aruco image is {}" .format(self.aruco_vis_image.shape))
        self.diff_image = self.aruco_vis_image - self.cv_image
        print("Non zero indices are {}".format(np.nonzero(self.diff_image)))
        non_zero_indices = np.nonzero(self.diff_image)

        offsets = []
        for i in range(-1000, 1000, 200):
            for j in range(-1000, 1000, 200):
                offsets.append(np.array([i,j,0]))

        # print(offsets)
        # offsets = [np.array([200,0,0]), np.array([-200, 0, 0]), np.array([-200, -200, 0]), np.array([-200, 200, 0]), np.array([0, -200, 0])]
        new_image = self.add_non_zero_with_offset(difference_image=self.diff_image, offsets =offsets, overlay_image = self.aruco_vis_image)

        filtered_image = self.filter_yellow(new_image)
        # print("Non-zero element values are {}".format(self.diff_image[non_zero_x_indices, non_zero_y_indices, non_zero_z_indices]))
        print("Shape of new_image is {}".format(new_image.shape))
        cv2.imwrite("raw_image.jpg", self.cv_image)
        cv2.imwrite("raw_aruco_image.jpg", self.aruco_vis_image)
        cv2.imwrite("saved_image_difference.jpg", self.diff_image)
        cv2.imwrite("new.jpg", new_image)
        cv2.imwrite("filtered_image.jpg", filtered_image)
   
    
    def add_non_zero_with_offset(self, difference_image, offsets, overlay_image):
        # Find the indices of non-zero elements
        non_zero_indices = np.nonzero(difference_image)

        # Create a new array of the same shape, initialized with zeros
        new_image = np.zeros_like(difference_image)

        new_image = self.cv_image.copy()
        # Assign the non-zero values to the new array at the corresponding positions
        new_image[non_zero_indices] = difference_image[non_zero_indices]

        # Apply each offset and assign the corresponding values to new_image
        for offset in offsets:
            # Create a copy of the non-zero indices
            copy_indices = np.copy(non_zero_indices)
            
            # Add the offset to the copy_indices and ensure they are within bounds
            for dim in range(3):
                copy_indices[dim] = np.clip(copy_indices[dim] + offset[dim], 0, difference_image.shape[dim] - 1)

            # Assign the values from the copied indices to the new_image
            new_image[tuple(copy_indices)] = overlay_image[non_zero_indices]

        return new_image

    def filter_yellow(self, image):
        # Convert the difference image to the HSV color space
        hsv_diff_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # # Define HSV range for the yellow color
        yellow_lower = np.array([20, 100, 100], np.uint8)
        yellow_upper = np.array([30, 255, 255], np.uint8)

        yellow_mask = cv2.inRange(hsv_diff_image, yellow_lower, yellow_upper)

        # # Invert the combined mask to get the regions to keep
        inverse_mask = cv2.bitwise_not(yellow_mask)

        # # Apply the inverse mask to the aruco_image to remove the blue voxelized grid
        result_image = cv2.bitwise_and(image, image, mask=inverse_mask)

        return result_image
    
    def annotate_vals(self):
        custom_height_start = 0.3
        custom_height_end = -0.3
        custom_width_start  = 0.3
        custom_width_end = 0.7

        return custom_height_start, custom_height_end, custom_width_start, custom_width_end
    
    def get_vlm_coords(self):
        with open("/home/ur_5/ur5_ws/gpt_result.txt") as file:
            arr = file.read()

            coords = arr.split("(") 
            coords = coords[1]
            coords_x = coords[0:3]
            coords_y = coords[5:8]
            coords_z = coords[10:13]
        return coords_x,  coords_y, coords_z

    
    def AP_full_view(self):
        pose_goal = Pose()
        pose_goal.orientation.x = -0.009052924476954417
        pose_goal.orientation.y = 0.992334854485075
        pose_goal.orientation.z = 0.11973647791802301
        pose_goal.orientation.w = 0.02920200315537318
        pose_goal.position.x = -0.09074780727730096
        pose_goal.position.y = 0.25281975634181075
        pose_goal.position.z = 0.8138046553615809

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
    
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
            self.go_to_position(-0.3, 0.3, 0.4)
        elif pos == "full_view":
            self.go_to_position(0.01, 0.45, 0.7)
        elif pos == "custom":
            position_x = input("Enter the x position to go to")
            position_y = input("Enter the y position to go to")
            position_z = input("Enter the z position to go to")
            self.go_to_position(position_x, position_y, position_z)

        # self.print_stuff()
        self.name = self.save_image(pos)
        self.divide_grids(image_path, 3, 3)

        # GPT-4o callback
        os.system('python3 /home/ur_5/ur5_ws/src/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/call_vlm.py')
        coord_x, coord_y, coord_z = self.get_vlm_coords()
        coord_x = float(coord_x)
        coord_y = float(coord_y)
        coord_z = float(coord_z)

        if coord_x == 0.0:
            coord_x = 0.01
        self.go_to_position(coord_x, coord_y, coord_z)


def main():
    rospy.init_node('broadcast_marker', anonymous=True)

    sm = SimpleMoveit()
    # divide_grids(image_path, 3,3)
    while not rospy.is_shutdown():
        
        # sm.predefined_postions("bottom_right")
        
        sm.get_vertices_pose()

if __name__ == "__main__":
    # while not rospy.is_shutdown():
    # detect = DetectToMotion()
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 2
    custom_values = True

    image_path = "/home/ur_5/ur5_ws/"
    
    
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
