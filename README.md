# active_perception

The code currently detectes the aruco marker, transforms it wrt base_link and fixes it.

Hence, the fixed 3D grid is irrespective of the camera views

## Making the code run on your system

This code runs on the aruco_ros package built for ROS Melodic

1. Add the .launch file to the launch folder
2. Replace the CMakeLists.txt in the aruco_ros folder with this
3. In the /aruco_ros/src folder replace the simple_double.cpp with this
