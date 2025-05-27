# active_perception

The code currently detectes the aruco marker, transforms it wrt base_link and fixes it.

Hence, the fixed 3D grid is irrespective of the camera views

## Making the code run on your system

This code runs on the aruco_ros package. It has been tested to work on ROS Melodic and ROS Noetic

1. Add the .launch file to the launch folder
2. Replace the CMakeLists.txt in the aruco_ros folder with this
3. In the /aruco_ros/src folder replace the simple_double_3D.cpp with this
4. Replace the /aruco_ros/include folder with the include folder here

Add the three files starting with AP_ and the config.py file to your franka workspace.

Replace the variables in config.py file as suitable for you

Bring up franka controllers and the realsense camera. After that in new terminal
```
cd aruco_ws

source devel/setup.bash

roslauch aruco_ros double.launch
```

In another terminal
```
cd franka_ws

source devel/setup.bash

rosrun franka_scripts AP_VLM_version2.py 
```

That's it. You should be able to see the 3D grid and the robot performing active perception
