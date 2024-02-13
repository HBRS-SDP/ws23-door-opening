# WS23_Door_Opening

|**Ubuntu Distribution**|Ubuntu 20.04 - Focal Fossa|
|---|---|
|ROS1 Distribution| noetic|

## Overview
This repository provides scripts to open a door with a Kinova arm Gen3 with 7 DoF using the Vision module to obtain the coordinates of the door's handle.

## Required Clone Repo
```
git clone git@github.com:HBRS-SDP/ws23-door-opening.git
```

### ros_kortex
- Refer to the [Github ros_kortex repo](https://github.com/Kinovarobotics/ros_kortex) to get information about the packages to interact with Kortex
```
git@github.com:Kinovarobotics/ros_kortex.git
```
- Refer to the [ros_kortex_vision repo](https://github.com/Kinovarobotics/ros_kortex_vision) to get information about the access to the Kinova Vision module.
```
git@github.com:Kinovarobotics/ros_kortex_vision.git
```

## Usage
### Kinova arm connection
1. Turn the Kinova arm on
2. Connect the pc with cable ethernet
3. Make sure that the IP from your computer is connected to the same network as the arm
> [!TIP]
> Type the Kinova arm IP in your browser to check if your connection was successful

### Update launch file
1. Go to `ros_kortex/kortex_driver/launch/kortex_driver.launch`
2. Verify that the IP adress (line 21) matches the arm's IP
3. Add default gripper (line 13) `<arg name="gripper" default="robotiq_2f_85" if="$(eval arg('arm') == 'gen3')"/>`


### Steps in terminal
For every terminal that is used:
1. Source ros .*sh files
```
source /opt/ros/netic/setup.bash
```
2. Source ros devel folder
```
source devel/setup.bash
```
3. Build the code in the workspace
```
catkin build
```
4. Launch the following files for arm movement:
```
roslaunch kortex_driver kortex_driver.launch
```
5. Launch the following files for initializing the kinova_vision node:
```
roslaunch kinova_vision kinova_vision.launch
rosrun rviz rviz
```

### Visualize information of the camera in Rviz
Subscribe to the following topics:
- /camera/color/image_raw
- /camera/depth_registered/points

### Visualize force plot
1. Install PlotJuggler
```
rosrun plotjuggler plotjuggler
```
2. Launch it with the command
```
rosrun plotjuggler plotjuggler
```
3. Under the `Streaming` tab select `ROS topic Subscriber`
4. Grab the topics you want to visualize to the tab in the right
> [!TIP]
> For more information of how to use PlotJuggler, visit [PlotJuggler](https://github.com/facontidavide/PlotJuggler)

