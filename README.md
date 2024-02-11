# WS23_Door_Opening

|**Ubuntu Distribution**|Ubuntu 20.04 - Focal Fossa|
|---|---|
|ROS1 Distribution| noetic|

## Overview
This repository provides scripts to open a door with a Kinova arm Gen3 with 7 DoF usign the Vision module to obtain the coordinates of the door's handle.

## Required clone repo
```
git clone git@github.com:HBRS-SDP/ws23-door-opening.git
```

### ros_kortex
- Refer to the [Github ros_kortex repo](https://github.com/Kinovarobotics/ros_kortex) to get information about the packages to interact with Kortex
- Refer to the [ros_kortex_vision repo](https://github.com/Kinovarobotics/ros_kortex_vision) to get information about the access to the Kinova Vision module.

## Usage
### Kinova Arm Connection
- Turn the Kinova arm on
- Connect the pc with cable ethernet
- Make sure that the IP from your computer is in the same network as the arm's IP
[!TIP]
Type the Kinova arm IP in your browser to check if your connection was successful

