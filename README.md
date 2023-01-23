# webot_digital_twin
## Instruction to install Webot Digital Twin Package

## Building the Project

### Basic Ubuntu ROS Setup

- Install Ubuntu 20.04 (Preferred)
- Install ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>
### Create workspace and clone repository

- Create a new workspace folder and `src/` subfolder, e.g. `mkdir -p ~/layup_ws/src`
- Clone this repo into the src/ folder of the workspace e.g. `git clone https:://gi... name-of-local-git-folder`

### Webots installation

Please install webots prior launching the package, The recommended webots version is R2022a

### Moveit installation 

Please install Moveit using Moveit Tutorial before installation of Webots:
https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

## addition package installation
### URDF installation
- Please clone repo abb_experimental in the 'webot_digital_twin':
https://github.com/ros-industrial/abb_experimental
- Pleae clone repo epson_experiemental in the 'webot_digital_twin' :
https://github.com/gavanderhoorn/epson_experimental
- Please clone repo iiwa_stack_cam in the 'webot_digital_twin'
https://github.com/RROS-Lab/iiwa_stack_cam
-Remember to 'catkin build' after complete with cloning all repository
## Digital Twin 
### TO access digital twin 

- Use `roslaunch zhao_Webots_MSEC_project digital_twin_features.launch` to launch the webots with digital twin features. 
- Use `roslaunch robot_cell_moveit_simulation demo.launch` for launching the MoveIt! after webot finished. 
- Drag the end effector of any planning group and hit planning and execute
- Press arrow 'up' in the webots windows for displaying current joint position of the robot, Press arrow 'down' in the webots windows for displaying the motion planing joint position of the robot
