# Robotiq Gripper 85F 2 Finger on ROS
Robotiq Gripper ROS package to give fulll functionality of the two finger Robotiq gripper on ROS. This package was developed based on the functionality requirements gathered from the RROS-Lab team. Code was developed from [Benoit CASTETS's Robotiq Gripper code](https://github.com/castetsb/pyRobotiqGripper/blob/master/robotiqGripper.py). 

Please note: Do NOT use the pypi.org version of minimalmodbus (use this repo's instead) due to a BAUDRATE error!


## Running Package
```bash
roslaunch Robotiq_Gripper gripper.launch
```

## Topics
_Allows for multiple grippers to be controlled (requires gripper to be attached to a robot)_
* moveGripper: Moves the gripper based on what robot it is attached to.
* grip_state: Shows the grip state of each gripper (along with which robot it is attached to)

## Moving the gripper
`rostopic pub /moveGripper Robotiq_Gripper/change_state robot_name position speed force justGrap`

_If justGrasp is set to True, the gripper will just go to a position that ideally grips the object._

## Reading gripper state
`rostopic echo /grip_state` 

_Returns gripper_name (also known as robot_name), position_state, force_state, speed_state, activated (True if object is in gripper's grasp)_

## Adding more grippers
Add to this dictionary
```python 
#COMs ports to connect gripper to respective robot
#Requires which robot the gripper is attached to and the COM PORT
robot_comms = {
    'bkuka': gripperControl('bkuka', "/dev/ttyUSB1"),
    'gkuka': gripperControl('gkuka', "/dev/ttyUSB2"),
}
```

## Code Structure
```
├── CMakeLists.txt  
├── launch  ──
│   └── gripper.launch  
├── LICENSE  ──
├── msg  
│   └── change_state.msg  
│   └── grip_state.msg
├── package.xml  
├── README.md  
├── src  
│   ├── gripper_ctrl.py
│   ├── gripper_publish.py  
└────── minimalmodbus.py 
```
## Additional Functionality
[Benoit CASTETS's Robotiq Gripper code](https://github.com/castetsb/pyRobotiqGripper/blob/master/robotiqGripper.py) allows for the grippers to move in millimeter increments. We have included that portion into our codebase but have not taken advantage of that feature.
