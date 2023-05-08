#! /usr/bin/env python3
from robotiq_gripper.msg import grip_state


#Updates Gripper State topic 
def updateGripperState(robot_name, position, speed, force, activated, pub):
    robot_name = robot_name
    position = position
    force = force
    speed = speed
    activated = activated

    #gets message structure
    pub_msg = grip_state()

    #sets up the message
    pub_msg.gripper_name = str(robot_name)
    pub_msg.position_state = int(position)
    pub_msg.force_state = int(force)
    pub_msg.speed_state = int(speed)
    pub_msg.activated = bool(activated)

    #publish the message
    pub.publish(pub_msg)
