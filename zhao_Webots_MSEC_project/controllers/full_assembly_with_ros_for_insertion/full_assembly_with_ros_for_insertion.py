"""all_controler controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

#development notes: All the point is generated using MoveIt! motion planning

from xml.etree.ElementTree import PI
from controller import Robot
from controller import Connector
from controller import Supervisor
from controller import Keyboard
import time
from time import sleep
import math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

pi_constant = math.pi
degree_to_radian = pi_constant/180
# from robotiq_gripper.msg import grip_state

# create the Robot instance.
robot = Supervisor()
keyboard_input = Keyboard()
rospy.init_node("all_robot_webots_hardcoded_insertion")
rates = rospy.Rate(20)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#setting up dictionary for motor with respect to URDF definition
motor_names = [
    "joint_1",       "joint_2",       "joint_3",       "joint_4",
    "joint_5",       "joint_6",       "c3_joint_1",    "c3_joint_2",
    "c3_joint_3",    "c3_joint_4",    "c3_joint_5",    "c3_joint_6",
    "s5_joint_1",    "s5_joint_2",    "s5_joint_3",    "s5_joint_4",
    "s5_joint_5",    "s5_joint_6", 
    "right_inner_knuckle_joint", "right_inner_finger_joint","left_inner_knuckle_joint", "left_inner_finger_joint",   
    "iiwa1_joint_1", "iiwa1_joint_2",
    "iiwa1_joint_3", "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
    "iiwa1_joint_7", "iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6", "iiwa2_joint_7"]
gripper_name = ["bkuka","gkuka","s5"]
# linear_motor_name = ["blue_kuka_right_gripper_motor","blue_kuka_left_gripper_motor","green_kuka_right_gripper_motor","green_kuka_left_gripper_motor"]
gripper_moter_map = {
    "bkuka":["blue_kuka_left_gripper_motor", "blue_kuka_right_gripper_motor"],
    "gkuka":["green_kuka_left_gripper_motor", "green_kuka_right_gripper_motor"],
    "s5":["right_inner_finger_joint","left_inner_finger_joint"]
}

#setting up the connector 
connector_iiwa1 = Connector("connector_iiwa1")
connector_iiwa1.enablePresence(timestep)
connector_iiwa2 = Connector("connector_iiwa2")
connector_iiwa2.enablePresence(timestep)
iiwa2_special_connector = Connector("iiwa2_special_connector")
iiwa2_special_connector.enablePresence(timestep)
connector_c3 = Connector("connector_c3")
connector_c3.enablePresence(timestep)
connector_s5 = Connector("connector_s5")
connector_s5.enablePresence(timestep)

#setting up the dictionary for motor
motor_map = {}
motor_value = {}
current_joint_states_motor_value = {}
gripper_states = {}
gripper_motor = {}
gripper_motor_values = {}
connector_map = {}
planned_motor_values = {}

#setup motor for robot manipulator
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    
    motor_map[name].setVelocity(2.0)
    motor_value[name] = 0
    current_joint_states_motor_value[name] = 0
#setup gripper motors
for name in gripper_name:
    gripper_motor[name] = []
    gripper_motor_values[name] = []
    for motor_name in gripper_moter_map[name]:
        device = robot.getDevice(motor_name)
        device.setVelocity(1.0)
        gripper_motor[name].append(device)
        gripper_motor_values[name].append(0)

#function for simulator to wait for real time seconds 
def i_want_to_wait (times):
    current_time = time.time()
    new_time = time.time()+times
    while robot.step(timestep) != -1:
                if (time.time() >= new_time):
                    break


#function to move the c3 robot
def move_c3(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6):
  c3_joints = {}
  c3_actual_joint = {}
  c3_joint_names = ["c3_joint_1", "c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6"]
  
  c3_joints["c3_joint_1"] = joint_1
  c3_joints["c3_joint_2"] = joint_2
  c3_joints["c3_joint_3"] = joint_3
  c3_joints["c3_joint_4"] = joint_4
  c3_joints["c3_joint_5"] = joint_5
  c3_joints["c3_joint_6"] = joint_6
  

  while robot.step(timestep) != -1:
    
    for name in c3_joint_names:
        motor_map[name].setPosition(c3_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        c3_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    
    flag = 0
    #for the function to move the robot to desire location before moving to the next path
    for name in  c3_joint_names:  
      
        if (c3_actual_joint[name] < c3_joints[name] + 0.02) and (c3_actual_joint[name] > c3_joints[name] - 0.02):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0

# move_c3 debugging tool
def move_c3_print_joint(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6):
  c3_joints = {}
  c3_actual_joint = {}
  c3_joint_names = ["c3_joint_1", "c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6"]
  
  c3_joints["c3_joint_1"] = joint_1
  c3_joints["c3_joint_2"] = joint_2
  c3_joints["c3_joint_3"] = joint_3
  c3_joints["c3_joint_4"] = joint_4
  c3_joints["c3_joint_5"] = joint_5
  c3_joints["c3_joint_6"] = joint_6

  while robot.step(timestep) != -1:
   
    for name in c3_joint_names:
        motor_map[name].setPosition(c3_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        c3_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
    flag = 0
    #for the function to move the robot to desire location before moving to the next path
    for name in  c3_joint_names:  
        # print("The current flag is: ")
        # print(flag)
        print(c3_actual_joint[name])      
        if (c3_actual_joint[name] < c3_joints[name] + 0.02) and (c3_actual_joint[name] > c3_joints[name] - 0.02):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0

# function for S5 moving 
def move_s5(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6):
  s5_joints = {}
  s5_actual_joint = {}
  s5_joint_names = ["s5_joint_1", "s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6"]
  
  s5_joints["s5_joint_1"] = joint_1
  s5_joints["s5_joint_2"] = joint_2
  s5_joints["s5_joint_3"] = joint_3
  s5_joints["s5_joint_4"] = joint_4
  s5_joints["s5_joint_5"] = joint_5
  s5_joints["s5_joint_6"] = joint_6
  
  while robot.step(timestep) != -1:

    for name in s5_joint_names:
        motor_map[name].setPosition(s5_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        s5_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
    flag = 0
    #for the function to move the robot to desire location before moving to the next path
    for name in  s5_joint_names:      
        if (s5_actual_joint[name] < s5_joints[name] + 0.01) and (s5_actual_joint[name] > s5_joints[name] - 0.01):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0

#move_s5 debugging  
def move_s5_print(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6):
  s5_joints = {}
  s5_actual_joint = {}
  s5_joint_names = ["s5_joint_1", "s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6"]
  
  s5_joints["s5_joint_1"] = joint_1
  s5_joints["s5_joint_2"] = joint_2
  s5_joints["s5_joint_3"] = joint_3
  s5_joints["s5_joint_4"] = joint_4
  s5_joints["s5_joint_5"] = joint_5
  s5_joints["s5_joint_6"] = joint_6
  
  while robot.step(timestep) != -1:

    for name in s5_joint_names:
        motor_map[name].setPosition(s5_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        s5_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
    flag = 0
    #for the function to move the robot to desire location before moving to the next path
    for name in  s5_joint_names:  

        print(name)
        print(s5_actual_joint[name])      
        if (s5_actual_joint[name] < s5_joints[name] + 0.01) and (s5_actual_joint[name] > s5_joints[name] - 0.01):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0

#Blue Kuka movement    
def move_iiwa1(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6,joint_7):
  iiwa1_joints = {}
  iiwa1_actual_joint = {}
  iiwa1_joint_names = ["iiwa1_joint_1", "iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7"]
  
  iiwa1_joints["iiwa1_joint_1"] = joint_1
  iiwa1_joints["iiwa1_joint_2"] = joint_2
  iiwa1_joints["iiwa1_joint_3"] = joint_3
  iiwa1_joints["iiwa1_joint_4"] = joint_4
  iiwa1_joints["iiwa1_joint_5"] = joint_5
  iiwa1_joints["iiwa1_joint_6"] = joint_6
  iiwa1_joints["iiwa1_joint_7"] = joint_7

  while robot.step(timestep) != -1:
    
    for name in iiwa1_joint_names:
        motor_map[name].setPosition(iiwa1_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        iiwa1_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
    flag = 0
    #for the function to move the robot to desire location before moving to the next path
    for name in  iiwa1_joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
        if (iiwa1_actual_joint[name] < iiwa1_joints[name] + 0.02) and (iiwa1_actual_joint[name] > iiwa1_joints[name] - 0.02):
            flag += 1
    if (flag >= 7):
        break
    else:
        flag = 0

#move blue kuka print joint states  
def move_iiwa1_print_joint_states(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6,joint_7):
  iiwa1_joints = {}
  iiwa1_actual_joint = {}
  iiwa1_joint_names = ["iiwa1_joint_1", "iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7"]
  
  iiwa1_joints["iiwa1_joint_1"] = joint_1
  iiwa1_joints["iiwa1_joint_2"] = joint_2
  iiwa1_joints["iiwa1_joint_3"] = joint_3
  iiwa1_joints["iiwa1_joint_4"] = joint_4
  iiwa1_joints["iiwa1_joint_5"] = joint_5
  iiwa1_joints["iiwa1_joint_6"] = joint_6
  iiwa1_joints["iiwa1_joint_7"] = joint_7

  while robot.step(timestep) != -1:

    for name in iiwa1_joint_names:
        motor_map[name].setPosition(iiwa1_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        iiwa1_actual_joint[name] = motor_map[name].getPositionSensor().getValue()

    flag = 0
    #for the function to move the robot to desire location before moving to the next path

    for name in  iiwa1_joint_names:  
        # print("The current flag is: ")
        # print(flag)
        print(name)
        print(iiwa1_actual_joint[name])      
        if (iiwa1_actual_joint[name] < iiwa1_joints[name] + 0.02) and (iiwa1_actual_joint[name] > iiwa1_joints[name] - 0.02):
            flag += 1
    if (flag >= 7):
        break
    else:
        flag = 0

#decapitated but could be used for cartesian path if moveit planner fail
def iiwa1_cartesian_mode (joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7,new_joint_1,new_joint_2,new_joint_3,new_joint_4,new_joint_5,new_joint_6,new_joint_7,data_points):
    iiwa1_joints = {}
    iiwa1_old_joints = {}
    iiwa1_actual_joint = {}
    iiwa1_joint_names = ["iiwa1_joint_1", "iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7"]
    
    iiwa1_old_joints["iiwa1_joint_1"] = joint_1
    iiwa1_old_joints["iiwa1_joint_2"] = joint_2
    iiwa1_old_joints["iiwa1_joint_3"] = joint_3
    iiwa1_old_joints["iiwa1_joint_4"] = joint_4
    iiwa1_old_joints["iiwa1_joint_5"] = joint_5
    iiwa1_old_joints["iiwa1_joint_6"] = joint_6
    iiwa1_old_joints["iiwa1_joint_7"] = joint_7

    iiwa1_joints["iiwa1_joint_1"] = new_joint_1
    iiwa1_joints["iiwa1_joint_2"] = new_joint_2
    iiwa1_joints["iiwa1_joint_3"] = new_joint_3
    iiwa1_joints["iiwa1_joint_4"] = new_joint_4
    iiwa1_joints["iiwa1_joint_5"] = new_joint_5
    iiwa1_joints["iiwa1_joint_6"] = new_joint_6
    iiwa1_joints["iiwa1_joint_7"] = new_joint_7
    g = 0

    while robot.step(timestep) != -1:
        for name in iiwa1_joint_names:
            motor_map[name].setPosition(iiwa1_old_joints[name] + (iiwa1_joints[name]-iiwa1_old_joints[name])/data_points*g)
            motor_map[name].getPositionSensor().enable(timestep)
            iiwa1_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
            if (g >= data_points):
                    break
            g +=1

#move the green kuka   
def move_iiwa2(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6,joint_7):
    iiwa2_joints = {}
    iiwa2_actual_joint = {}
    iiwa2_joint_names = ["iiwa2_joint_1", "iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"]
    while robot.step(timestep) != -1:
        iiwa2_joints["iiwa2_joint_1"] = joint_1
        iiwa2_joints["iiwa2_joint_2"] = joint_2
        iiwa2_joints["iiwa2_joint_3"] = joint_3
        iiwa2_joints["iiwa2_joint_4"] = joint_4
        iiwa2_joints["iiwa2_joint_5"] = joint_5
        iiwa2_joints["iiwa2_joint_6"] = joint_6
        iiwa2_joints["iiwa2_joint_7"] = joint_7
        flag = 0
        for name in iiwa2_joint_names:
            motor_map[name].setPosition(iiwa2_joints[name])
            motor_map[name].getPositionSensor().enable(timestep)
            iiwa2_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  iiwa2_joint_names:  
     
             if (iiwa2_actual_joint[name] < iiwa2_joints[name] + 0.02) and (iiwa2_actual_joint[name] > iiwa2_joints[name] - 0.02):
                flag += 1
        if (flag >= 7):
         break
        else:
            flag = 0

#move the iiwa2 with current sensor value display
def move_iiwa2_print_joint_states(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6,joint_7):
    iiwa2_joints = {}
    iiwa2_actual_joint = {}
    iiwa2_joint_names = ["iiwa2_joint_1", "iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"]
    while robot.step(timestep) != -1:
        iiwa2_joints["iiwa2_joint_1"] = joint_1
        iiwa2_joints["iiwa2_joint_2"] = joint_2
        iiwa2_joints["iiwa2_joint_3"] = joint_3
        iiwa2_joints["iiwa2_joint_4"] = joint_4
        iiwa2_joints["iiwa2_joint_5"] = joint_5
        iiwa2_joints["iiwa2_joint_6"] = joint_6
        iiwa2_joints["iiwa2_joint_7"] = joint_7
        flag = 0
        for name in iiwa2_joint_names:
            motor_map[name].setPosition(iiwa2_joints[name])
            motor_map[name].getPositionSensor().enable(timestep)
            iiwa2_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  iiwa2_joint_names:  
        # print("The current flag is: ")
        # print(flag)
             print(name)
             print(iiwa2_actual_joint[name])      
             if (iiwa2_actual_joint[name] < iiwa2_joints[name] + 0.02) and (iiwa2_actual_joint[name] > iiwa2_joints[name] - 0.02):
                flag += 1
        if (flag >= 7):
         break
        else:
            flag = 0


#move abb 
def move_abb(joint_1, joint_2,joint_3,joint_4,joint_5,joint_6):
    abb_joints = {}
    abb_actual_joint = {}
    abb_joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6"]
    while robot.step(timestep) != -1:
        abb_joints["joint_1"] = joint_1
        abb_joints["joint_2"] = joint_2
        abb_joints["joint_3"] = joint_3
        abb_joints["joint_4"] = joint_4
        abb_joints["joint_5"] = joint_5
        abb_joints["joint_6"] = joint_6

        flag = 0
        for name in abb_joint_names:
            motor_map[name].setPosition(abb_joints[name])
            motor_map[name].getPositionSensor().enable(timestep)
            abb_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  abb_joint_names:      
             if (abb_actual_joint[name] < abb_joints[name] + 0.01) and (abb_actual_joint[name] > abb_joints[name] - 0.01):
                flag += 1
        if (flag >= 6):
         break
        else:
            flag = 0

#for abb screw pickup 
def abb_screw_pickup_cycyles():
    f = 0
    while robot.step(timestep) != -1:
        if (f == 1):
            move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
        if (f == 2):
            move_abb(-2.6250423309671804, 0.4541524299854982, 0.513736461374945, 0.0007820188173276552, 0.6041151545952707, 0.5131486710829466)
        if (f == 3):
            move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
        if (f == 4):
            break
        f+=1
        # joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,iiwa1_joint_1,iiwa1_joint_2,iiwa1_joint_3,iiwa1_joint_4,iiwa1_joint_5,iiwa1_joint_6,iiwa1_joint_7,iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7

#general move function 
def move(        joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,
c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,
s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,
iiwa1_joint_1,iiwa1_joint_2,iiwa1_joint_3,iiwa1_joint_4,iiwa1_joint_5,iiwa1_joint_6,iiwa1_joint_7,
iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7
 ):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6","c3_joint_1","c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6",
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    "iiwa1_joint_1","iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7",
    "iiwa2_joint_1","iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"
]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
        joints["c3_joint_1"] = c3_joint_1
        joints["c3_joint_2"] = c3_joint_2
        joints["c3_joint_3"] = c3_joint_3
        joints["c3_joint_4"] = c3_joint_4
        joints["c3_joint_5"] = c3_joint_5
        joints["c3_joint_6"] = c3_joint_6
        
        joints["iiwa1_joint_1"] = iiwa1_joint_1
        joints["iiwa1_joint_2"] = iiwa1_joint_2
        joints["iiwa1_joint_3"] = iiwa1_joint_3
        joints["iiwa1_joint_4"] = iiwa1_joint_4
        joints["iiwa1_joint_5"] = iiwa1_joint_5
        joints["iiwa1_joint_6"] = iiwa1_joint_6
        joints["iiwa1_joint_7"] = iiwa1_joint_7
        joints["iiwa2_joint_1"] = iiwa2_joint_1
        joints["iiwa2_joint_2"] = iiwa2_joint_2
        joints["iiwa2_joint_3"] = iiwa2_joint_3
        joints["iiwa2_joint_4"] = iiwa2_joint_4
        joints["iiwa2_joint_5"] = iiwa2_joint_5
        joints["iiwa2_joint_6"] = iiwa2_joint_6
        joints["iiwa2_joint_7"] = iiwa2_joint_7

    
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:    
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= 31):
         break
        else:
            flag = 0

#move function for C3, S5, and blue kuka 
def move_c3_s5_iiwa1 (c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,
s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,
iiwa1_joint_1,iiwa1_joint_2,iiwa1_joint_3,iiwa1_joint_4,iiwa1_joint_5,iiwa1_joint_6,iiwa1_joint_7):
    joints = {}
    actual_joint = {}
    joint_names = ["c3_joint_1","c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6",
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    "iiwa1_joint_1","iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7",
    
]
    while robot.step(timestep) != -1:
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
        joints["c3_joint_1"] = c3_joint_1
        joints["c3_joint_2"] = c3_joint_2
        joints["c3_joint_3"] = c3_joint_3
        joints["c3_joint_4"] = c3_joint_4
        joints["c3_joint_5"] = c3_joint_5
        joints["c3_joint_6"] = c3_joint_6
        
        joints["iiwa1_joint_1"] = iiwa1_joint_1
        joints["iiwa1_joint_2"] = iiwa1_joint_2
        joints["iiwa1_joint_3"] = iiwa1_joint_3
        joints["iiwa1_joint_4"] = iiwa1_joint_4
        joints["iiwa1_joint_5"] = iiwa1_joint_5
        joints["iiwa1_joint_6"] = iiwa1_joint_6
        joints["iiwa1_joint_7"] = iiwa1_joint_7
       
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:  
    
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)-1):
         break
        else:
            flag = 0

#move function for c3, blue kuka, green kuka
def move_c3_iiwa1_iiwa2 (c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,
iiwa1_joint_1,iiwa1_joint_2,iiwa1_joint_3,iiwa1_joint_4,iiwa1_joint_5,iiwa1_joint_6,iiwa1_joint_7,
iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7):
    joints = {}
    actual_joint = {}
    joint_names = ["c3_joint_1","c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6",
    "iiwa1_joint_1","iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7",
    "iiwa2_joint_1","iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"
    
]
    while robot.step(timestep) != -1:
        
        joints["c3_joint_1"] = c3_joint_1
        joints["c3_joint_2"] = c3_joint_2
        joints["c3_joint_3"] = c3_joint_3
        joints["c3_joint_4"] = c3_joint_4
        joints["c3_joint_5"] = c3_joint_5
        joints["c3_joint_6"] = c3_joint_6
        joints["iiwa1_joint_1"] = iiwa1_joint_1
        joints["iiwa1_joint_2"] = iiwa1_joint_2
        joints["iiwa1_joint_3"] = iiwa1_joint_3
        joints["iiwa1_joint_4"] = iiwa1_joint_4
        joints["iiwa1_joint_5"] = iiwa1_joint_5
        joints["iiwa1_joint_6"] = iiwa1_joint_6
        joints["iiwa1_joint_7"] = iiwa1_joint_7
       
        joints["iiwa2_joint_1"] = iiwa2_joint_1
        joints["iiwa2_joint_2"] = iiwa2_joint_2
        joints["iiwa2_joint_3"] = iiwa2_joint_3
        joints["iiwa2_joint_4"] = iiwa2_joint_4
        joints["iiwa2_joint_5"] = iiwa2_joint_5
        joints["iiwa2_joint_6"] = iiwa2_joint_6
        joints["iiwa2_joint_7"] = iiwa2_joint_7
       

    
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        #for the function to move the robot to desire location before moving to the next path
        for name in  joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)-1):
         break
        else:
            flag = 0

#move c3, and abb 
def move_c3_abb(c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,
joint_1,joint_2,joint_3,joint_4,joint_5,joint_6
):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6",
        "c3_joint_1","c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6",
    
    
]
    while robot.step(timestep) != -1:
        
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["c3_joint_1"] = c3_joint_1
        joints["c3_joint_2"] = c3_joint_2
        joints["c3_joint_3"] = c3_joint_3
        joints["c3_joint_4"] = c3_joint_4
        joints["c3_joint_5"] = c3_joint_5
        joints["c3_joint_6"] = c3_joint_6
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        #for the function to move the robot to desire location before moving to the next path
        for name in  joint_names:  
          
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function S5 and ABB       
def move_s5_abb(s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,
joint_1,joint_2,joint_3,joint_4,joint_5,joint_6):
    joints = {}
    actual_joint = {}
    joint_names = ["s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6","joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6"]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()

        #for the function to move the robot to desire location before moving to the next path        
        for name in  joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function for C3, S5, and ABB
def move_c3_s5_abb(c3_joint_1,c3_joint_2,c3_joint_3,c3_joint_4,c3_joint_5,c3_joint_6,
s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,
joint_1,joint_2,joint_3,joint_4,joint_5,joint_6):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6","c3_joint_1","c3_joint_2","c3_joint_3","c3_joint_4","c3_joint_5","c3_joint_6",
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    
]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
        joints["c3_joint_1"] = c3_joint_1
        joints["c3_joint_2"] = c3_joint_2
        joints["c3_joint_3"] = c3_joint_3
        joints["c3_joint_4"] = c3_joint_4
        joints["c3_joint_5"] = c3_joint_5
        joints["c3_joint_6"] = c3_joint_6
        
     
        #get sensor data to check if current position is in desire location
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        flag = 0
        #for the function to move the robot to desire location before moving to the next path
        for name in  joint_names:     
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function for S5, abb, and green kuka
def move_s5_abb_iiwa2(      s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,  joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,
iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7

 ):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6",
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    
    "iiwa2_joint_1","iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"
]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
       
        
        joints["iiwa2_joint_1"] = iiwa2_joint_1
        joints["iiwa2_joint_2"] = iiwa2_joint_2
        joints["iiwa2_joint_3"] = iiwa2_joint_3
        joints["iiwa2_joint_4"] = iiwa2_joint_4
        joints["iiwa2_joint_5"] = iiwa2_joint_5
        joints["iiwa2_joint_6"] = iiwa2_joint_6
        joints["iiwa2_joint_7"] = iiwa2_joint_7
  
        flag = 0
        #for the function to move the robot to desire location before moving to the next path
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:  
     
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function for S5, ABB, and Blue Kuka            
def move_s5_abb_iiwa1(      s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6,  joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,


iiwa1_joint_1,iiwa1_joint_2,iiwa1_joint_3,iiwa1_joint_4,iiwa1_joint_5,iiwa1_joint_6,iiwa1_joint_7

 ):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6",
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    
    "iiwa1_joint_1","iiwa1_joint_2","iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6","iiwa1_joint_7"
]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
       
        
        joints["iiwa1_joint_1"] = iiwa1_joint_1
        joints["iiwa1_joint_2"] = iiwa1_joint_2
        joints["iiwa1_joint_3"] = iiwa1_joint_3
        joints["iiwa1_joint_4"] = iiwa1_joint_4
        joints["iiwa1_joint_5"] = iiwa1_joint_5
        joints["iiwa1_joint_6"] = iiwa1_joint_6
        joints["iiwa1_joint_7"] = iiwa1_joint_7
  

    
        flag = 0

        #for the function to move the robot to desire location before moving to the next path
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:     
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function for ABB and Green KuKa
def move_abb_iiwa2(      joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,
iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7

 ):
    joints = {}
    actual_joint = {}
    joint_names = ["joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6",
    
    
    "iiwa2_joint_1","iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"
]
    while robot.step(timestep) != -1:
        joints["joint_1"] = joint_1
        joints["joint_2"] = joint_2
        joints["joint_3"] = joint_3
        joints["joint_4"] = joint_4
        joints["joint_5"] = joint_5
        joints["joint_6"] = joint_6
      
       
        
        joints["iiwa2_joint_1"] = iiwa2_joint_1
        joints["iiwa2_joint_2"] = iiwa2_joint_2
        joints["iiwa2_joint_3"] = iiwa2_joint_3
        joints["iiwa2_joint_4"] = iiwa2_joint_4
        joints["iiwa2_joint_5"] = iiwa2_joint_5
        joints["iiwa2_joint_6"] = iiwa2_joint_6
        joints["iiwa2_joint_7"] = iiwa2_joint_7
  

    
        flag = 0
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()

        #for the function to move the robot to desire location before moving to the next path
        for name in  joint_names:       
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

#move function for S5 and Green KuKa
def move_s5_iiwa2(      s5_joint_1,s5_joint_2,s5_joint_3,s5_joint_4,s5_joint_5,s5_joint_6, 
iiwa2_joint_1,iiwa2_joint_2,iiwa2_joint_3,iiwa2_joint_4,iiwa2_joint_5,iiwa2_joint_6,iiwa2_joint_7

 ):
    joints = {}
    actual_joint = {}
    joint_names = [
    "s5_joint_1","s5_joint_2","s5_joint_3","s5_joint_4","s5_joint_5","s5_joint_6",
    
    "iiwa2_joint_1","iiwa2_joint_2","iiwa2_joint_3","iiwa2_joint_4","iiwa2_joint_5","iiwa2_joint_6","iiwa2_joint_7"
]
    while robot.step(timestep) != -1:
 
        joints["s5_joint_1"] = s5_joint_1
        joints["s5_joint_2"] = s5_joint_2
        joints["s5_joint_3"] = s5_joint_3
        joints["s5_joint_4"] = s5_joint_4
        joints["s5_joint_5"] = s5_joint_5
        joints["s5_joint_6"] = s5_joint_6
       
        
        joints["iiwa2_joint_1"] = iiwa2_joint_1
        joints["iiwa2_joint_2"] = iiwa2_joint_2
        joints["iiwa2_joint_3"] = iiwa2_joint_3
        joints["iiwa2_joint_4"] = iiwa2_joint_4
        joints["iiwa2_joint_5"] = iiwa2_joint_5
        joints["iiwa2_joint_6"] = iiwa2_joint_6
        joints["iiwa2_joint_7"] = iiwa2_joint_7
  

    
        flag = 0
        
        #for the function to move the robot to desire location before moving to the next path
        for name in joint_names:
            
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:      
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0


#indicates human operation
human_pub = rospy.Publisher("/human_operation_que",Bool,queue_size=200)
solar_module_pub = rospy.Publisher("/solar_module_procedure",String,queue_size=200)

#function to call for showing the human feature to indicate human operation
def human_on():

            msg = Bool()
            msg.data = True
            human_pub.publish(msg)
            
#function to call for disappearing the human feature to indicate human operation      
def human_off():

            msg = Bool()
            msg.data = False
            human_pub.publish(msg)

# function for antenna connector for saving computer power
def antenna_insertion ():
    msg = String()
    msg.data = "antenna_inserted"
    solar_module_pub.publish(msg)

# function for star tracker connector for saving computer power
def star_tracker_insertion ():
    msg = String()
    msg.data = "star_tracker_inserted"
    solar_module_pub.publish(msg)

# function for battery insertion connector for saving computer power
def battery_insertion ():
    msg = String()
    msg.data = "battery_inserted"
    solar_module_pub.publish(msg)

#function to indicates frame 4 human operation    
def frame4_start():
    msg = String()
    msg.data = "frame4_start"
    solar_module_pub.publish(msg)

#function for solar module star (disabled for the demo)         
def solar_start():
    msg = String()
    msg.data = "solar_start"
    solar_module_pub.publish(msg) 

#function in indicate frame 4 finished
def frame4_finished():
    msg = String()
    msg.data = "frame4_finished"
    solar_module_pub.publish(msg)

#function in indicate solar1_half finished    
def solar_one_finshed_half():
    msg = String()
    msg.data = "solar_module_1_finished_half"
    solar_module_pub.publish(msg)

#function in indicate solar1 finished    
def solar_one_finshed():
    msg = String()
    msg.data = "solar_module_1_finished"
    solar_module_pub.publish(msg)

#function to indicate solar2 half finished
def solar_two_finshed_half():
    msg = String()
    msg.data = "solar_module_2_finished_half"
    solar_module_pub.publish(msg)

#function in indicate solar2 finished    
def solar_two_finshed():
    msg = String()
    msg.data = "solar_module_2_finished"
    solar_module_pub.publish(msg)

#function in indicate solar3_half finished    
def solar_three_finshed_half():
    msg = String()
    msg.data = "solar_module_3_finished_half"
    solar_module_pub.publish(msg)

#function in indicate solar3 finished    
def solar_three_finshed():
    msg = String()
    msg.data = "solar_module_3_finished"
    solar_module_pub.publish(msg)

#function in indicate solar4_half finished    
def solar_four_finshed_half():
    msg = String()
    msg.data = "solar_module_4_finished_half"
    solar_module_pub.publish(msg)

#function in indicate solar4 finished    
def solar_four_finshed():
    msg = String()
    msg.data = "solar_module_4_finished"
    solar_module_pub.publish(msg)

#function in indicate endcard_negative_half finished    
def endcard_negative_finished_half():
    msg = String()
    msg.data = "endcard_negative_finished_half"
    solar_module_pub.publish(msg)

#function in indicate endcard_negative finished 
def endcard_negative_finished():
    msg = String()
    msg.data = "endcard_negative_finished"
    solar_module_pub.publish(msg)

#function in indicate endcard_positive_half finished 
def endcard_positive_finished_half():
    msg = String()
    msg.data = "endcard_positive_finished_half"
    solar_module_pub.publish(msg)

#function in indicate endcard_positive finished 
def endcard_positive_finished():
    msg = String()
    msg.data = "endcard_positive_finished"
    solar_module_pub.publish(msg)

#function in control iiwa2 gripper 
def iiwa2_gripper (distance1, distance2):
    motor1 = robot.getDevice("green_kuka_left_gripper_motor")
    motor2 = robot.getDevice("green_kuka_right_gripper_motor")
    sensor1 = robot.getDevice("green_kuka_right_gripper_sensor")
    sensor2 = robot.getDevice("green_kuka_left_gripper_sensor")
    while robot.step(timestep) != -1:
                motor1.setPosition(distance1)
                motor2.setPosition(distance2)

                sensor1.enable(timestep)
                sensor2.enable(timestep)
                motor1_position = sensor1.getValue()
                # print("motor1_position")
                # print(motor1_position)
                # print("distance1")
                # print(distance1)
                if (abs(motor1_position) < abs(distance1) + 0.01) and (abs(motor1_position) > abs(distance1) - 0.01):
                        break

##setting function for solar module screwing
def abb_solar_module_1_2_screws ():
        #first screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(1.848693321299648, -0.543290785885235, 0.5349943917438886, 0.000567480087451969, 1.580091598522319, 0.6924774596072147)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #second screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(2.282901795018469, -0.37883132785003626, 0.43553707501149647, 0.0009362495882457525, 1.514831858073312, 1.1268027887562284)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
def abb_solar_module_3_4_screws ():
        #third screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(2.5454026190331676, -0.11420273007139016, 0.22744364943039708, 0.0011986206919659145, 1.457933770954946, 1.3890688145348045)
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #fourth screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(2.685623527411623, 0.16634227948369992, -0.02695035754527958, 0.0011560852286533721, 1.4315255049485036, 1.5293776239522412)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()

def abb_solar_module_4_screws ():
        #first screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(1.8486735029068402, -0.551806289577936, 0.5726434708129883, 0.0006252086593768184, 1.5509951206054322, 0.6924174024332395)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #second screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(2.282996903940546, -0.3846278558163751, 0.4720667501182722, 0.0009397685347604525, 1.484049177592619, 1.126706491834365)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #third screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(2.54542420722709, -0.1177363047951821, 0.26316029578649996, 0.001150990495610996, 1.4258451354151505, 1.389042105809328)
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #fourth screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(2.685623527411623, 0.16634227948369992, -0.02695035754527958, 0.0011560852286533721, 1.4315255049485036, 1.5293776239522412)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
def abb_screwing_endcap_4_screws():
        # abb screw_endcard
        #first screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.2459569383872267, -0.492005882734291, 0.30245944534448177, 0.0010097919636789573, 1.7609815421760484, 1.0898128475354234)
        move_abb(2.2459571984214457, -0.5558790422183612, 0.5083353557115623, 0.0009921735290835338, 1.6189788620075911, 1.0896699515025905)
        move_abb(2.2459569383872267, -0.492005882734291, 0.30245944534448177, 0.0010097919636789573, 1.7609815421760484, 1.0898128475354234)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #second screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.7060056907545587, -0.026558961878305625, -0.052769623811910235, 0.0012125643209655304, 1.6504739024989208, 1.549764362157218)
        move_abb(2.706005690694027, -0.06856827192380247, 0.15310216827636758, 0.0012130132601408097, 1.4866115404266347, 1.5495658532792822)
        move_abb(2.7060056907545587, -0.026558961878305625, -0.052769623811910235, 0.0012125643209655304, 1.6504739024989208, 1.549764362157218)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #third screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(1.7330235506209084, -0.38167943310806596, 0.23607990320400876, 0.0005852424530005912, 1.7173140584806048, 0.5768783091955337)
        move_abb(1.733023687388145, -0.43654296356310274, 0.43996204471412764, 0.0005786166068941564, 1.568295472502554, 0.5767915416552652)
        move_abb(1.7330235506209084, -0.38167943310806596, 0.23607990320400876, 0.0005852424530005912, 1.7173140584806048, 0.5768783091955337)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
        #forth screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.6204898005525292, 0.307205212261171, -0.4293095604705171, 0.0011477358264624033, 1.693115026205753, 1.4645286607997998)
        move_abb(2.6204898005822947, 0.25184595057466413, -0.19307123026376727, 0.0011411161079528471, 1.5122360754894146, 1.4643218349398162)
        move_abb(2.6204898005525292, 0.307205212261171, -0.4293095604705171, 0.0011477358264624033, 1.693115026205753, 1.4645286607997998)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()

#initalized inital global parameters
k = 1

while robot.step(timestep) != -1:
   
    # total assembly process (for insertion and solar module + endcard + & -)

    if (k == 1):
     
        move_iiwa2(112*degree_to_radian,21*degree_to_radian,88*degree_to_radian,-80*degree_to_radian,153*degree_to_radian,-105*degree_to_radian,-135*degree_to_radian)
        move_c3(2.2653607233125,  -0.13453550799878,  -0.22716406236104,  -0.050635476592814,  -1.1806466892077,  -0.656)
        move_c3(2.059219466845119, -1.3017633196567302, -0.12746187566417105, -0.39951569025014977, -0.13326858382132958, -0.07267500779692976)
        move_s5_abb_iiwa1(0.0, 0.31, -0.54, 0.0, -0.91, 0.75,-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145,-2.175429352743721, 0.7132396533949974, 0.8687322834606889, -1.7676837525001998, 1.2103287202939845, 1.1478391512027266, -2.27203253794364)
        move_s5_abb_iiwa1(0.0, 0.31, -0.54, 0.0, -0.91, 0.75,-1.8645502853848008, 0.3153908822061336, 0.32082163093265237, 0.0013131770990751866, 0.9353683137449873, -0.569932522419301,1.775053858757019, 0.12268832325935364, 1.2970420122146606, -1.444926381111145, -0.08761254698038101, 1.6743909120559692, -1.6920497417449951)
        move_abb(-1.8645502853288132, 0.255152782865962, 0.22597437215082783, 0.001191727839920763, 1.090453551002733, -0.5697037999835963)
        move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
        move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
        abb_screw_pickup_cycyles()
        
        #screwing the antenna while iiwa pickup battery insertion
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-1.4853272438049316, 0.18084903061389923, -0.01287818793207407, 0, 0.0031921921763569117, 1.6500217914581299, -1.477664589881897)
        move_abb_iiwa2(-0.5398876282193393, -0.14044425467392588, 0.0459794987939382, -0.02370256124287502, 1.70058053829685, -0.04382695397959718,-1.8344966788856316, 0.8176748764600984, 1.8211549231164652, -1.4951087182826748, -0.8226734663841251, 1.8015257293040383, 1.373635117167009)
        move_abb_iiwa2(-0.5399036950512087, -0.13544478064221926, 0.6985723841851069, -0.02730699442691207, 1.0432485231595223, -0.026882402559685728,-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_abb_iiwa2(-0.5398876282193393, -0.14044425467392588, 0.0459794987939382, -0.02370256124287502, 1.70058053829685, -0.04382695397959718,-1.7838280817553236, 1.1256552210713355, 1.4236858017120668, -1.9075514016468627, -1.194775461181099, 1.2986927011510963, 1.5620754025394958)
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-1.766061082571852, 1.161263427636813, 1.3701981211176706, -1.9173302730755886, -1.2462948623717125, 1.2571041671620533, 1.5816349478320404)
        abb_screw_pickup_cycyles()
       
        connector_iiwa2.lock() #picking up battery
        
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-1.7838280817553236, 1.1256552210713355, 1.4236858017120668, -1.9075514016468627, -1.194775461181099, 1.2986927011510963, 1.5620754025394958)
        move_abb_iiwa2(-0.17263625625515594, 0.023648042167017307, -0.10945148946775907, -0.009299101952616986, 1.698125003444409, 0.32526688540925236,-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, -0.09728966070509085)
        move_abb_iiwa2(-0.18893779270086275, -0.02890232540529255, 0.27066666089889396, -0.010101512490093688, 1.3704168760917868, 0.31217017026434907,-0.5969481414980955, 0.48156354285931713, 0.47875933457530473, -1.8585173756389062, -0.29688951248051093, 0.8674472505818945, 0.04983038138284982)
        move_abb_iiwa2(-0.17263625625515594, 0.023648042167017307, -0.10945148946775907, -0.009299101952616986, 1.698125003444409, 0.32526688540925236,-0.6092106655526445, 0.4474379337556961, 0.49259220725607633, -1.8446612093533783, -0.27462377217156314, 0.9119531487722079, -0.028737077574713048)
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-0.38932784085430294, 0.43360072280455314, 0.5343149320229683, -1.8749258276990186, -0.28863485134506556, 0.9073320572671125, 0.244)
        abb_screw_pickup_cycyles()
        
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-0.37941179430110805, 0.43454097278952103, 0.5356989616814904, -1.8740748757012728, -0.28967785361014337, 0.9078843374595413, 0.25564088139248314)
        
        connector_iiwa2.unlock()
        
        move_abb_iiwa2(-0.3428995829514646, 0.4120397238077253, -0.5753662162537994, -0.016452427553756505, 1.7732514708779736, 0.15298335619994485,-0.6092106655526445, 0.4474379337556961, 0.49259220725607633, -1.8446612093533783, -0.27462377217156314, 0.9119531487722079, -0.028737077574713048)
        move_abb_iiwa2(-0.3428995829523859, 0.31971545528709433, -0.121820520355974, -0.01632154151876618, 1.4120774940312009, 0.15887171173823594,-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        iiwa2_gripper (-0.05,0.05)
        move_abb_iiwa2(-0.3428995829514646, 0.4120397238077253, -0.5753662162537994, -0.016452427553756505, 1.7732514708779736, 0.15298335619994485,-0.6092106655526445, 0.4474379337556961, 0.49259220725607633, -1.8446612093533783, -0.27462377217156314, 0.9119531487722079, -0.028737077574713048)
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-0.5780579175392716, 0.4411859857633598, 0.49957271345537996, -1.8552624363032153, -0.27541058238243743, 0.908971517613135, 0.011664232304563767)
        abb_screw_pickup_cycyles()
        battery_insertion()
        move_abb_iiwa2(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652,-0.42548356582533636, 0.43149597436089293, 0.528664385623303, -1.8763168295792896, -0.2850357510091193, 0.9057839883455264, 0.20094889496195958)
        move_abb_iiwa2(-0.610050934353922, 0.25843845680142463, -0.3745625715137532, -0.026260157240759672, 1.7203668320097636, -0.11459175795832709,-0.37941179430110805, 0.43454097278952103, 0.5356989616814904, -1.8740748757012728, -0.28967785361014337, 0.9078843374595413, 0.25564088139248314)
        move_abb_iiwa2(-0.6100509343567877, 0.18947857541880553, 0.03704580603893819, -0.02645805620116717, 1.3778350494696212, -0.10560286245649904,-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        iiwa2_gripper (0,0)
        move_abb_iiwa2(-0.610050934353922, 0.25843845680142463, -0.3745625715137532, -0.026260157240759672, 1.7203668320097636, -0.11459175795832709,-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
        abb_screw_pickup_cycyles()
        #abb is out iiwa2 keep insertion
        move_iiwa2(-0.713322865632312, 0.4206186522063446, 0.49506247453580793, -1.7719952980513909, -0.24365685237702026, 1.0036242591956464, -2.2435)
        move_iiwa2(-0.6562201064387703, 0.3993136769865359, 0.5127917142385011, -1.8045450435860804, -0.24155423330640444, 0.9943535712739969, -2.2435)
        move_iiwa2(-0.6287669356064784, 1.137666884204557, -0.10621056365407022, -1.0354210149708842, 1.9673649028237934, -0.8824188132847066, -2.2435)
        move_iiwa2(-0.49865109369550986, 1.2363028516901984, -0.3251146203912102, -1.1733192520981173, 2.397161525231847, -1.6215123520216737, -2.2435)
        move_iiwa2(-0.4466041326522827, 1.2758291959762573, -0.41276055574417114, -1.2283869981765747, 2.569065570831299, -1.9171887636184692, -2.2435)
        move_iiwa2(-0.3394803615971062, 1.240212700330852, -0.3644027640047344, -1.3916198125568475, 2.6716591125251887, -1.8872762491628499, -2.1041216978057817)
        # #cartiesan path to get adacs card
        move_iiwa2(-0.3475541547853751, 1.2152803124778528, -0.3732621986400389, -1.4740870700600053, 2.6620633574955503, -1.8259236193966317, -2.0872895339053916)
        move_iiwa2(-0.3574996660273974, 1.1941742887324838, -0.3833240492361148, -1.551072926639862, 2.6500773324633986, -1.7654013219223874, -2.071374076657053)
        move_iiwa2(-0.38752062753668404, 1.1587193162838103, -0.411894401119775, -1.7122540949241947, 2.6127259279689152, -1.6255439465326273, -2.036030662542301)

        connector_iiwa2.lock() #picking up adacs card
        
        move_iiwa2(-0.37546120929331694, 1.0902625048855612, -0.4297354923142857, -1.746372571548005, 2.591241692109061, -1.6487905629911206, -2.061120489182747)
        move_iiwa2(-0.3238730643269186, 0.8225785705080699, -0.5058615829410056, -1.8269292403774902, 2.482484096830495, -1.7795683769720427, -2.185339057640259)
    #     #inter points universial
        move_iiwa2(0.18213337659835815, 0.5959590673446655, -0.412906289100647, -1.5645935535430908, 2.143127918243408, -2.0325796604156494, -2.185339057640259)
        move_iiwa2(-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        move_iiwa2(-0.8022392300153134, 0.4723393221573285, 0.4657909240700092, -1.6930174689322384, -0.25149838423568116, 1.0286846895757409, -0.2908820066021039)
        move_iiwa2(-0.7133057358897874, 0.420553569180271, 0.49501414852626563, -1.7719364557039263, -0.24359199099276763, 1.0037118549307755, -0.1637426268198841)
        move_iiwa2(-0.5623789517179739, 0.37993745127956197, 0.5396488808237749, -1.8358519657862353, -0.24301265685228676, 0.986886296377886, 0.04044728084774929)
        move_iiwa2(-0.5623789517179739, 0.37993745127956197, 0.5396488808237749, -1.8358519657862353, -0.24301265685228676, 0.986886296377886, 0.04044728084774929)
        move_iiwa2(-0.4048859392040555, 0.3813765044142337, 0.5763325720270103, -1.8425942910293158, -0.25803052060397896, 0.9896757913150496, 0.24156758510219856)

        connector_iiwa2.unlock()

        move_iiwa2(-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        iiwa2_gripper (-0.05,0.05)
        move_iiwa2(-0.7133057358897874, 0.420553569180271, 0.49501414852626563, -1.7719364557039263, -0.24359199099276763, 1.0037118549307755, -0.1637426268198841)
        move_iiwa2(-0.5623789517179739, 0.37993745127956197, 0.5396488808237749, -1.8358519657862353, -0.24301265685228676, 0.986886296377886, 0.04044728084774929)
        move_iiwa2(-0.4048859392040555, 0.3813765044142337, 0.5763325720270103, -1.8425942910293158, -0.25803052060397896, 0.9896757913150496, 0.24156758510219856)
        move_iiwa2(-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        iiwa2_gripper (0,0)
        move_iiwa2(-1.8344966788856316, 0.8176748764600984, 1.8211549231164652, -1.4951087182826748, -0.8226734663841251, 1.8015257293040383, 1.373635117167009)
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_iiwa2(-0.002094395,-0.1197296,-0.6544985,1.91951311,0.1635374,-1.0861184,-1.4987142)
        move_iiwa2(-0.83060219,-0.1197296,-0.6544985,1.91951311,0.1635374,-1.0861184,-1.4987142)
        move_iiwa2(-0.7807373075311612, -0.9122191247474123, -0.8819441872200112, 1.2149534294237714, -0.06840302171228774, -1.4845997142879956, -0.49810821725865173)
        move_iiwa2(-0.7597057541734445, -1.245870099685717, -0.9777484639522092, 0.9182752538879038, -0.16606454763737236, -1.652355786848462, -0.0768222913827501)
        move_iiwa2(-0.7386742008157279, -1.5795210746240218, -1.073552740684407, 0.6215970783520364, -0.2637260735624569, -1.8201118594089283, 0.3444636344931511)
        move_iiwa2(-0.9338509464732244, -1.723976112597616, -1.0746125185835682, 0.2241895022828482, -0.26524759339939763, -1.9818501971128297, 0.33872058830770596)
        move_iiwa2(-0.8371568071773258, -1.6358509286235452, -1.0703953912122601, 0.64891778531348, -0.26004992243144165, -1.6857985625698295, 0.38723105996442303)
   
        connector_iiwa2.lock()#pickup startracker

        move_iiwa2(-0.7752551866335202, -1.3584483418684514, -1.1650304585843583, 0.7687495849118791, -0.3625165648572414, -1.6711562453105966, 0.15252199040616873)
        move_iiwa2(-1.3129766459691332, 0.6730875471468423, 1.4248024109635367, -1.862666665803954, 2.398556903447481, -1.2477805399758883, 0.33831832056425915)
        move_iiwa2(-1.2128265934339901, 0.711850338070752, 1.363988233525985, -1.7778779053810112, 2.3745434018530904, -1.273894724329848, 0.34448560972129105)
        move_iiwa2(-1.144542452931464, 0.7502168949472972, 1.3155800667291844, -1.7306142095514752, 2.345835733419051, -1.2748804525846, 0.3542800921602489)
        move_iiwa2(-1.084758269681641, 0.7320581358135668, 1.320468781846213, -1.7544173977719457, 2.3591039004341203, -1.2646855345640535, 0.42975228068783594)
        move_iiwa2(-1.015721473040896, 0.71783211022488, 1.3207001254660125, -1.7691631199414042, 2.370609453269549, -1.2574311012236123, 0.5052470571084948)
        move_iiwa2(-0.8958888221829051, 0.7049964646446141, 1.3117052315395639, -1.7741513417895816, 2.3834973917015567, -1.2525514072969963, 0.6176346434720483)
        move_iiwa2(-0.7831687158980788, 0.7030680551767206, 1.2959444143237617, -1.7625734316020893, 2.3893840735450973, -1.2544215399756677, 0.7092377488610024)

        connector_iiwa2.unlock()

        # kuka_inserting the start tracker while S5 inserting the wifi card
        move_s5_iiwa2(-0.415, -1.004, -0.063, 0.398, 1.084, -1.005,-1.144542452931464, 0.7502168949472972, 1.3155800667291844, -1.7306142095514752, 2.345835733419051, -1.2748804525846, 0.3542800921602489)
        iiwa2_gripper (-0.05,0.05)
        move_s5_iiwa2(-0.40986274820131996, -1.2948950315186045, 0.08211882104117106, 0.3679177259727595, 1.2184295076598297, -0.9431270057771934,-1.084758269681641, 0.7320581358135668, 1.320468781846213, -1.7544173977719457, 2.3591039004341203, -1.2646855345640535, 0.42975228068783594)
        move_s5_iiwa2(-0.5123763088683295, -1.3934445606275214, 0.34644715672318005, 0.5100251111153655, 1.087139000159019, -1.0634865685372892,-1.015721473040896, 0.71783211022488, 1.3207001254660125, -1.7691631199414042, 2.370609453269549, -1.2574311012236123, 0.5052470571084948)
        move_s5_iiwa2(-0.48246738292919095, -1.4764512746946665, 0.555190650926489, 0.5135170666082708, 0.9693754165331561, -1.1184829166719084,-0.8958888221829051, 0.7049964646446141, 1.3117052315395639, -1.7741513417895816, 2.3834973917015567, -1.2525514072969963, 0.6176346434720483)
        star_tracker_insertion()

        connector_s5.lock() #grab WIFI card

        move_s5_iiwa2(-0.5123763088683295, -1.3934445606275214, 0.34644715672318005, 0.5100251111153655, 1.087139000159019, -1.0634865685372892,-1.144542452931464, 0.7502168949472972, 1.3155800667291844, -1.7306142095514752, 2.345835733419051, -1.2748804525846, 0.3542800921602489)
        iiwa2_gripper (0,0)
        move_s5(-0.4865211811071808, -1.2148167347358083, 0.38665519086553574, 0.5537529285057752, 0.890693639598321, -1.1803874254426248)
        move_s5(-0.40025497328339027, -0.9300319510046098, -0.2834059929597, 0.35788923217930424, 1.2181558881222578, -0.9396216016257749)
        move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
        # placing s5 back to the holder
        move_s5(-1.068,0.834,-0.54,0,-0.910,0.75)
        move_s5(-1.958,-0.854,0.850,0,-1.555,-2.01159265359)
        move_s5(-1.71726818450289, -0.3697217632676958, 0.09536755761636127, 0.003039789876150188, -1.2849087136414357, -2.2532468277787094)
        move_s5(-1.712,-0.459,-0.124,0.009,-1.005,-2.230)

        connector_s5.unlock()
 
        move_s5(-1.71726818450289, -0.3697217632676958, 0.09536755761636127, 0.003039789876150188, -1.2849087136414357, -2.2532468277787094)
        move_s5(-1.068,0.834,-0.54,0,-0.910,0.75)
        move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
        #iiwa2 pickup wifi
        move_iiwa2(-1.8344966788856316, 0.8176748764600984, 1.8211549231164652, -1.4951087182826748, -0.8226734663841251, 1.8015257293040383, 1.373635117167009)
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_iiwa2(-0.002094395,-0.1197296,-0.6544985,1.91951311,0.1635374,-1.0861184,-1.4987142)
        move_iiwa2(-0.83060219,-0.1197296,-0.6544985,1.91951311,0.1635374,-1.0861184,-1.4987142)
        move_iiwa2(-0.7807373075311612, -0.9122191247474123, -0.8819441872200112, 1.2149534294237714, -0.06840302171228774, -1.4845997142879956, -0.49810821725865173)
        move_iiwa2(-0.7597057541734445, -1.245870099685717, -0.9777484639522092, 0.9182752538879038, -0.16606454763737236, -1.652355786848462, -0.0768222913827501)
        move_iiwa2(-0.7386742008157279, -1.5795210746240218, -1.073552740684407, 0.6215970783520364, -0.2637260735624569, -1.8201118594089283, 0.3444636344931511)
        move_iiwa2(-0.9338509464732244, -1.723976112597616, -1.0746125185835682, 0.2241895022828482, -0.26524759339939763, -1.9818501971128297, 0.33872058830770596)
        move_iiwa2(-0.8371568071773258, -1.6358509286235452, -1.0703953912122601, 0.64891778531348, -0.26004992243144165, -1.6857985625698295, 0.38723105996442303)

        connector_iiwa2.lock() #picking up wifi

        move_iiwa2(-0.7752551866335202, -1.3584483418684514, -1.1650304585843583, 0.7687495849118791, -0.3625165648572414, -1.6711562453105966, 0.15252199040616873)
        move_iiwa2(-1.3129766459691332, 0.6730875471468423, 1.4248024109635367, -1.862666665803954, 2.398556903447481, -1.2477805399758883, 0.33831832056425915)
        move_iiwa2(-1.2128265934339901, 0.711850338070752, 1.363988233525985, -1.7778779053810112, 2.3745434018530904, -1.273894724329848, 0.34448560972129105)
        move_iiwa2(-1.2164800419389947, 0.6827884565165866, 1.4023919247797787, -1.707598707876776, 2.4165006416938533, -1.3516964131761693, 0.3129560661542958)
       
        #cartesian insert wifi
        move_iiwa2(-1.2164800419389947, 0.6827884565165866, 1.4023919247797787, -1.707598707876776, 2.4165006416938533, -1.3516964131761693, 0.3129560661542958)
        move_iiwa2(-1.1430505235267578, 0.6561644343737831, 1.409033502541079, -1.735827111172503, 2.4390262519354367, -1.3379421948438694, 0.40448104207211105)
        move_iiwa2(-1.0526123843846165, 0.635806931509248, 1.405764829123053, -1.7499183908071703, 2.457643258016603, -1.3285784374036267, 0.49672372755865296)
        move_iiwa2(-1.0117643224188582, 0.6297564783238618, 1.4014876912510856, -1.7515717171343421, 2.4636762440777344, -1.3261950896973345, 0.533739155770002)
        move_iiwa2(-0.8746043494620462, 0.6200573355644099, 1.378771606324591, -1.7428536624569622, 2.475838663561989, -1.32423717471021, 0.6446067135720597)

        connector_iiwa2.unlock()
   
        move_iiwa2(-1.2164800419389947, 0.6827884565165866, 1.4023919247797787, -1.707598707876776, 2.4165006416938533, -1.3516964131761693, 0.3129560661542958)
        move_iiwa2(-1.1908281445429476, 0.7210450513557736, 1.351040522633058, -1.7574996822942108, 2.369328885279234, -1.2801698473036056, 0.345193455460204)
        iiwa2_gripper (-0.05,0.05)
        move_iiwa2(-1.1892864235405871, 0.72322483262866, 1.3481339706686692, -1.7599155882314579, 2.366547264662839, -1.2765316447788841, 0.3468770191188402)
        move_iiwa2(-1.160448712732756, 0.7128645396482777, 1.351528181072719, -1.7730095417277516, 2.3743635643673606, -1.2707299465301438, 0.3846525737668533)
        move_iiwa2(-0.8188006975408825, 0.669009188650749, 1.3252102194197708, -1.7920159754187936, 2.417256820978998, -1.2526582241804958, 0.7055324316012814)
        move_iiwa2(-1.2164800419389947, 0.6827884565165866, 1.4023919247797787, -1.707598707876776, 2.4165006416938533, -1.3516964131761693, 0.3129560661542958)
        iiwa2_gripper (0,0)
        #motion planning to get c3 card
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_iiwa2(-1.7532628984474183, 0.8652596269516487, 1.6800677080733553, -1.5774361441767086, -0.8056357582632172, 1.6015811073155342, 1.2647175007363676)
        move_iiwa2(-1.589796449245989, 0.8644091606629589, 1.5028827609155828, -1.5369744054774217, -0.7155461669478322, 1.419209204722581, 1.0309760434821897)
        move_iiwa2(-1.4590232898848459, 0.863728787632007, 1.3611348031893649, -1.5046050145179921, -0.6434744938955242, 1.2733116826482185, 0.8439828776788476)
        move_iiwa2(-1.3936367102042744, 0.863388601116531, 1.2902608243262557, -1.4884203190382772, -0.6074386573693703, 1.200362921611037, 0.7504862947771764)
        move_iiwa2(-1.0340105219611302, 0.8615175752814134, 0.9004539405791563, -1.399404493899846, -0.4092415564755234, 0.79914473590654, 0.2362550888179853)
        move_iiwa2(-0.8378507829194155, 0.8604970157349856, 0.6878320039898294, -1.3508504074607015, -0.30113404689706147, 0.5802984527949961, -0.044234659887028016)
        move_iiwa2(-0.7724642032388438, 0.8601568292195096, 0.6169580251267204, -1.3346657119809868, -0.2650982103709075, 0.5073496917578151, -0.13773124278869897)
        move_iiwa2(-0.5109178845165574, 0.8587960831576058, 0.33346210967428447, -1.2699269300621276, -0.12095486426629165, 0.21555464760909016, -0.5117175743953832)
        move_iiwa2(-0.41283801499570005, 0.858285803384392, 0.227151141379621, -1.2456498868425554, -0.06690110947706074, 0.10613150605331811, -0.6519624487478899)
        move_iiwa2(-0.3474514353151281, 0.857945616868916, 0.15627716251651202, -1.2294651913628405, -0.030865272950906686, 0.03318274501613683, -0.7454590316495613)
        move_iiwa2(-0.29383022223987454, 0.93607413900739, -0.006859630257084956, -1.2498685963753742, 0.5521496765414957, -0.4187470891119939, -1.0984631659140136)
        move_iiwa2(-0.3394877093837656, 1.2401325961292857, -0.36446643412940677, -1.391713804943699, 2.6716558305152387, -1.8872899505403795, -2.104101942606463)
        move_iiwa2(-0.3394803615971062, 1.240212700330852, -0.3644027640047344, -1.3916198125568475, 2.6716591125251887, -1.8872762491628499, -2.1041216978057817)
        move_iiwa2(-0.3475541547853751, 1.2152803124778528, -0.3732621986400389, -1.4740870700600053, 2.6620633574955503, -1.8259236193966317, -2.0872895339053916)
        move_iiwa2(-0.3574996660273974, 1.1941742887324838, -0.3833240492361148, -1.551072926639862, 2.6500773324633986, -1.7654013219223874, -2.071374076657053)
        move_iiwa2(-0.38752062753668404, 1.1587193162838103, -0.411894401119775, -1.7122540949241947, 2.6127259279689152, -1.6255439465326273, -2.036030662542301)
        
        #grabing c3 card
        move_iiwa2(-0.4175739702023292, 1.1446673362671076, -0.43976160638223943, -1.820901338898834, 2.5745728152869445, -1.5167403164501412, -2.0086150743305304)
        
        connector_iiwa2.lock()#grabbing c3 card
 
        move_iiwa2(-0.3143331969970972, 0.674056865121637, -0.5905457908798697, -1.9454559924320607, 2.380975582725732, -1.7547294343998352, -2.232793240797215)
        #intermide motion planning
        move_iiwa2(0.18213337659835815, 0.5959590673446655, -0.412906289100647, -1.5645935535430908, 2.143127918243408, -2.0325796604156494, -2.185339057640259)
        move_iiwa2(-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        move_iiwa2(-0.8022392300153134, 0.4723393221573285, 0.4657909240700092, -1.6930174689322384, -0.25149838423568116, 1.0286846895757409, -0.2908820066021039)
        move_iiwa2(-0.810609093845846, 0.3941851149792085, 0.4936346834631587, -1.5563003031130767, -0.2037578615777187, 1.235035216843221, -0.3225871033473701)
        #cartesian insertion
        move_iiwa2(-0.767520028670185, 0.35933887398008324, 0.5133525377665227, -1.6043996487428407, -0.19503696222781264, 1.222102121908848, -0.25538813997281584)
        move_iiwa2(-0.7324322507047493, 0.33732639770496947, 0.5290652451343889, -1.6344034496519035, -0.189717029760804, 1.2146402093691382, -0.20217161376576462)
        move_iiwa2(-0.5685192276364339, 0.2896203833084879, 0.5969279875632051, -1.7023043804782694, -0.18440771288307264, 1.201409218534279, 0.03572138390269422)
        move_iiwa2(-0.42073953448680473, 0.3014444345908689, 0.6456337121752286, -1.6980233333122072, -0.20390389417321653, 1.2072596945498122, 0.23630150160520794)
    
        connector_iiwa2.unlock()
    
        move_iiwa2(-0.810609093845846, 0.3941851149792085, 0.4936346834631587, -1.5563003031130767, -0.2037578615777187, 1.235035216843221, -0.3225871033473701)
        iiwa2_gripper (-0.05,0.05)
        move_iiwa2(-0.767520028670185, 0.35933887398008324, 0.5133525377665227, -1.6043996487428407, -0.19503696222781264, 1.222102121908848, -0.25538813997281584)
        move_iiwa2(-0.7324322507047493, 0.33732639770496947, 0.5290652451343889, -1.6344034496519035, -0.189717029760804, 1.2146402093691382, -0.20217161376576462)
        move_iiwa2(-0.5685192276364339, 0.2896203833084879, 0.5969279875632051, -1.7023043804782694, -0.18440771288307264, 1.201409218534279, 0.03572138390269422)
        move_iiwa2(-0.42073953448680473, 0.3014444345908689, 0.6456337121752286, -1.6980233333122072, -0.20390389417321653, 1.2072596945498122, 0.23630150160520794)
        move_iiwa2(-0.767520028670185, 0.35933887398008324, 0.5133525377665227, -1.6043996487428407, -0.19503696222781264, 1.222102121908848, -0.25538813997281584)
        iiwa2_gripper (0,0)
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        
        #motion planning to get card_with_connector card
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        move_iiwa2(-1.7532628984474183, 0.8652596269516487, 1.6800677080733553, -1.5774361441767086, -0.8056357582632172, 1.6015811073155342, 1.2647175007363676)
        move_iiwa2(-1.589796449245989, 0.8644091606629589, 1.5028827609155828, -1.5369744054774217, -0.7155461669478322, 1.419209204722581, 1.0309760434821897)
        move_iiwa2(-1.4590232898848459, 0.863728787632007, 1.3611348031893649, -1.5046050145179921, -0.6434744938955242, 1.2733116826482185, 0.8439828776788476)
        move_iiwa2(-1.3936367102042744, 0.863388601116531, 1.2902608243262557, -1.4884203190382772, -0.6074386573693703, 1.200362921611037, 0.7504862947771764)
        move_iiwa2(-1.0340105219611302, 0.8615175752814134, 0.9004539405791563, -1.399404493899846, -0.4092415564755234, 0.79914473590654, 0.2362550888179853)
        move_iiwa2(-0.8378507829194155, 0.8604970157349856, 0.6878320039898294, -1.3508504074607015, -0.30113404689706147, 0.5802984527949961, -0.044234659887028016)
        move_iiwa2(-0.7724642032388438, 0.8601568292195096, 0.6169580251267204, -1.3346657119809868, -0.2650982103709075, 0.5073496917578151, -0.13773124278869897)
        move_iiwa2(-0.5109178845165574, 0.8587960831576058, 0.33346210967428447, -1.2699269300621276, -0.12095486426629165, 0.21555464760909016, -0.5117175743953832)
        move_iiwa2(-0.41283801499570005, 0.858285803384392, 0.227151141379621, -1.2456498868425554, -0.06690110947706074, 0.10613150605331811, -0.6519624487478899)
        move_iiwa2(-0.3474514353151281, 0.857945616868916, 0.15627716251651202, -1.2294651913628405, -0.030865272950906686, 0.03318274501613683, -0.7454590316495613)
        move_iiwa2(-0.29383022223987454, 0.93607413900739, -0.006859630257084956, -1.2498685963753742, 0.5521496765414957, -0.4187470891119939, -1.0984631659140136)
        move_iiwa2(-0.3394877093837656, 1.2401325961292857, -0.36446643412940677, -1.391713804943699, 2.6716558305152387, -1.8872899505403795, -2.104101942606463)
        move_iiwa2(-0.3394803615971062, 1.240212700330852, -0.3644027640047344, -1.3916198125568475, 2.6716591125251887, -1.8872762491628499, -2.1041216978057817)

        #getting card with connector
        move_iiwa2(-0.4313097563845256, 1.142389235165169, -0.45243794418677113, -1.8605686900094394, 2.556849596028324, -1.4728330910978427, -1.9972876552384466)
        move_iiwa2(-0.46203278615821414, 1.1438141183078656, -0.48088505788665475, -1.9333248139831793, 2.5171953327841896, -1.3838370027817382, -1.9735320283214994)
    
        connector_iiwa2.lock() #grabbing card with connector

        move_iiwa2(-0.36868395687957595, 0.7581248634631086, -0.6252240612098656, -2.0574196415468355, 2.3732833337502575, -1.5564476546601416, -2.1488530566653807)

        #motion planning result generated by KDL solver
        move_iiwa2(0.18213337659835815, 0.5959590673446655, -0.412906289100647, -1.5645935535430908, 2.143127918243408, -2.0325796604156494, -2.185339057640259)
        move_iiwa2(-0.7878259753596926, 0.5226441884768438, 0.44856430021208527, -1.7226796025138071, -0.2798358434995873, 0.953122393944973, -0.2693257552050775)
        move_iiwa2(-0.810609093845846, 0.3941851149792085, 0.4936346834631587, -1.5563003031130767, -0.2037578615777187, 1.235035216843221, -0.3225871033473701)
        move_iiwa2(-0.8140871630405938, 0.3715314053508643, 0.4992439503151433, -1.511853467167299, -0.1911088786483023, 1.2992582096311467, -0.3331087875672249)
        move_iiwa2(-0.8140871630405938, 0.3715314053508643, 0.4992439503151433, -1.511853467167299, -0.1911088786483023, 1.2992582096311467, -0.3331087875672249)
        
        #cartisean 
        move_iiwa2(-0.8040812579472022, 0.36198897434361565, 0.5041895503550412, -1.5247682739447095, -0.1885034653645363, 1.2958652289177925, -0.3169157640089162)
        move_iiwa2(-0.7385564573653197, 0.31299989823688895, 0.5359186090525719, -1.5899628632715628, -0.17524095032280337, 1.2800119962642675, -0.2142203047930574)
        move_iiwa2(-0.5942421420192592, 0.2650230523679322, 0.601248096860812, -1.6552154681607505, -0.16702424626954687, 1.2679735321589922, 0.0004569606731565294)
        move_iiwa2(-0.4350580004958742, 0.2757707393480309, 0.6603028405878444, -1.653418495884653, -0.18696361424611185, 1.2733801739153867, 0.22210915462649433)
   
        connector_iiwa2.unlock()

        move_iiwa2(-0.8140871630405938, 0.3715314053508643, 0.4992439503151433, -1.511853467167299, -0.1911088786483023, 1.2992582096311467, -0.3331087875672249)
        iiwa2_gripper (-0.05,0.05)
        move_iiwa2(-0.8040812579472022, 0.36198897434361565, 0.5041895503550412, -1.5247682739447095, -0.1885034653645363, 1.2958652289177925, -0.3169157640089162)
        move_iiwa2(-0.7385564573653197, 0.31299989823688895, 0.5359186090525719, -1.5899628632715628, -0.17524095032280337, 1.2800119962642675, -0.2142203047930574)
        move_iiwa2(-0.5942421420192592, 0.2650230523679322, 0.601248096860812, -1.6552154681607505, -0.16702424626954687, 1.2679735321589922, 0.0004569606731565294)
        move_iiwa2(-0.4350580004958742, 0.2757707393480309, 0.6603028405878444, -1.653418495884653, -0.18696361424611185, 1.2733801739153867, 0.22210915462649433)
        move_iiwa2(-0.8140871630405938, 0.3715314053508643, 0.4992439503151433, -1.511853467167299, -0.1911088786483023, 1.2992582096311467, -0.3331087875672249)
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
    
        #pickup antenna 
        iiwa2_gripper (-0,0)
        move_abb(-2.8,0,0,0,0,0)
        move_iiwa2(-0.771,0.884,2.908,-1.602,-0.860,-1.711,1.405)
        # move_iiwa2(-0.4350580004958742, 0.2757707393480309, 0.6603028405878444, -1.653418495884653, -0.18696361424611185, 1.2733801739153867, 0.22210915462649433)
        move_iiwa2(1.0590657901101592, 0.7161085920932735, -0.4274311338134113, -0.5878268920716901, 0.4703662334124718, 1.5988961277520053, 0.8192575508861383)
        move_iiwa2(0.938463538797351, 1.1868238913561442, -0.27977627909469105, -0.27960174616949157, 0.28361600344907856, 1.6599826515718068, 0.9253735694073936)
        move_iiwa2(0.9991710999016039, 1.2166086907485294, -0.27820759128232836, -0.21995481196211988, 0.2842877938525927, 1.6861404986830064, 1.0026258502291026)
        move_iiwa2(1.0163304375407625, 1.194759823442505, -0.2802277245243474, -0.3419524852077291, 0.2818364727713683, 1.590649225518856, 0.9852362364452769)
    
        if (iiwa2_special_connector.getPresence()== 1):
            iiwa2_special_connector.lock()#grabbing antenna

        move_iiwa2(0.9991710999016039, 1.2166086907485294, -0.27820759128232836, -0.21995481196211988, 0.2842877938525927, 1.6861404986830064, 1.0026258502291026)
        move_iiwa2(0.938463538797351, 1.1868238913561442, -0.27977627909469105, -0.27960174616949157, 0.28361600344907856, 1.6599826515718068, 0.9253735694073936)
        #antenna delivery
        move_iiwa2(0.873892679773605, 0.7321556605496228, -0.32155112421155635, -0.5847757175850665, 0.2489626472127175, 1.8231840160869635, 0.7602413582770322)
        move_iiwa2(0.760627948211075, 0.3557888403441205, -0.39520509577825425, -1.1633597096509227, 0.16662752026048086, 1.634905103344048, 0.4618943341363667)
        move_iiwa2(0.18543027327404404, 0.36514067619604945, -0.8428753812211408, -1.2387776684367382, 0.29847040082759607, 1.670725426399168, 2.505)
        #cartisan path for insertion
        move_iiwa2(0.2103870828801063, 0.3842807932624192, -0.8302984703782562, -1.4433902482950358, 0.31032818804650253, 1.4578595788022937, 2.479785006697003)
        move_iiwa2(0.30525846209456337, 0.34079831897627555, -0.8252075928283554, -1.488628694728669, 0.2800736719203658, 1.439629019409783, 2.571391183113379)
        move_iiwa2(0.3730726396923048, 0.31244843072729134, -0.8174883504911968, -1.5158822673754933, 0.2594632495889409, 1.4283081226512317, 2.6429803790663646)
        move_iiwa2(0.5517412880386071, 0.255043186453883, -0.7727271092674919, -1.5637260287384118, 0.21309223358125495, 1.405079933532898, 2.862140424925185)
        move_iiwa2(0.653842139497969, 0.24480984617309678, -0.7365617176814503, -1.5674785644631226, 0.2004184243408291, 1.3978864459086964, 3.0007072254611433)

        iiwa2_special_connector.unlock()

        antenna_insertion ()
        move_iiwa2(0.2103870828801063, 0.3842807932624192, -0.8302984703782562, -1.4433902482950358, 0.31032818804650253, 1.4578595788022937, 2.479785006697003)
        iiwa2_gripper (-0.05,0.05)
        move_iiwa2(0.2061811016710163, 0.4176114307929445, -0.8158950162172619, -1.535109956337328, 0.33686588940154166, 1.3442929736799574, 2.454593086474859)
        move_iiwa2(0.4528401011690704, 0.3200953220176893, -0.7969572633092955, -1.6349011449279587, 0.268862044801138, 1.3004606756892798, 2.7096937727299184)
        move_iiwa2(0.47340594782224465, 0.3140540815963943, -0.7933852884084717, -1.640230269632912, 0.26413019992573805, 1.2977466998666705, 2.733727293260705)
        move_iiwa2(0.5134170037379856, 0.303622794142728, -0.7854387574889181, -1.649013710534221, 0.2556354677348466, 1.2929191669488622, 2.781699093107104)
        move_iiwa2(0.6241214658933427, 0.28630615230952217, -0.7564058981625922, -1.660332024506846, 0.23860357043783037, 1.283060885370422, 2.9225531354632235)
        move_iiwa2(0.6741820432717557, 0.28576109520627097, -0.7399020339540748, -1.657522852735302, 0.2350111873330958, 1.2806453975111796, 2.9899877322608592)
        move_iiwa2(0.2103870828801063, 0.3842807932624192, -0.8302984703782562, -1.4433902482950358, 0.31032818804650253, 1.4578595788022937, 2.479785006697003)
        move_iiwa2(-1.851410557122822, 0.8657033344853333, 1.7864513874396701, -1.601782267760985, -0.8597793085422902, 1.7110267049204335, 1.405)
        iiwa2_gripper (0,0)
        move_iiwa2(0,0,0,0,0,0,0)
        #human operation for frame 4
        human_on()
        frame4_start()
        i_want_to_wait(7)
        frame4_finished()
        i_want_to_wait(10)
        human_off()
        
    #iiwa1 grabbing solar module
    
        move_iiwa1(0.274114164189028, 1.3528982428390415, 0.8167755188264708, -1.1098877852586346, 0.3106153773083506, 1.9833900037516454, -0.5640792813341853)
        move_iiwa1(0.17273568525195646, 1.3493876513770473, 0.7820581416519324, -1.2343459643001855, 0.2337967165759507, 1.9550715431490673, -0.6605171158346975)
        
        #begain cartsitan to grab solar 1 
        move_iiwa1(0.0926730814531301, 1.4064597229141336, 0.9418959163414617, -1.3309300857920914, 0.1356345730414388, 1.8621438930667973, -0.5589070237752715)
        move_iiwa1(0.0807768721017844, 1.3816664634770865, 0.951995203119097, -1.4755575597232313, 0.145791076308548, 1.7412815669213735, -0.5659941669170978)
        move_iiwa1(0.10541280199688127, 1.3569826692090747, 0.9837633546001976, -1.778882009515814, 0.18201974307496246, 1.4306003000400207, -0.5819853728942274)
        move_iiwa1(0.17136142596343606, 1.3746127769815255, 1.0108926897717616, -1.9361314222190555, 0.21594715043069276, 1.2073719742508409, -0.5934392696015109)
        move_iiwa1(0.202892596746628, 1.3872507142192438, 1.0212756363800868, -1.978942863358589, 0.22897733152830224, 1.1305770388768324, -0.5977986042983559)
   
        connector_iiwa1.lock() #grab solar 1 
    
        move_iiwa1(0.07096822947436097, 1.1074046329607963, 1.345289551634842, -2.0488601527609034, 0.456180209105014, 1.2206550380135026, -0.3782053294478712)
        move_iiwa1(-0.036131184601100604, 1.186577086399863, 1.2296519836639723, -1.5731667216597385, 0.3065196130820024, 1.7765062856624956, -0.2983366820927498)
        move_iiwa1(0,0,0,0,0,0,0)
        
        #cartesian placement for solar module
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
        
        connector_iiwa1.unlock()
    
        #immoblized solar module
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.5187504761414474, -0.44312555153976646, -1.2687875804174655, 1.5537252449925654, 0.4265736856165881, -1.4593607977784373, -0.3024032845165509)
    
        abb_screw_pickup_cycyles()
    
        #first screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(1.8486735029068402, -0.551806289577936, 0.5726434708129883, 0.0006252086593768184, 1.5509951206054322, 0.6924174024332395)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
    
        #second screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(2.282996903940546, -0.3846278558163751, 0.4720667501182722, 0.0009397685347604525, 1.484049177592619, 1.126706491834365)
        move_abb(2.2829967490127507, -0.32085506824614396, 0.19480113476883712, 0.0009442512365445149, 1.6975419114936843, 1.1269070963373466)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
    
        #kuka_move_away 
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
        move_iiwa1(0,0,0,0,0,0,0)
    
        #third screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(2.54542420722709, -0.1177363047951821, 0.26316029578649996, 0.001150990495610996, 1.4258451354151505, 1.389042105809328)
        move_abb(2.5454242073417976, -0.0670718081857006, -0.014029442584627953, 0.0011427194562357611, 1.6523702294777791, 1.3893014726433797)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)     
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
    
        #fourth screw
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(2.685623527411623, 0.16634227948369992, -0.02695035754527958, 0.0011560852286533721, 1.4315255049485036, 1.5293776239522412)
        move_abb(2.685623527522499, 0.22491315815949636, -0.32707577414757516, 0.0011509059701854528, 1.6730798837333964, 1.5296556268889172)
        move_abb(1.8486733861738345, -0.4716185882525836, 0.2910647441099619, 0.0006358021575295636, 1.7523861062823283, 0.6925444913859231)
        move_abb(-0.058,-0.499,0.315,0.001,1.076,0.513)
        abb_screw_pickup_cycyles()
    
        #human rotation operation
        solar_one_finshed_half()
        human_on()
        i_want_to_wait(5)
        human_off()
    
        abb_solar_module_4_screws()
        human_on()
        solar_one_finshed()
        i_want_to_wait(5)
        human_off()
   
        #pickup solar module 2
        move_iiwa1(0.274114164189028, 1.3528982428390415, 0.8167755188264708, -1.1098877852586346, 0.3106153773083506, 1.9833900037516454, -0.5640792813341853)
        # move_iiwa1(0.17273568525195646, 1.3493876513770473, 0.7820581416519324, -1.2343459643001855, 0.2337967165759507, 1.9550715431490673, -0.6605171158346975)
        move_iiwa1(0.4187374622341085, 1.2365199354623566, 0.96500304631246, -1.8940505008486246, 0.49119263747990427, 1.1530209644217153, -0.6898622529772007)
        move_iiwa1(0.32284794213113854, 1.2844186992822992, 0.9235258842404973, -2.0178322913398783, 0.408285896294785, 1.0962506408865103, -0.7578468548750482)
        move_iiwa1(0.362635332672711, 1.3043788419554128, 0.9427559593422122, -2.0510635877094097, 0.43289648076396864, 1.017294290500458, -0.7682649529387409)
    
        connector_iiwa1.lock() #grab solar 2
    
        move_iiwa1(0.28836757116874434, 0.9679626109714596, 1.298866319004364, -1.9787909725066857, 0.6699251753971475, 1.1654624036782781, -0.43998520820141196)
        move_iiwa1(0.116113397309139, 1.106591342065707, 1.1292351102366913, -1.4164956887886644, 0.43449175354912595, 1.8409937962822447, -0.30304347361236555)
        move_iiwa1(0,0,0,0,0,0,0)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
   
        connector_iiwa1.unlock()

        #immoblized solar module
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.5187504761414474, -0.44312555153976646, -1.2687875804174655, 1.5537252449925654, 0.4265736856165881, -1.4593607977784373, -0.3024032845165509)
   
        #abb_screw
        abb_solar_module_1_2_screws()
   
        #kuka_move_away 
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
        move_iiwa1(0,0,0,0,0,0,0)
        abb_solar_module_3_4_screws()
        
        #human rotation + ABB Screwing
        human_on()
        solar_two_finshed_half()
        i_want_to_wait(5)
        human_off()
        abb_solar_module_4_screws()
        human_on()
        solar_two_finshed()
        i_want_to_wait(5)
        human_off()
    
        #pickup solar module 3
        move_iiwa1(0.274114164189028, 1.3528982428390415, 0.8167755188264708, -1.1098877852586346, 0.3106153773083506, 1.9833900037516454, -0.5640792813341853)
        # move_iiwa1(0.17273568525195646, 1.3493876513770473, 0.7820581416519324, -1.2343459643001855, 0.2337967165759507, 1.9550715431490673, -0.6605171158346975)
        move_iiwa1(0.4187374622341085, 1.2365199354623566, 0.96500304631246, -1.8940505008486246, 0.49119263747990427, 1.1530209644217153, -0.6898622529772007)
        move_iiwa1(0.32284794213113854, 1.2844186992822992, 0.9235258842404973, -2.0178322913398783, 0.408285896294785, 1.0962506408865103, -0.7578468548750482)
        move_iiwa1(0.362635332672711, 1.3043788419554128, 0.9427559593422122, -2.0510635877094097, 0.43289648076396864, 1.017294290500458, -0.7682649529387409)
        move_iiwa1(0.5059334854767669, 1.3582159132998857, 0.9933416408314079, -2.0198109005506995, 0.5004045079232952, 0.8886665934606159, -0.7656270128505559)
        
        connector_iiwa1.lock()#grab solar 3

        move_iiwa1(0.4501242333998944, 1.0133344633670573, 1.3153125992762575, -1.938629961413916, 0.6952107740489952, 1.0227612072104997, -0.42507916653179234)
        move_iiwa1(0.116113397309139, 1.106591342065707, 1.1292351102366913, -1.4164956887886644, 0.43449175354912595, 1.8409937962822447, -0.30304347361236555)
        move_iiwa1(0,0,0,0,0,0,0)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
    
        connector_iiwa1.unlock()
    
        #immoblized solar module
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.5187504761414474, -0.44312555153976646, -1.2687875804174655, 1.5537252449925654, 0.4265736856165881, -1.4593607977784373, -0.3024032845165509)
    
        abb_solar_module_1_2_screws()
    
        #kuka_move_away 
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
        move_iiwa1(0,0,0,0,0,0,0)
        abb_solar_module_3_4_screws()
        
        #human rotation + abb screwing
        human_on()
        solar_three_finshed_half()
        i_want_to_wait(5)
        human_off()
        abb_solar_module_4_screws()
        human_on()
        solar_three_finshed()
        i_want_to_wait(5)
        human_off()
    
        #pickup solar module 4
        move_iiwa1(0.274114164189028, 1.3528982428390415, 0.8167755188264708, -1.1098877852586346, 0.3106153773083506, 1.9833900037516454, -0.5640792813341853)
        move_iiwa1(0.4187374622341085, 1.2365199354623566, 0.96500304631246, -1.8940505008486246, 0.49119263747990427, 1.1530209644217153, -0.6898622529772007)
        move_iiwa1(0.32284794213113854, 1.2844186992822992, 0.9235258842404973, -2.0178322913398783, 0.408285896294785, 1.0962506408865103, -0.7578468548750482)
        move_iiwa1(0.362635332672711, 1.3043788419554128, 0.9427559593422122, -2.0510635877094097, 0.43289648076396864, 1.017294290500458, -0.7682649529387409)
        move_iiwa1(0.5059334854767669, 1.3582159132998857, 0.9933416408314079, -2.0198109005506995, 0.5004045079232952, 0.8886665934606159, -0.7656270128505559)
        move_iiwa1(0.6091824184468476, 1.4170421149809442, 1.0412207380619445, -2.0422149963364555, 0.5515167959432676, 0.7383874905362103, -0.7926042636500827)
    
        connector_iiwa1.lock() #grab solar 4
    
        move_iiwa1(0.5597067314120505, 1.048513792212997, 1.356313952578829, -1.9601441334120602, 0.72442366872, 0.8688530278454538, -0.4333634038699449)
        move_iiwa1(0.116113397309139, 1.106591342065707, 1.1292351102366913, -1.4164956887886644, 0.43449175354912595, 1.8409937962822447, -0.30304347361236555)
        move_iiwa1(0,0,0,0,0,0,0)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
    
        connector_iiwa1.unlock()
    
        #immoblized solar module
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.5187504761414474, -0.44312555153976646, -1.2687875804174655, 1.5537252449925654, 0.4265736856165881, -1.4593607977784373, -0.3024032845165509)    
    
        abb_solar_module_1_2_screws()
    
        #kuka_move_away 
        move_iiwa1(2.51936480426423, -0.43554089625305703, -1.2721743419722709, 1.5336075950160573, 0.4187811958685798, -1.481227236327701, -0.2956475739852107)
        move_iiwa1(2.4948915771886377, -0.40417123566573704, -1.2913595001735503, 1.3485280709153553, 0.3915256424644876, -1.668656827140235, -0.2648259359457797)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
        move_iiwa1(0,0,0,0,0,0,0)
        abb_solar_module_3_4_screws()
    
        #human rotation + abb screwing
        human_on()
        solar_four_finshed_half()
        i_want_to_wait(5)
        human_off()
        abb_solar_module_4_screws()
        human_on()
        solar_four_finshed()
        i_want_to_wait(5)
        human_off()
    
        #pickup end card negative
        move_iiwa1(0.274114164189028, 1.3528982428390415, 0.8167755188264708, -1.1098877852586346, 0.3106153773083506, 1.9833900037516454, -0.5640792813341853)
        move_iiwa1(0.4187374622341085, 1.2365199354623566, 0.96500304631246, -1.8940505008486246, 0.49119263747990427, 1.1530209644217153, -0.6898622529772007)
        move_iiwa1(0.32284794213113854, 1.2844186992822992, 0.9235258842404973, -2.0178322913398783, 0.408285896294785, 1.0962506408865103, -0.7578468548750482)
        move_iiwa1(0.362635332672711, 1.3043788419554128, 0.9427559593422122, -2.0510635877094097, 0.43289648076396864, 1.017294290500458, -0.7682649529387409)
        move_iiwa1(0.5059334854767669, 1.3582159132998857, 0.9933416408314079, -2.0198109005506995, 0.5004045079232952, 0.8886665934606159, -0.7656270128505559)
        move_iiwa1(0.6091824184468476, 1.4170421149809442, 1.0412207380619445, -2.0422149963364555, 0.5515167959432676, 0.7383874905362103, -0.7926042636500827)
        move_iiwa1(0.7570478420729008, 1.5077157104261316, 1.1109197060631573, -2.034444711942313, 0.6120758191548569, 0.5568028397041793, -0.83094765349212)
    
        connector_iiwa1.lock() #grab end card negative

        move_iiwa1(0.724733630617584, 1.0997553435878955, 1.393276194195795, -1.9525932922418086, 0.7614642057545516, 0.6843514843198513, -0.45246840579181635)
        move_iiwa1(0.49125846966339265, 1.0578450465795075, 1.3594994756909946, -1.9514480816396647, 0.6723775660484055, 0.9318034266070772, -0.4004667575889291)
        move_iiwa1(0.15180870116486045, 1.3467118141471506, 1.3793784328128245, -1.1676500105886654, 0.22287694747436926, 1.9114878229823256, -0.06743501622917722)
        move_iiwa1(0,0,0,0,0,0,0)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
    
        connector_iiwa1.unlock()
    
        #kuka move away 
        move_iiwa1(0,0,0,0,0,0,0)
    
        # abb screw_endcard_neg
        abb_screwing_endcap_4_screws()
        human_on()
        endcard_negative_finished_half()
        i_want_to_wait(5)
        human_off()
        abb_screwing_endcap_4_screws()
        human_on()
        endcard_negative_finished()
        i_want_to_wait(5)
        human_off()

        #pick up end card positive
        move_iiwa1(-2.077,-0.584,0.593,2.058,-1.496,1.372,-1.249)
        move_iiwa1(-1.5278683600476375, -0.7795866973975736, 0.19048607523270694, 2.0564260105421854, -1.5146848567987103, 1.3859080833140753, -1.2487660726396328)
        move_iiwa1(-1.512704123568405, -0.7827613881560882, 0.2011401766343559, 2.050161811958376, -1.4960139500940166, 1.371507968119456, -1.2486819536252105)
    
        connector_iiwa1.lock() #grab end card positive
    
        move_iiwa1(0,0,0,0,0,0,0)
        move_iiwa1(2.6684083450539386, -0.448957093189538, -1.2773969723557186, 1.3051745997577815, 0.4335154363270688, -1.686802636010664, -0.0569631177530032)
    
        connector_iiwa1.unlock()
    
        #iwwa move out
        move_iiwa1(0,0,0,0,0,0,0)
        # abb screw_endcard_pos
        abb_screwing_endcap_4_screws()
        human_on()
        endcard_positive_finished_half()
        i_want_to_wait(5)
        human_off()
        abb_screwing_endcap_4_screws()
        human_on()
        endcard_positive_finished()

    k += 1
pass


    