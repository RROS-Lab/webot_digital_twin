"""all_controler controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from xml.etree.ElementTree import PI
from controller import Robot
from controller import Connector
from controller import Supervisor
from controller import Keyboard
from time import time, sleep
import math


pi_constant = math.pi
degree_to_radian = pi_constant/180
# from robotiq_gripper.msg import grip_state

# create the Robot instance.
robot = Supervisor()
keyboard_input = Keyboard()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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

connector_iiwa1 = Connector("connector_iiwa1")
connector_iiwa1.enablePresence(timestep)
connector_iiwa2 = Connector("connector_iiwa2")
connector_iiwa2.enablePresence(timestep)
connector_c3 = Connector("connector_c3")
connector_c3.enablePresence(timestep)
connector_s5 = Connector("connector_s5")
connector_s5.enablePresence(timestep)

motor_map = {}
motor_value = {}
current_joint_states_motor_value = {}
gripper_states = {}
gripper_motor = {}
gripper_motor_values = {}
connector_map = {}

planned_motor_values = {}
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    
    motor_map[name].setVelocity(2.0)
    motor_value[name] = 0
    current_joint_states_motor_value[name] = 0
for name in gripper_name:
    gripper_motor[name] = []
    gripper_motor_values[name] = []
    for motor_name in gripper_moter_map[name]:
        device = robot.getDevice(motor_name)
        device.setVelocity(1.0)
        gripper_motor[name].append(device)
        gripper_motor_values[name].append(0)


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
    # for c3_joint_name in c3_joint_names:
    #     motor_map["c3_" + c3_joint_name].setPosition(joint_1)
    #     joint_1_value = motor_map["c3_joint_1"].getPositionSensor()

    # motor_map["c3_joint_2"].setPosition(joint_2)
    # motor_map["c3_joint_3"].setPosition(joint_3)
    # motor_map["c3_joint_4"].setPosition(joint_4)
    # motor_map["c3_joint_5"].setPosition(joint_5)
    # motor_map["c3_joint_6"].setPosition(joint_6)
    
    for name in c3_joint_names:
        motor_map[name].setPosition(c3_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        c3_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    # joint_1_value = motor_map["c3_joint_1"].getPositionSensor()
    # joint_2_value = motor_map["c3_joint_2"].getPositionSensor()
    # joint_3_value = motor_map["c3_joint_3"].getPositionSensor()
    # joint_4_value = motor_map["c3_joint_4"].getPositionSensor()
    # joint_5_value = motor_map["c3_joint_5"].getPositionSensor()
    # joint_6_value = motor_map["c3_joint_6"].getPositionSensor()
    # joint_1_value.enable(timestep)
    # joint_2_value.enable(timestep)
    # joint_3_value.enable(timestep)
    # joint_4_value.enable(timestep)
    # joint_5_value.enable(timestep)
    # joint_6_value.enable(timestep)

    flag = 0
    for name in  c3_joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(c3_actual_joint[name])      
        if (c3_actual_joint[name] < c3_joints[name] + 0.02) and (c3_actual_joint[name] > c3_joints[name] - 0.02):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0



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
    # for c3_joint_name in c3_joint_names:
    #     motor_map["c3_" + c3_joint_name].setPosition(joint_1)
    #     joint_1_value = motor_map["c3_joint_1"].getPositionSensor()

    # motor_map["c3_joint_2"].setPosition(joint_2)
    # motor_map["c3_joint_3"].setPosition(joint_3)
    # motor_map["c3_joint_4"].setPosition(joint_4)
    # motor_map["c3_joint_5"].setPosition(joint_5)
    # motor_map["c3_joint_6"].setPosition(joint_6)
    
    for name in c3_joint_names:
        motor_map[name].setPosition(c3_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        c3_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    # joint_1_value = motor_map["c3_joint_1"].getPositionSensor()
    # joint_2_value = motor_map["c3_joint_2"].getPositionSensor()
    # joint_3_value = motor_map["c3_joint_3"].getPositionSensor()
    # joint_4_value = motor_map["c3_joint_4"].getPositionSensor()
    # joint_5_value = motor_map["c3_joint_5"].getPositionSensor()
    # joint_6_value = motor_map["c3_joint_6"].getPositionSensor()
    # joint_1_value.enable(timestep)
    # joint_2_value.enable(timestep)
    # joint_3_value.enable(timestep)
    # joint_4_value.enable(timestep)
    # joint_5_value.enable(timestep)
    # joint_6_value.enable(timestep)

    flag = 0
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
    # for c3_joint_name in c3_joint_names:
    #     motor_map["c3_" + c3_joint_name].setPosition(joint_1)
    #     joint_1_value = motor_map["c3_joint_1"].getPositionSensor()

    # motor_map["c3_joint_2"].setPosition(joint_2)
    # motor_map["c3_joint_3"].setPosition(joint_3)
    # motor_map["c3_joint_4"].setPosition(joint_4)
    # motor_map["c3_joint_5"].setPosition(joint_5)
    # motor_map["c3_joint_6"].setPosition(joint_6)
    
    for name in s5_joint_names:
        motor_map[name].setPosition(s5_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        s5_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    # joint_1_value = motor_map["c3_joint_1"].getPositionSensor()
    # joint_2_value = motor_map["c3_joint_2"].getPositionSensor()
    # joint_3_value = motor_map["c3_joint_3"].getPositionSensor()
    # joint_4_value = motor_map["c3_joint_4"].getPositionSensor()
    # joint_5_value = motor_map["c3_joint_5"].getPositionSensor()
    # joint_6_value = motor_map["c3_joint_6"].getPositionSensor()
    # joint_1_value.enable(timestep)
    # joint_2_value.enable(timestep)
    # joint_3_value.enable(timestep)
    # joint_4_value.enable(timestep)
    # joint_5_value.enable(timestep)
    # joint_6_value.enable(timestep)

    flag = 0
    for name in  s5_joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(s5_actual_joint[name])      
        if (s5_actual_joint[name] < s5_joints[name] + 0.01) and (s5_actual_joint[name] > s5_joints[name] - 0.01):
            flag += 1
    if (flag >= 6):
        break
    else:
        flag = 0
    
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
    # for c3_joint_name in c3_joint_names:
    #     motor_map["c3_" + c3_joint_name].setPosition(joint_1)
    #     joint_1_value = motor_map["c3_joint_1"].getPositionSensor()

    # motor_map["c3_joint_2"].setPosition(joint_2)
    # motor_map["c3_joint_3"].setPosition(joint_3)
    # motor_map["c3_joint_4"].setPosition(joint_4)
    # motor_map["c3_joint_5"].setPosition(joint_5)
    # motor_map["c3_joint_6"].setPosition(joint_6)
    
    for name in iiwa1_joint_names:
        motor_map[name].setPosition(iiwa1_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        iiwa1_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    # joint_1_value = motor_map["c3_joint_1"].getPositionSensor()
    # joint_2_value = motor_map["c3_joint_2"].getPositionSensor()
    # joint_3_value = motor_map["c3_joint_3"].getPositionSensor()
    # joint_4_value = motor_map["c3_joint_4"].getPositionSensor()
    # joint_5_value = motor_map["c3_joint_5"].getPositionSensor()
    # joint_6_value = motor_map["c3_joint_6"].getPositionSensor()
    # joint_1_value.enable(timestep)
    # joint_2_value.enable(timestep)
    # joint_3_value.enable(timestep)
    # joint_4_value.enable(timestep)
    # joint_5_value.enable(timestep)
    # joint_6_value.enable(timestep)

    flag = 0
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
    # for c3_joint_name in c3_joint_names:
    #     motor_map["c3_" + c3_joint_name].setPosition(joint_1)
    #     joint_1_value = motor_map["c3_joint_1"].getPositionSensor()

    # motor_map["c3_joint_2"].setPosition(joint_2)
    # motor_map["c3_joint_3"].setPosition(joint_3)
    # motor_map["c3_joint_4"].setPosition(joint_4)
    # motor_map["c3_joint_5"].setPosition(joint_5)
    # motor_map["c3_joint_6"].setPosition(joint_6)
    
    for name in iiwa1_joint_names:
        motor_map[name].setPosition(iiwa1_joints[name])
        motor_map[name].getPositionSensor().enable(timestep)
        iiwa1_actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        

    # joint_1_value = motor_map["c3_joint_1"].getPositionSensor()
    # joint_2_value = motor_map["c3_joint_2"].getPositionSensor()
    # joint_3_value = motor_map["c3_joint_3"].getPositionSensor()
    # joint_4_value = motor_map["c3_joint_4"].getPositionSensor()
    # joint_5_value = motor_map["c3_joint_5"].getPositionSensor()
    # joint_6_value = motor_map["c3_joint_6"].getPositionSensor()
    # joint_1_value.enable(timestep)
    # joint_2_value.enable(timestep)
    # joint_3_value.enable(timestep)
    # joint_4_value.enable(timestep)
    # joint_5_value.enable(timestep)
    # joint_6_value.enable(timestep)

    flag = 0
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
  
k = 1
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
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (iiwa2_actual_joint[name] < iiwa2_joints[name] + 0.02) and (iiwa2_actual_joint[name] > iiwa2_joints[name] - 0.02):
                flag += 1
        if (flag >= 7):
         break
        else:
            flag = 0

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
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (abb_actual_joint[name] < abb_joints[name] + 0.01) and (abb_actual_joint[name] > abb_joints[name] - 0.01):
                flag += 1
        if (flag >= 6):
         break
        else:
            flag = 0
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
            if (joints[name] != 0):
                motor_map[name].setPosition(joints[name])
                motor_map[name].getPositionSensor().enable(timestep)
                actual_joint[name] = motor_map[name].getPositionSensor().getValue()
        
        for name in  joint_names:  
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= 31):
         break
        else:
            flag = 0

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
        # print("The current flag is: ")
        # print(flag)
        # print(iiwa1_actual_joint[name])      
             if (actual_joint[name] < joints[name] + 0.01) and (actual_joint[name] > joints[name] - 0.01):
                flag += 1
        if (flag >= len(joint_names)-1):
         break
        else:
            flag = 0
while robot.step(timestep) != -1:
    #total assembly process (kill me now)
    if (k == 1):
     # # # iiwa_picking_up_frame_1_and_PCB + Battery_assembly+star_tracker_on_camera
     #0, 0, 0, 0, 0, 0,
        
        move_c3_s5_iiwa1(-1.543,-0.986,-0.168,0,-0.518,-0.062,-1.958,-0.854,0.850,0,-1.555,1.130,-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
        move_c3_s5_iiwa1(-1.543,-1.143,-0.168,0,-0.283,-0.062,-1.958,-0.967,0.713,0,-1.367,1.130,0.0779401883482933, 1.5567342042922974, 1.2206289768218994, -1.0333815813064575, -0.0469394288957119, 2.058905282974243, -0.428)
        if (connector_s5.getPresence() == 1 ):
              connector_s5.lock()
        if (connector_c3.getPresence() == 1 ):
             connector_c3.lock()
        move_c3_s5_iiwa1(-1.543,-0.986,-0.168,0,-0.518,-0.062,-1.958,-0.854,0.850,0,-1.555,1.130,0.0134669728577137, 1.5551997423171997, 1.2205066680908203, -1.2719889879226685, -0.06519236415624619, 1.914966344833374, -0.428)
        move_c3_s5_iiwa1(-2.196,-1.025,0.023,0,-0.565,0.504,-0.47, -0.28, 0.03, 0.06, -1.31, 2.13,4*degree_to_radian, 76*degree_to_radian,54*degree_to_radian,-89*degree_to_radian,9*degree_to_radian,97*degree_to_radian,-0.489)
        if (connector_iiwa1.getPresence() == 1):
             connector_iiwa1.lock()
        move_c3_s5_iiwa1(-2.196,-1.222,0.170,0,-0.565,0.669,-0.29017, -0.38765, -0.87555, 1.857, -1.461, -0.502,4*degree_to_radian, 37*degree_to_radian,54*degree_to_radian,-88*degree_to_radian,45*degree_to_radian,118*degree_to_radian,-0.489)
        connector_c3.unlock()
        move_c3_s5_iiwa1(-2.196,-1.025,0.023,0,-0.565,0.669,-0.27662542616551916, -1.1532876057695325, -0.4988641463914138, 1.8632736290932876, -1.5746517460884837, -0.8745676704037179,0.03332231566309929, 1.0522798299789429, 1.22066330909729, -1.2091633081436157, 0.48238345980644226, 2.058905282974243, -0.489)
        connector_s5.unlock()
        move_c3_s5_iiwa1(-1.365,-0.947,-0.360,0,-0.236,-0.245,-0.014562749123123124, -1.1417072481390163, -0.5569789975593534, 1.602887308549487, -1.5551027663103594, -0.9245321016675899,0.03331824392080307, 1.0522654056549072, 1.2205824851989746, -1.209204912185669, -0.8317239284515381, 2.058905282974243, -0.489)
        move_c3_s5_iiwa1(-1.365,-1.143,-0.216,0,-0.236,-0.245,-0.29017, -0.38765, -0.87555, 1.857, -1.461, -0.502,-0.953333854675293, 1.0523062944412231, 1.2205657958984375, -1.209246039390564, -0.8316945433616638, 2.058905282974243, -0.489)
        abb_screw_pickup_cycyles()
        move_c3_s5_iiwa1(
    # # # iiwa_picking_up_frame_1_and_PCB
    # if (k == 1):
    #     move_iiwa1(-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
    # if (k == 2):
    #     move_iiwa1(0.0779401883482933, 1.5567342042922974, 1.2206289768218994, -1.0333815813064575, -0.0469394288957119, 2.058905282974243, -0.428)
    # if (k == 3):
    #     move_iiwa1(0.0134669728577137, 1.5551997423171997, 1.2205066680908203, -1.2719889879226685, -0.06519236415624619, 1.914966344833374, -0.428)
    # if (k == 4):
    #     move_iiwa1(4*degree_to_radian, 76*degree_to_radian,54*degree_to_radian,-89*degree_to_radian,9*degree_to_radian,97*degree_to_radian,-0.489)
    # if (k == 5):
    #     if (connector_iiwa1.getPresence() == 1):
    #         connector_iiwa1.lock()
    # if (k == 6):
    #     move_iiwa1(4*degree_to_radian, 37*degree_to_radian,54*degree_to_radian,-88*degree_to_radian,45*degree_to_radian,118*degree_to_radian,-0.489)
    # if (k == 7):
    #     move_iiwa1(0.03332231566309929, 1.0522798299789429, 1.22066330909729, -1.2091633081436157, 0.48238345980644226, 2.058905282974243, -0.489)
    # if (k == 8):
    #     move_iiwa1(0.03331824392080307, 1.0522654056549072, 1.2205824851989746, -1.209204912185669, -0.8317239284515381, 2.058905282974243, -0.489)
    # if (k == 9):
    #     move_iiwa1(-0.953333854675293, 1.0523062944412231, 1.2205657958984375, -1.209246039390564, -0.8316945433616638, 2.058905282974243, -0.489)
    # if (k == 10):
    #     move_iiwa1(-0.953318178653717, 1.0522785186767578, 1.2205114364624023, -1.2093230485916138, -0.8316895961761475, 1.4909693002700806, -0.489) 
    # if (k == 11):
    #     move_iiwa1(-0.901426374912262, 1.2724210023880005, 1.2203257083892822, -0.5130434632301331, -1.1515997648239136, 1.6697752475738525, -0.428)   
    # if (k == 12):
    #     move_iiwa1(-0.9630538821220398, 1.405735731124878, 1.2203832864761353, -0.652347207069397, -1.2150418758392334, 1.4862936735153198, -0.428)
    
    # if (k == 14):
    #     connector_iiwa1.unlock()
    # if (k == 15):
    #     move_iiwa1(-0.901426374912262, 1.2724210023880005, 1.2203257083892822, -0.5130434632301331, -1.1515997648239136, 1.6697752475738525, -0.428) 
    # if (k == 16):
    #     move_iiwa1(0.07792849838733673, 1.556743860244751, 1.220532774925232, -1.0336353778839111, -0.04685851186513901, 2.058905282974243, -0.489)
    # if (k == 17):
    #     move_iiwa1(0.07003629207611084, 1.5945794582366943, 1.2205535173416138, -1.3431715965270996, -0.012548441998660564, 1.8020039796829224, -0.37667807936668396)
    # if (k == 18):
    #     #move_iiwa1(36*degree_to_radian,62*degree_to_radian,11*degree_to_radian,-85*degree_to_radian,-140*degree_to_radian,-111*degree_to_radian,-117*degree_to_radian)
    #     move_iiwa1(0.06335160743628049, 1.5920179144677278, 1.2207737274193737, -1.6015540995256925, -0.012107467932807583, 1.550722620047753, -0.37319941218693153)
    #     #iiwa1_cartesian_mode(0.07003629207611084, 1.5945794582366943, 1.2205535173416138, -1.3431715965270996, -0.012548441998660564, 1.8020039796829224, -0.37667807936668396,36*degree_to_radian,62*degree_to_radian,11*degree_to_radian,-85*degree_to_radian,-140*degree_to_radian,-111*degree_to_radian,-117*degree_to_radian,20)
    # if (k == 19):
    #     if (connector_iiwa1.getPresence() == 1):
    #          connector_iiwa1.lock()  
    
    # if (k == 20):
    #     move_iiwa1(-0.0012771035018149025, 1.2187325075442472, 1.600334331330132, -1.6020960150050583, 0.36704506268980785, 1.6098033912359533, -0.0062710723310366485)
    # if (k == 21):
    #    move_iiwa1(-1.5556524991989136, 0.8650952577590942, 1.220565915107727, -1.3611725568771362, 0.6817491054534912, 2.0456504821777344, -0.1672627478837967)
    # if (k == 22):
    #     move_iiwa1(-1.0599322319030762, 1.0551937818527222, 1.2205629348754883, -1.0886116027832031, -0.9644252061843872, 1.532305121421814, 1.222)
    # if (k == 23):
    #     move_iiwa1(-0.984226644039154, 1.2162747383117676, 1.2206645011901855, -0.5889201760292053, -1.095007061958313, 1.6766457557678223, 1.222)
    # if (k == 24):
    #     move_iiwa1(-0.9759791626718927, 1.3228973056629585, 1.1890477562289448, -0.6840075906115147, -1.1266585771001127, 1.5291298054303513, 1.222)

    # if (k == 27):
    #     connector_iiwa1.unlock()
    # if (k == 28):
    #     move_iiwa1(-0.9578371191702857, 1.1627607778496283, 1.2370601437274198, -1.312886149497805, -1.0901393446072707, 1.380543469952686, 1.8860359965337192)
    # #abb screwing
    # if (k == 29):
    #     move_iiwa1(-0.949170234213693, 1.2238746900389093, 1.191496545066366, -1.3315577668150742, -1.1367943978775406, 1.3106933553261801, 1.905129084121886)
    # # if (k == 30):
    # #     abb_screw_pickup_cycyles()
    # # #screw_pcb_and_frame_1_first_screws
    # # if (k == 31):
    # #     move_abb(1.0668907993892975, 0.14251364269781006, 0.35851581977866054, 0.0003060741815670643, 1.0708321717760356, 0.3704260962794732)
    # # if (k == 32):
    # #     move_abb(1.0668766765905406, 0.3979288960400913, 0.553888754139916, 0.0006004133288249669, 0.6202059784541681, 0.3703003377272314)
    # # if (k == 33):
    # #     move_abb(1.0668907993892975, 0.14251364269781006, 0.35851581977866054, 0.0003060741815670643, 1.0708321717760356, 0.3704260962794732)
    # #  #screw_pcb_and_frame_1_second_screws
    # # if (k == 34):
    # #     abb_screw_pickup_cycyles()
    # # if (k == 35):
    # #     move_abb(0.8755221459919103, -0.16694873765429044, 0.6647224015821317, 0.00012237554367559102, 1.0743179717016904, 0.1793078639271352)
    # # if (k == 36):
    # #     move_abb(0.8755221459387418, 0.1455876051653, 0.8877662125113176, 0.00020972585091440736, 0.5387378245011853, 0.17918613567053993)
    # # if (k == 37):
    # #     move_abb(0.8755221459919103, -0.16694873765429044, 0.6647224015821317, 0.00012237554367559102, 1.0743179717016904, 0.1793078639271352)
    # #  #screw_pcb_and_frame_1_third_screws
    # # if (k == 38):
    # #     abb_screw_pickup_cycyles()
    # # if (k == 39):
    # #     move_abb(0.8755221459919103, -0.16694873765429044, 0.6647224015821317, 0.00012237554367559102, 1.0743179717016904, 0.1793078639271352)
    # #     move_abb(0.6603987489246198, 0.7669328305349711, -0.5042831031188966, -0.00018598793019669203, 1.3092226851450948, -0.0357167205037262)
    # # if (k == 40):
    # #     move_abb(0.660398688449106, 0.9063891154129058, -0.27105204811328215, -0.00022252226704528363, 0.9365353527605713, -0.0356331556172594)
    # # if (k == 41):
    # #     move_abb(0.6603987489246198, 0.7669328305349711, -0.5042831031188966, -0.00018598793019669203, 1.3092226851450948, -0.0357167205037262)
    # #  #screw_pcb_and_frame_1_forth_screws
    # # if (k == 42):
    # #     abb_screw_pickup_cycyles()
    # # if (k == 43):
    # #     move_abb(0.8755221459919103, -0.16694873765429044, 0.6647224015821317, 0.00012237554367559102, 1.0743179717016904, 0.1793078639271352)
    # #     move_abb(0.5010451523801631, 0.5501135747138775, -0.1714779914558355, -0.00037788868593303915, 1.193293687678138, -0.1952063987917222)
    # # if (k == 44):
    # #     move_abb(0.5010451523939866, 0.7320800952683996, 0.02767326729345955, -0.0004840008244149905, 0.8121759423892914, -0.19501273422057505)
    # # if (k == 45):
    # #     move_abb(0.5010451523801631, 0.5501135747138775, -0.1714779914558355, -0.00037788868593303915, 1.193293687678138, -0.1952063987917222)
    
    # if (k == 46):
    #     abb_screw_pickup_cycyles()
    # #kuka_move_out_of_the_way
    # if (k == 47):
    #     move_iiwa1(-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
    # if (k == 48):
    #     move_iiwa1(1.775053858757019, 0.12268832325935364, 1.2970420122146606, -1.444926381111145, -0.08761254698038101, 1.6743909120559692, -1.6920497417449951)

    
    # if (k == 50):
    #     move_iiwa2(-1.4853272438049316, 0.18084903061389923, -0.01287818793207407, 0, 0.0031921921763569117, 1.6500217914581299, -1.477664589881897)
    # if (k == 51):
    #     move_iiwa2(0.31000828742980957, 0.4270554482936859, -0.012890651822090149, -1.811691164970398, -0.005011707078665495, 0.9043905735015869, 0.3213548958301544)
    # if (k == 52):
    #     move_iiwa2(0.3083815574645996, 0.5658107995986938, -0.01289214938879013, -1.8429688215255737, -0.003548768814653158, 0.7345393300056458, 0.32021793723106384)
    # if (k == 53):
    #     if (connector_iiwa2.getPresence() == 1):
    #          connector_iiwa2.lock() 
    # if (k == 54):
    #     move_iiwa2(0.31000828742980957, 0.4270554482936859, -0.012890651822090149, -1.811691164970398, -0.005011707078665495, 0.9043905735015869, 0.3213548958301544)
    # if (k == 55):
    #     move_iiwa2(-0.045563362538814545, 0.19725003838539124, 0.1365334540605545, -1.6749385595321655, 1.6509283781051636, 1.4810177087783813, 2.8315742015838623)
    # if (k == 56):
    #     move_iiwa2(-0.1497136836562787, 0.7913052677232785, 0.14596097009277714, -1.8955310555045923, 1.5970671267920138, 1.5328481938272047, 2.022205733521377)
    # if (k == 57):
    #     move_iiwa2(-0.1590682550815723, 0.9639333164520236, 0.1401645834100513, -1.8081418639291007, 1.4754754880064531, 1.5622987535452322, 2.0424494091483525)
    # if (k == 58):
    #     connector_iiwa2.unlock()
    # if (k == 59):
    #     move_iiwa2(-0.13769859095551962, 0.7804229747492768, 0.1556170959798383, -1.9263676331357262, 1.6136410884549677, 1.5198030263630193, 2.000409757773769)
    # if (k == 60):
    #     move_iiwa2(0,0,0,0,0,0,0)
    
    # #Blue_KuKa_Frame3_pickup
    # if (k == 61):
    #     move_iiwa1(-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
    # if (k == 62):
    #     move_iiwa1(0.11988958716392517, 1.0034582614898682, 1.5435606241226196, -1.2085002660751343, 0.6194223165512085, -1.3118194341659546, -1.405)
    # if (k == 63):
    #     move_iiwa1(0.08675584197044373, 1.8182295560836792, 1.5435400009155273, -1.4041098356246948, -0.2384127378463745, -1.4791195392608643, -1.405)
    # if (k == 64):
    #     move_iiwa1(0.15928059816360474, 1.8006699085235596, 1.5434728860855103, -1.020553708076477, -0.2755407392978668, -1.1818885803222656, -1.405)
       
    # if (k == 65):
    #     if (connector_iiwa1.getPresence() == 1):
    #          connector_iiwa1.lock() 
    # if (k == 66):
    #     print("I finished here")
    #     move_iiwa1(0.14437034017570516, 1.7446018440397066, 1.5780882403313903, -1.0632834344386122, -0.21980634537569954, -1.1937577610578405, -1.475795843454133)
    #     move_iiwa1(0.14166508550675747, 1.7527429099115355, 1.671030711052479, -1.0807701828897, -0.2398912534575898, -1.1899336851112388, -1.5612927138654458)
    #     move_iiwa1(0.1392490120895974, 1.7069999042999606, 1.700553931342581, -1.0949278966688107, -0.19334117102360665, -1.1963155262831386, -1.6153600310017135)

    #     move_iiwa1(0.13546901941299438, 1.6986061334609985, 1.5433796644210815, -1.0958319902420044, -0.16494713723659515, -1.2183781862258911, -1.405)
    # if (k == 67):
    #     print("I finished here 2")
    #     move_iiwa1(0.11988958716392517, 1.0034582614898682, 1.5435606241226196, -1.2085002660751343, 0.6194223165512085, -1.3118194341659546, -1.405)
    # if (k == 68):
    #     move_iiwa1(1.1971948146820068, 1.1385958194732666, 1.5434645414352417, 0.9375529885292053, 0.7061322927474976, -0.2914133369922638, -1.405)
    # if (k == 69):
    #     move_iiwa1(0.9636580944061279, 0.8521612286567688, 1.543437123298645, 1.3644012212753296, 2.208350419998169, -0.5461215376853943, -1.5)
    # if (k == 70):
    #     move_iiwa1(0.3909376859664917, 1.7951682806015015, 1.5772618055343628, 1.9289307594299316, 0.20569857954978943, 1.546726942062378, -1.405)
    # if (k == 71):
    #     move_iiwa1(-0.038004495203495026, 1.7947585582733154, 1.5773537158966064, 1.4883850812911987, 0.24525821208953857, 1.5698388814926147, -1.465)
    # if (k == 72):
    #     move_iiwa1(-0.2077738642692566, 1.8513702154159546, 1.5774123668670654, 1.189300775527954, 0.2973238229751587, 1.4454984664916992, -1.772)
    # if (k == 73):
    #     move_iiwa1(-0.23156093060970306, 1.8507188558578491, 1.5775201320648193, 1.1703054904937744, 0.2791872024536133, 1.4721970558166504, -1.772)
    # if (k == 74):
    #     move_iiwa1(-0.26183056831359863, 1.844139814376831, 1.577405333518982, 1.1343351602554321, 0.2703976631164551, 1.467003345489502, -1.772)
    # if (k == 75):
    #     move_iiwa1(-0.27867012918955325, 1.869463762395101, 1.4926423046434392, 1.163484885686218, 0.30906612420295, 1.440549525287419, -1.6822069700813742)
    # if (k == 76):
    #     sleep(2)
    #     connector_iiwa1.unlock()
    # if (k == 77):
    #     print("I can't reach here")
    #     move_iiwa1(-0.3270172788612569, 1.8345336197028896, 1.5616036299304952, 1.3279174762800676, 0.2622622731919648, 1.713958908763541, -1.7028783953529503)
    # if (k == 78):
    #     move_iiwa1(-0.3245303772769131, 1.496144219886644, 1.4732013219313154, 1.3612519357977535, -0.02213505723401971, 1.7429985564157882, -1.5354960917430176)
    # #back to zero position
    # if (k == 79):
    #     move_iiwa1(-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
    # if (k == 80):
    #     move_iiwa1(0.08337189242171261, 1.5965768696569689, 1.2202001359186108, -1.3518285179254808, -0.0054645215807255084, 1.7665325193305284, -1.896954561903709)
    # if (k == 81):
    #     move_iiwa1(0.08212826496581782, 1.5952797290193308, 1.2205227221585406, -1.604239575383511, -0.004583757569799801, 1.515739725462034, -1.8952680369345936)
    # if (k == 82):
    #     move_iiwa1(0.13992943915108558, 1.6151166961780175, 1.2229887242938402, -1.7942300575040062, -0.003634263452795058, 1.264686007511798, -1.894)
    # if (k == 83):
    #     if (connector_iiwa1.getPresence() == 1):
    #          connector_iiwa1.lock()  
    # if (k == 84):
    #     move_iiwa1(0.08143783360276742, 1.3504322319287951, 1.5452732091665013, -1.8543421169228227, 0.24380004070625072, 1.2829240450941743, -1.637617972682015)
    # if (k == 85):
    #     move_iiwa1(0.08193399012088776, 0.4928024709224701, 1.220682978630066, -1.6360968351364136, 0.3980826735496521, 1.5329352617263794, -1.637617972682015)
    # if (k == 86):
    #     move_iiwa1(-2.437218189239502, 0.4928644299507141, 1.2206048965454102, -1.63614022731781, 0.39814165234565735, 1.532779335975647, -2.138)
    # if (k == 87):
    #     move_iiwa1(-2.2249577045440674, 0.36847686767578125, 1.2207285165786743, -1.894509196281433, 1.4994399547576904, 0.9475332498550415, -2.138)
    # if (k == 88):
    #     move_iiwa1(-1.943572521964558, 1.2290598526628733, 0.8685270306084057, -1.970754660350053, 1.4363733775150436, 0.7600135943402724, -3.0194196701049805)
    # if (k == 89):
    #     move_iiwa1(-2.0133486424399414, 1.1565435802267725, 0.8664378086895086, -2.016557668989607, 1.3913629021681246, 0.7984682418540109, -2.95522804379348)
    # if (k == 90):
    #     move_iiwa1(-1.90645051090645, 1.190489041467632, 0.7881229084378092, -1.696813761677694, 1.2689239510988266, 0.9051985072269774, -2.716032162607879)
    # if (k == 91):
    #     sleep(2)
    #     connector_iiwa1.unlock()
    # if (k == 92):
    #     move_iiwa1(-2.0751925404915785, 1.117295614532358, 0.7125783506206788, -1.7392429660175124, 1.1186462532798642, 1.0687120444333271, -2.6139284185324203)
    # if (k == 93):
    #     move_iiwa1(-2.175429352743721, 0.7132396533949974, 0.8687322834606889, -1.7676837525001998, 1.2103287202939845, 1.1478391512027266, -2.27203253794364)
    # if (k == 94):
    #     move_iiwa1(-1.3533446788787842, 0.12267376482486725, 1.2969950437545776, -1.4448424577713013, -0.08766728639602661, 1.6745530366897583, -1.69206702709198)
    # if (k == 95):
    #     move_iiwa1(1.775053858757019, 0.12268832325935364, 1.2970420122146606, -1.444926381111145, -0.08761254698038101, 1.6743909120559692, -1.6920497417449951)
    







    # #s5 star tracker with kuka immobilization + abb screwing
    # if (k == 1):
    #     move_s5(-1.958,-0.854,0.850,0,-1.555,1.130)
    # if (k == 2):
    #     move_s5(-1.958,-0.967,0.713,0,-1.367,1.130)
    # if (k == 3):
    #     if (connector_s5.getPresence() == 1 ):
    #          connector_s5.lock()
    # if (k == 4):
    #     move_s5(-1.958,-0.854,0.850,0,-1.555,1.130)

    # if (k == 5):
    #     move_s5(-0.47, -0.28, 0.03, 0.06, -1.31, 2.13)
    
    # if (k == 6):
    #     move_s5(-0.29017, -0.38765, -0.87555, 1.857, -1.461, -0.502)
    # if (k == 7):
    #     move_s5(-0.27662542616551916, -1.1532876057695325, -0.4988641463914138, 1.8632736290932876, -1.5746517460884837, -0.8745676704037179)
        
    # if (k == 8):
    #     sleep(0.1)
    #     connector_s5.unlock()
    # if (k == 9):
    #     move_s5(-0.014562749123123124, -1.1417072481390163, -0.5569789975593534, 1.602887308549487, -1.5551027663103594, -0.9245321016675899)
    # if (k == 10):
    #     move_s5(-0.29017, -0.38765, -0.87555, 1.857, -1.461, -0.502)
    # if (k == 11):
    #     abb_screw_pickup_cycyles()
    #     move_abb(-2.6251, 0.4, 0.315381, 0.0005235, 1.07634, 0.513475)
    # if (k == 12):
    #     move_iiwa2(-1.4853272438049316, 0.18084903061389923, -0.01287818793207407, 0, 0.0031921921763569117, 1.6500217914581299, -1.477664589881897)
    #     move_iiwa2(0.31087970477936894, 0.2183416481702319, -0.013499697221277552, -1.5388926047246125, -0.006474041840806241, 1.3857208793602962, 0.31878328401154016)
    #     move_iiwa2(0.9981796333879075, 1.1246960206823462, -1.2427584266102831, -0.21734453233579346, -1.9991583633374643, -1.9332399251669994, -2.3608782174203604)
    #     move_iiwa2(1.083905955806051, 1.0326393090768806, -0.6104552698076945, -0.6862533891945515, -2.6379842195273397, -1.5166135465667407, -2.4455139704001088)
    #     move_iiwa2(1.0841859941898244, 1.1158538245950147, -0.5957684880405452, -0.6513522457586076, -2.622021966196267, -1.4648988712597737, -2.4214150063031004)
    #     move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    # if (k == 13):
    #     move_s5(-1.958,-0.854,0.850,0,-1.555,1.130)
    #     move_s5(-1.7374632522532578, -0.6343487432666972, -0.5790851504039649, -0.01962048988398411, -0.3866383530595364, 0.9716778496769897)
    #     move_s5(-1.7382306745482978, -0.827899836186708, -0.5491720868869517, -0.032780550453645786, -0.22282168246308506, 0.9862278258183743)
    # if (k == 14):
    #     if (connector_s5.getPresence() == 1 ):
    #          connector_s5.lock()
    # if (k == 15):
    #     print("help me i am stucked")
    #     move_s5(-1.7374632522532578, -0.6343487432666972, -0.5790851504039649, -0.01962048988398411, -0.3866383530595364, 0.9716778496769897)
    #     move_s5(-1.04077, 0.15693, 0.17875, -0.09419, -1.90616, 1.86977)
    # if (k == 16):
    #     move_s5(-0.5334672516388818, -0.6288815039430755, -0.19063342933795763, -0.11997541368505422, -0.7124808831427882, 1.3907856400262963)
    #     move_s5(-0.6436692429450254, -0.7343125908764423, -0.009328578156170426, -0.11547259201422637, -0.7970113443217335, 1.4911703741151983)
    # #abb_star_tracker_first_screw
    # if (k == 17):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     move_abb(-1.09404725732659, 0.145673282797211, 0.19065356401490324, 0.0004633386349007454, 1.2357637855268573, 0.5134952733325893)
    # if (k == 18):
    #     move_abb(-0.8425391012810101, 0.10808934264642314, 0.23240496742014513, 0.0007912082507884547, 1.2314470042356658, 0.7648921533108586)
    #     move_abb(-0.8425312111142141, 0.1186733015456657, 0.28587130524466225, 0.0007578038560231688, 1.1673176656330604, 0.7650197832980318)
    #     move_abb(-0.8425391012810101, 0.10808934264642314, 0.23240496742014513, 0.0007912082507884547, 1.2314470042356658, 0.7648921533108586)
    # if (k == 19):
    #     abb_screw_pickup_cycyles()
    # if (k == 20):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     move_abb(-0.7561524677966905, 0.0022889365038901155, 0.34333816293226105, 0.0009086900016939674, 1.2261859835640314, 0.8511738467984202)
    # if (k == 21):
    #     move_abb(-0.7561252752396168, -0.004620035196271232, 0.18869163755859106, 0.0007882031761204941, 1.3879440417392697, 0.8514744545592764)
    #     move_abb(-0.7560434310124123, 0.1086831778741202, 0.5689944558773228, 0.000976781986218332, 0.8940310405388412, 0.8509795256385336)
    #     move_abb(-0.7561252752396168, -0.004620035196271232, 0.18869163755859106, 0.0007882031761204941, 1.3879440417392697, 0.8514744545592764)

    # if(k == 22):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     abb_screw_pickup_cycyles()
    
    # #s5 leaving 
    # if (k == 23):
    #     connector_s5.unlock()
    #     sleep(0.1)
    #     move_s5(-0.6478485614286326, -0.5873866060753675, 0.10822397114366963, -0.09465571645666142, -1.0605150467801618, 1.460896616509276)
    # if (k == 24):
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 25):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     move_abb(-0.6309729147982844, 0.44684175256719794, -0.18702832314029516, 0.0009503680593441442, 1.312026195275389, 0.9764416151911173)
    #     move_abb(-0.6309729148075629, 0.4847123908778742, -0.011456844845323055, 0.0010316223243162195, 1.098584182508343, 0.9762155653522174)
    #     move_abb(-0.6309729147982844, 0.44684175256719794, -0.18702832314029516, 0.0009503680593441442, 1.312026195275389, 0.9764416151911173)
    # if (k == 26):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     abb_screw_pickup_cycyles()
    # if (k == 27):
    #     move_abb(-0.529679746948934, 0.33907106471409376, -0.043335527884459116, 0.0011487699727351502, 1.2758338950004997, 1.0776457301306737)
    #     move_abb(-0.5296797470085536, 0.38517179909628657, 0.12586365976685496, 0.001259612377664822, 1.0605341275364295, 1.07736447940269)
    #     move_abb(-0.529679746948934, 0.33907106471409376, -0.043335527884459116, 0.0011487699727351502, 1.2758338950004997, 1.0776457301306737)
    # if (k == 28):
    #     move_abb(-1.094, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     abb_screw_pickup_cycyles()
    # if (k == 29):
    #     move_s5(-0.014562749123123124, -1.1417072481390163, -0.5569789975593534, 1.602887308549487, -1.5551027663103594, -0.9245321016675899)
    #     move_s5(-0.27662542616551916, -1.1532876057695325, -0.4988641463914138, 1.8632736290932876, -1.5746517460884837, -0.8745676704037179)
    # if (k == 30):
    #     if (connector_s5.getPresence() == 1 ):
    #          connector_s5.lock()
    # if (k == 31):
    #     move_iiwa2(1.083905955806051, 1.0326393090768806, -0.6104552698076945, -0.6862533891945515, -2.6379842195273397, -1.5166135465667407, -2.4455139704001088)
    #     move_iiwa2(0.9981796333879075, 1.1246960206823462, -1.2427584266102831, -0.21734453233579346, -1.9991583633374643, -1.9332399251669994, -2.3608782174203604)
    #     move_iiwa2(0.31087970477936894, 0.2183416481702319, -0.013499697221277552, -1.5388926047246125, -0.006474041840806241, 1.3857208793602962, 0.31878328401154016)
    # if (k == 32):
    #     move_s5(-0.29017, -0.38765, -0.87555, 1.857, -1.461, -0.502)
        
    #     move_s5(-1.958,-0.854,0.850,0,-1.555,1.130)
    #     move_s5(-1.958,-0.967,0.713,0,-1.367,1.130)
    
    
    # # if (k == 33):
    # #     connector_s5.unlock()
        # move_s5(-1.958,-0.967,0.713,0,-1.367,1.130)
    # if (k == 26):

    # #s5 antenna subassembly
    # if (k == 1):
    #     move_s5(1.958 , -0.066, -0.382, 0.066, -1.037, 0.377)
    # if (k == 2):
    #     move_s5(1.958 , -0.291, -0.519, 0.066, -0.801, 0.377)
    # #print(connector_s5.getPresence())
    # if (k == 3):
    #     if (connector_s5.getPresence() == 1 ):
    #         connector_s5.lock()
    # if (k == 5):    
    #     move_s5(1.958 , -0.066, -0.382, 0.066, -1.037, 0.377)
    
    
    
    # #c3 battery subassembly
    # if (k == 1):
    #     move_c3(-1.543,-0.986,-0.168,0,-0.518,-0.062)
        
        
    # if (k == 10):
        
    #     move_c3(-1.543,-1.143,-0.168,0,-0.283,-0.062)
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 11):
    #     move_c3(-1.543,-0.986,-0.168,0,-0.518,-0.062)
    # if (k == 12):
    #     move_c3(-2.196,-1.025,0.023,0,-0.565,0.504)
    # if (k == 13):
    #     move_c3(-2.196,-1.222,0.170,0,-0.565,0.669)
    #     print("13")
    # if (k == 14):
    #     connector_c3.unlock()
    # if (k == 15):
    #     move_c3(-2.196,-1.025,0.023,0,-0.565,0.669)
    # if (k == 16):
    #     move_c3(-1.365,-0.947,-0.360,0,-0.236,-0.245)
    # if (k == 17):
    #     move_c3(-1.365,-1.143,-0.216,0,-0.236,-0.245)
    # if (k == 18):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 19):
    #     move_c3(-1.365,-0.947,-0.360,0,-0.236,-0.245)
    # if (k == 20):
    #     move_c3(-2.097,-0.986,-0.168,0,-0.377,0.473)
    # if (k == 21):
    #     move_c3(-2.097,-1.180,-0.071,0,-0.336,0.532)
    # if (k == 22):
    #     connector_c3.unlock()
    # if (k == 23):
    #     move_c3(-2.077,-0.986,-0.168,0,-0.377,0.473)
    # if (k == 24):
    #     move_c3(-1.246,-0.986,-0.119,0,-0.518,-0.29)
    # if (k == 25):
    #     move_c3(-1.2,-1.143,-0.119,0,-0.33,-0.29)
    # if (k == 26):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 27):
    #     move_c3(-1.246,-0.986,-0.119,0,-0.518,-0.29)
    # if (k == 28):
    #     move_c3(-1.958,-0.75,-0.392,-0.14,-0.377,0.3956)
    # if (k == 29):
    #     # move_c3(-2.02,-1.065,-0.216,0.03,-0.290,0.349)
    #     move_c3(-2.019948424431183, -1.0257850211869033, -0.23459006435987642, 0.02808115533261577, -0.31055939859055803, 0.35094440548568895)
    # if (k == 30):
    #     connector_c3.unlock()
    # if (k == 31):
    #     move_c3(-1.958,-0.75,-0.392,-0.14,-0.377,0.3956)
    # if (k == 32):
    #     move_c3(-1.009,-0.619,-0.327,0,-0.565,-0.656)
    # if (k == 33):
    #     move_c3(-1.068,-1.156,-0.010,0.14,-0.424,-0.656)
    # if (k == 34):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 35):
    #     move_c3(-1.009,-0.619,-0.327,0,-0.565,-0.656)
    # if (k == 36):
    #     move_c3(-1.78,-0.75,-0.294,0,-0.518,0.212)
    # if (k == 37):
    #     move_c3(-1.78,-1.045,-0.360,0,-0.153,0.212)
    # if (k == 38):
    #     connector_c3.unlock()
    # if (k == 39):
    #     move_c3(-1.78,-0.75,-0.294,0,-0.518,0.212)
    # if (k == 40):
    #     move_c3(-0.831,-1.222,0.555,0,-0.848,1.343)
    # if (k == 41):
    #     move_c3(-0.771,-1.381,0.507,0.03,-0.656,2.314) 
    # if (k == 42):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 43):
    #     move_c3(-0.831,-1.222,0.555,0,-0.848,2.314)
    # if (k == 44):
    #     print("here")
    #     move_c3(-2.018,-0.829,-0.216,0,-0.565,0.395)
    # if (k == 45):
    #     print("here2")
    #     move_c3(-2.018,-0.911,-0.216,0,-0.473,0.395)
    # if (k == 46):
    #     print("Herer")
    #     connector_c3.unlock()
    # if (k == 47):
    #     print("Herer")
    #     move_c3(-2.018,-0.829,-0.216,0,-0.565,0.395)
    # if (k == 48):
    #     move_c3(-0.059,-0.79,-0.408,0,-0.424,1.572)
    # if (k == 49):
    #     move_c3(-0.059,-1.182,-0.216,0,-0.141,1.572)
    # if (k == 50):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock()
    # if (k == 51):
    #     move_c3(-0.059,-0.79,-0.408,0,-0.424,1.572) 
    # if (k == 52):
    #     move_c3(-2.018,-0.868,-0.216,0,-0.471,0.395)
    # if (k == 54):
    #     move_c3(-2.009,-1.030,-0.1618,0.0205,-0.4173,0.4071)
    # if (k == 55):

    #     connector_c3.unlock()
    # if (k == 56):
    #     move_c3(-2.018,-0.868,-0.216,0,-0.471,0.395)
    # if (k == 57):
    #     move_c3(0,0,0,0,0,0)
    
    # #c3 wifi assembly
    # if (k == 1):
    #     move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     move_c3(2.2653607233125,  -0.13453550799878,  -0.22716406236104,  -0.050635476592814,  -1.1806466892077,  -0.656)
    #     move_c3(2.059219467614236, -0.16875934452152247, -0.18761564353460694, -0.0555626330436642, -1.196259794932288, -0.44866388948715913)
    #     move_c3(1.884108226411429, -0.5526744119143677, 0.33680898611158877, -0.055642721009926135, -1.3458905553822367, -0.28139062784362573)
    #     move_c3(1.8841082264067082, -1.2892248683084033, 0.29021671876322774, -0.10147028165429715, -0.5647397461567932, -0.20801292861161153)
    # if(k == 2):
    #      if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock() #camera_board_pick_up
    # if (k == 3):
    #     move_c3(1.884108226411429, -0.5526744119143677, 0.33680898611158877, -0.055642721009926135, -1.3458905553822367, -0.28139062784362573)
    #     move_c3(1.295464911243009, -0.6176871014308714, 0.43829562129425503, -0.05090452047673471, -1.4139436420208102, 0.30368957187717605)
    #     move_c3(1.295464911237066, -0.8546371766313355, 0.22589985023311226, -0.061163890217894624, -0.9652738980477225, 0.3305744117377703)
    # if (k == 4):
    #     connector_c3.unlock()
    # if (k == 5):
    #     move_c3(1.2505384582909158, -0.6530120871650208, -0.15941477339193394, -0.06974643379789901, -0.7841817398859605, 0.39014436415903553)
    #     move_c3(1.2504635895262266, -0.7449923501341165, -0.16039645844776498, -0.07717822233610679, -0.6914571092559805, 0.40016359013288555)

    # if (k == 6):
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    #     move_abb(-1.9869048997641179, 0.12942434805388517, 0.5458799319247134, 0.001217304484346937, 0.8961158361385471, 1.1513109850693408)
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    # if (k == 7):
    #     abb_screw_pickup_cycyles()
    # if (k == 8):
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    #     move_abb(-1.6438568387897876, -0.03500081240822435, 0.5447052791310724, 0.0014536158609799543, 1.0612887420190082, 1.4943598323987177)
    #     move_abb(-1.6438568387767902, 0.036494080651874136, 0.6462680115703299, 0.001635379148619234, 0.8882313211897204, 1.4940372565500013)
    #     move_abb(-1.6438568387897876, -0.03500081240822435, 0.5447052791310724, 0.0014536158609799543, 1.0612887420190082, 1.4943598323987177)
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    # if (k == 9):
    #     abb_screw_pickup_cycyles()
    # if (k == 10):
    #     #switch to pickup wifi card
    #     move_c3(1.2505384582909158, -0.6530120871650208, -0.15941477339193394, -0.06974643379789901, -0.7841817398859605, 0.39014436415903553)
    #     move_c3(2.059219467614236, -0.16875934452152247, -0.18761564353460694, -0.0555626330436642, -1.196259794932288, -0.44866388948715913)
    #     move_c3(2.059219466845119, -1.3017633196567302, -0.12746187566417105, -0.39951569025014977, -0.13326858382132958, -0.07267500779692976)
    # if (k == 11):
    #     if (connector_c3.getPresence() == 1 ):
    #         connector_c3.lock() #wifi_card_pick_up
    # if (k == 12):
    #     move_c3(2.059219467614236, -0.16875934452152247, -0.18761564353460694, -0.0555626330436642, -1.196259794932288, -0.44866388948715913)
    # if (k == 13):
    #     #abb_finished the #3 srews
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    #     move_abb(-1.8876040982718971, 0.39845433907766176, 0.04140629955702223, 0.0010295270429717892, 1.1314629032988717, 1.2509489677020742)
    #     move_abb(-1.8876040983490214, 0.45127187126420754, 0.13709415343381287, 0.0011197125273937618, 0.9829576025179247, 1.2507659104843996)
    #     move_abb(-1.8876040982718971, 0.39845433907766176, 0.04140629955702223, 0.0010295270429717892, 1.1314629032988717, 1.2509489677020742)
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    # if (k == 14):
    #     abb_screw_pickup_cycyles()
    # if (k == 15):
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    #     move_abb(-1.6301459652197823, 0.3269340158442247, 0.13554677922902172, 0.0011655192673570762, 1.1085139169733582, 1.5082406179434666)
    #     move_abb(-1.6301459652083592, 0.3835737791535649, 0.2304196902353583, 0.001276114341995592, 0.9570013549018342, 1.5080254217089555)
    #     move_abb(-1.6301459652197823, 0.3269340158442247, 0.13554677922902172, 0.0011655192673570762, 1.1085139169733582, 1.5082406179434666)
    #     move_abb(-1.9869048999895298, 0.06099311082103416, 0.44757372468306134, 0.0010879606156530114, 1.0628531707136137, 1.1515422076311579)
    # if (k == 16):
    #     abb_screw_pickup_cycyles()
    # if (k == 17):
    #     move_c3(1.3502979852871029, -0.18873619472305436, -0.16393067983496798, -0.05436331651678263, -1.2381045559600254, 1.857)
    #     move_c3(1.3502979852871029, -0.18873619472305436, -0.16393067983496798, -0.05436331651678263, -1.2381045559600254, 1.857)
    #     move_c3(1.350297985171678, -0.5411808717293126, -0.37238823104491287, -0.08193071912496522, -0.6783868177004514, 1.857)
    #     move_c3(1.144844567197346, -0.6440266648796535, -0.18075297423116543, -0.0660270576567625, -0.7768645346357111, 2.086)

    # if (k == 18):
    #     connector_c3.unlock()
    # if (k == 19):
    #     move_c3(1.144844567197346, -0.6440266648796535, -0.18075297423116543, -0.0660270576567625, -0.7768645346357111, -1.15191731)
    #     move_c3(1.0225514897014678, -0.7238451500128076, -0.029559021990588505, -0.05607839908724796, -0.8535011342742949, -1.0397095075734422)
    #     move_c3(1.0225514896971946, -0.83986772687066, -0.027180392016176797, -0.06267773365048956, -0.7400564228462388, -1.0302838028789956)
    # if (k == 20):
       
    #     move_abb(-2.6054983265762015, 0.18034512881983875, 0.3154223838319276, 0.0005181582655728796, 1.0763445736026553, -1.3102595442487581)
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
    #     move_abb(-2.124985615852379, 0.3208099616506842, 0.3137933760911099, 0.0010856440649690937, 0.9371260653639908, -0.8302917210059807)
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
    #     move_abb(-2.6054983265762015, 0.18034512881983875, 0.3154223838319276, 0.0005181582655728796, 1.0763445736026553, -1.3102595442487581)
    # if (k == 21):
    #     abb_screw_pickup_cycyles()
    # if (k == 22):
    #     move_c3(1.0225514897014678, -0.7238451500128076, -0.029559021990588505, -0.05607839908724796, -0.8535011342742949, -1.0397095075734422)
    #     move_c3(1.144844567197346, -0.6440266648796535, -0.18075297423116543, -0.0660270576567625, -0.7768645346357111, -1.15191731)
    #     move_c3(0,0,0,0,0,0)
    # if (k == 23):
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
    #     move_abb(-2.075317677878935, 0.37943829725983175, 0.06682280210042804, 0.0010047701370130178, 1.1253691388046512, -0.7803612734183589)
    #     move_abb(-2.075317677888415, 0.43329692536300535, 0.1622101124959828, 0.00109464783347942, 0.9761232820769392, -0.7805416382244548)
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)

    # if (k == 24):
    #     abb_screw_pickup_cycyles()
    # if (k == 25):
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
    #     move_abb(-1.8645502853288132, 0.255152782865962, 0.22597437215082783, 0.001191727839920763, 1.090453551002733, -0.5697037999835963)
    #     move_abb(-1.8645502853848008, 0.3153908822061336, 0.32082163093265237, 0.0013131770990751866, 0.9353683137449873, -0.5699325224193015)
    #     move_abb(-1.8645502853288132, 0.255152782865962, 0.22597437215082783, 0.001191727839920763, 1.090453551002733, -0.5697037999835963)
    #     move_abb(-2.1249856158286238, 0.2608454762777418, 0.21897396134374197, 0.0009857670011330522, 1.0919098829906164, -0.8301031367245145)
    # if(k == 26):
    #     move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)



    #s5 antenna assembly
    # if (k == 1):
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75) #zero position
    # #pick up mezanine card
    # if (k == 2):
    #     move_s5(1.9252106850842343, -0.02411744452605953, -0.30441975443850977, -0.019756314392676947, -1.2323293613650184, 0.44728848334163307)
    # if (k == 3):
    #     move_s5(1.92175246029508, -0.23598323923824957, -0.5676191840598876, -0.02717466024136485, -0.7574518613811542, 0.4639335710915744)
    # if (k == 4):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock()
    # if (k == 5):
    #     move_s5(1.9252106850842343, -0.02411744452605953, -0.30441975443850977, -0.019756314392676947, -1.2323293613650184, 0.44728848334163307)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 6):
    #     move_s5(0.35, -0.68, -0.56, 1.26, -1.71, -0.502)
    #     move_s5(-0.36, -0.69, -0.55, 1.94, -1.414, -0.502)
    #     move_s5(-0.41005078676120693, -0.6545359509130284, -0.6399895391049999, 1.9964724818743826, -1.4213266552862938, -0.558752974241143)
    #     move_s5(-0.4008664683977777, -1.1375490160108526, -0.4739238199176526, 1.965, -1.556226100652115, -0.846754894125419)
    # if (k == 7):
    #     connector_s5.unlock()
    # if (k == 8):
    #     #linear path 
    #     move_s5(-0.14878587664735873, -1.1226806839439292, -0.5750457011481085, 1.714924570494449, -1.558631527851103, -0.9347395411650071)
    #     move_s5(-0.15642696295835193, -0.7527345927183351, -0.7226232953454355, 1.7163750484540004, -1.5265228837269318, -0.7146055338849259)
    # if (k == 9):
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 10):
    #     move_s5(2.172839632008246, -0.3256232720619874, 0.2942274723930391, -0.018413006900047515, -1.5027376509582377, 0.20916057305745273)
    #     move_s5(2.1689312471003834, -0.3710622432545478, -0.037797045346621386, -0.020515086249184203, -1.1254165196035808, 0.22065358417529557)
    # if(k == 11):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock()
    # if (k == 12):
    #     move_s5(2.172839632008246, -0.3256232720619874, 0.2942274723930391, -0.018413006900047515, -1.5027376509582377, 0.20916057305745273)
    # if (k == 13):
    #     move_s5(0.0, 0.28, -0.17, 0.0, -1.26, 0.75)
    #     move_s5(-0.28924033972456664, -0.6347157500514058, -0.6792309823974232, 1.8807777118507059, -1.4579758835862633, -0.5654874723152347)
    #     move_s5(-0.2804463724584672, -1.0779991020156452, -0.5387220983027776, 1.8908374718176846, -1.5533282422401744, -0.8535795999259553)
    # if (k == 14):
    #     connector_s5.unlock()
    # if(k == 15):
    #     move_s5(-0.01898469709847403, -1.0641118430850058, -0.5987957281865682, 1.6308298273517823, -1.5443746268052698, -0.9018693138183381)
    #     move_s5(-0.026662581749596573, -0.6930911990989901, -0.7290064206121393, 1.6304023670122283, -1.529689880222173, -0.6613693030351984)
    # if(k == 16):
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75) #zero position
    # if (k == 17):
    #     move_s5(1.3294360090468227, 0.031844860931506, -0.36090946417331365, -0.022161006388537605, -1.2440338458295221, 1.0437525068869857)
    #     move_s5(1.3341851630204444, -0.5827288947866237, -0.6859128745389713, -0.06998306344369777, -0.30498351297591897, 1.0986514231304016)
    # if (k == 18):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock()
    # if (k == 19):
    #     move_s5(1.3333530592341516, -0.42032571703131316, -0.6890359601901915, -0.046929077732611915, -0.46402059153143976, 1.0746933514389942)
    #     move_s5(1.3294360090468227, 0.031844860931506, -0.36090946417331365, -0.022161006388537605, -1.2440338458295221, 1.0437525068869857)
    # if (k == 20):
    #     move_s5(-0.47, -0.28, 0.03, 0.06, -1.31, 2.13)
    #     move_s5(-0.91403, -0.69263, 0.41277, -0.06609, -1.29212, 1.005)
    #     move_s5(-0.936962143031431, -0.7648786671497702, 0.5315758614528487, -0.06532831817324604, -1.3400283456539335, 0.17)
    #     move_s5(-0.9443584323868043, -0.7987994441852174, 0.35763653980882054, -0.07020377201788061, -1.1331107953020414, 0.17)
    # if(k == 21):
    #     connector_s5.unlock()
    # if (k == 22):
    #     move_s5(-0.936962143031431, -0.7648786671497702, 0.5315758614528487, -0.06532831817324604, -1.3400283456539335, 0.17)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
       
    # if (k == 23):

    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.6413067380705493, -1.413318314675726, -0.5758789619708733, 0.9944898736272417, 1.8112460630668317, -0.4020011387065372)
    #     move_s5(0.8572702897978474, -1.3646756595784062, -0.3928879536638339, 0.7447264180433938, 1.7112960585382333, -0.6236182510833207)
    # if (k == 24):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock() #trape door #1 
    # if (k == 25):
    #     move_s5(0.6413067380705493, -1.413318314675726, -0.5758789619708733, 0.9944898736272417, 1.8112460630668317, -0.4020011387065372)
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 26):
    #     move_s5(-1.22, -0.1, -0.56, -1.26, -0.45, 2.4)
    #     move_s5(-0.9788944289443595, -0.18546432540313076, -0.695332072417504, -1.801292685366173, -0.13305901660844033, 1.147727764484247)
    # if (k == 27):
    #     connector_s5.unlock()
    # if (k == 28):
    #     move_s5(-1.0140431812291189, -0.019168918410652967, -0.8802392446287555, -1.857500348096273, -0.1590711536527649, 1.2321670869777128)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if(k == 29):
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(0.8450951745807799, -1.5038237551146476, -0.7168636251728241, 0.8627181760946049, 2.032727899533095, -0.27214099691602384)
    #     move_s5(1.0509815498079509, -1.3876651259222437, -0.5354041327537781, 0.5707282915983751, 1.873324055765527, -0.5622026192171916)
    # if (k == 30):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock() #trape door #2 
    # if (k == 31):
    #     move_s5(0.8450951745807799, -1.5038237551146476, -0.7168636251728241, 0.8627181760946049, 2.032727899533095, -0.27214099691602384)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 32):
    #     move_s5(-0.5866366968905607, -0.7729792735971659, 0.6207573967269319, 0.5559302922437777, -1.5543821179939008, 0.5826399099928261)
    #     move_s5(-0.5831490260646013, -0.8217854848749521, 0.4090487281983526, 0.5730304654155244, -1.3357023515695194, 0.4395839445146388)

    # if (k == 33):
    #     connector_s5.unlock()
    # if (k == 34):
    #     move_s5(-0.4942953996207684, -0.8704836645992249, 0.4996213817925879, 0.5483642059012466, -1.4148717875934471, 0.3913542634267625)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 35):
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(1.1954819771575875, -1.5364927939404993, -0.7512649087801693, 0.5090963237978449, 2.2239793499925686, -0.42380595000416194)
    #     move_s5(1.2834178793164466, -1.4190736045613066, -0.6170776942716283, 0.3438284580469112, 2.014969234590358, -0.5976979432746875)
    # if (k == 36):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock() #trape door #3 
    # if (k == 37):
    #     move_s5(1.1954819771575875, -1.5364927939404993, -0.7512649087801693, 0.5090963237978449, 2.2239793499925686, -0.42380595000416194)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 38):
    #     move_s5(-0.93676, -0.96899, 1.04592, -0.00906, -1.696, 2.512)
    #     move_s5(-0.9020469862117904, -0.9845011436895288, 1.075946416325128, -0.010761924781414203, -1.7100324337373585, 2.47710649053323)
    #     move_s5(-0.9001398736896784, -0.9353153477869279, 0.7572081195607324, -0.010740728212544941, -1.440640104596813, 2.4778876795382723)
    # if (k == 39):
    #     connector_s5.unlock()
    # if (k == 40):
    #     move_s5(-0.9001398736896784, -0.9353153477869279, 0.7572081195607324, -0.010740728212544941, -1.178, 2.4778876795382723)
    #     move_s5(-0.9020469862117904, -0.9845011436895288, 1.075946416325128, -0.010761924781414203, -1.7100324337373585, 2.47710649053323)
    #     move_s5(-0.93676, -0.96899, 1.04592, -0.00906, -1.696, 2.512)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75) 
    # if (k == 41):
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(1.1954819771575875, -1.5364927939404993, -0.7512649087801693, 0.5090963237978449, 2.2239793499925686, -0.42380595000416194)
    #     move_s5(1.5669865812119483, -1.488063177351483, -0.7107452809498523, 0.03182571763487291, 2.201060669915619, -0.730967442767599)
    #     move_s5(1.5675659146872485, -1.4367449265517027, -0.6482150854863076, 0.028900833185129777, 2.0872739954468034, -0.7354520460127794)
    # if (k == 42):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock() #trape door #4
    # if (k == 43):
    #     move_s5(1.5669865812119483, -1.488063177351483, -0.7107452809498523, 0.03182571763487291, 2.201060669915619, -0.730967442767599)
    #     move_s5(0.8331431852123611, -0.8450491898647704, -1.1148402571829505, 0.7992644664369459, 1.8518624061006408, -0.47433088133791174)
    #     move_s5(0.6185800176739229, -0.1314586319233018, -0.9735948959756123, 1.0247413830932868, 1.3179369566788022, -1.1437719589939253)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 44):
    #     move_s5(-1.0215611158451294, -0.8309904443240202, 0.792182246034641, -0.13059580401945087, -1.5874935837757962, -2.108144637125975)
    #     move_s5(-1.019884068915975, -0.8305249764899946, 0.5637131285533591, -0.1336104140943005, -1.3612424044960452, -2.07968708747807)
    # if (k == 45):
    #     connector_s5.unlock()
    # if (k == 46):
    #     move_s5(-1.019884068915975, -0.8305249764899946, 0.5637131285533591,  0.133, -1.3612424044960452, -2.07968708747807)
    #     move_s5(-1.0215611158451294, -0.8309904443240202, 0.792182246034641, -0.13059580401945087, -1.5874935837757962, -2.108144637125975)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 47):
    #     move_s5(1.45, -0.69, -0.46, -0.1, -0.47, -0.56)
    #     move_s5(1.7621123673268286, -0.5175512352185114, -0.9056530095583291, -0.31343785866725443, -0.18740806715837566, -0.6532551449936834)
    #     move_s5(1.7616851146843089, -0.5879347134136692, -0.8954839556128692, -0.45358580531246395, -0.13144611155819705, -0.5109387143391654)
    # if (k == 48):
    #     if (connector_s5.getPresence() == 1):
    #         connector_s5.lock() #top cap
    # if (k == 49):
    #     move_s5(1.7621123673268286, -0.5175512352185114, -0.9056530095583291, -0.31343785866725443, -0.18740806715837566, -0.6532551449936834)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    #     move_s5(-1.00146, -0.99944, 0.90548, -0.04827, -1.51809, -0.6)
    #     move_s5(-0.9274278350134159, -0.792686326698126, 0.5418119989660105, -0.05221945102875378, -1.3578766419166177, -0.6654007682574737)
    #     move_s5(-0.9254718373830728, -0.8456019789382624, 0.40168870220045233, -0.055634681389070045, -1.1650177373286006, -0.6564221930990074)
    # if (k == 50):
    #     connector_s5.unlock()
    # if (k == 51):
    #     move_s5(-0.9274278350134159, -0.792686326698126, 0.5418119989660105, -0.05221945102875378, -1.3578766419166177, -0.6654007682574737)
    #     move_s5(0.0, 0.31, -0.54, 0.0, -0.91, 0.75)
    # if (k == 52):
    #     #first screw
    #     move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.5398876282193393, -0.14044425467392588, 0.0459794987939382, -0.02370256124287502, 1.70058053829685, -0.04382695397959718)
    #     move_abb(-0.5399036950512087, -0.13544478064221926, 0.6985723841851069, -0.02730699442691207, 1.0432485231595223, -0.026882402559685728)
    #     move_abb(-0.5398876282193393, -0.14044425467392588, 0.0459794987939382, -0.02370256124287502, 1.70058053829685, -0.04382695397959718)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 53):
    #     abb_screw_pickup_cycyles()
    # if (k == 54):
    #     #second screws
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.17263625625515594, 0.023648042167017307, -0.10945148946775907, -0.009299101952616986, 1.698125003444409, 0.32526688540925236)
    #     move_abb(-0.18893779270086275, -0.02890232540529255, 0.27066666089889396, -0.010101512490093688, 1.3704168760917868, 0.31217017026434907)
    #     move_abb(-0.17263625625515594, 0.023648042167017307, -0.10945148946775907, -0.009299101952616986, 1.698125003444409, 0.32526688540925236)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 55):
    #     abb_screw_pickup_cycyles()
    # if (k == 56):
    #     #third screws
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.3428995829514646, 0.4120397238077253, -0.5753662162537994, -0.016452427553756505, 1.7732514708779736, 0.15298335619994485)
    #     move_abb(-0.3428995829523859, 0.31971545528709433, -0.121820520355974, -0.01632154151876618, 1.4120774940312009, 0.15887171173823594)
    #     move_abb(-0.3428995829514646, 0.4120397238077253, -0.5753662162537994, -0.016452427553756505, 1.7732514708779736, 0.15298335619994485)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 57):
    #     abb_screw_pickup_cycyles()
    # if (k == 58):
    #     #forth screws
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.610050934353922, 0.25843845680142463, -0.3745625715137532, -0.026260157240759672, 1.7203668320097636, -0.11459175795832709)
    #     move_abb(-0.6100509343567877, 0.18947857541880553, 0.03704580603893819, -0.02645805620116717, 1.3778350494696212, -0.10560286245649904)
    #     move_abb(-0.610050934353922, 0.25843845680142463, -0.3745625715137532, -0.026260157240759672, 1.7203668320097636, -0.11459175795832709)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)

    # if (k == 59):
    #     abb_screw_pickup_cycyles()
    # if (k == 60):
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.5249123390569875, 0.17552691442041912, -0.2750567829351552, -0.023319261297431358, 1.7061345031776929, -0.02890174115640969)
    #     move_abb(-0.5249123390539815, 0.12770317118719218, -0.05445758625047, -0.023122150393079716, 1.5334053958912817, -0.024890308001811197)
    #     move_abb(-0.5249123390569875, 0.17552691442041912, -0.2750567829351552, -0.023319261297431358, 1.7061345031776929, -0.02890174115640969)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 61):
    #     abb_screw_pickup_cycyles()
    # if (k == 62):
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.2954407349556111, 0.04948383891117021, -0.13609013596230293, -0.014355764789265181, 1.697397762145704, 0.20171135120658498)
    #     move_abb(-0.2954407349582928, 0.005645292045513998, 0.07470707082731484, -0.014252459949896127, 1.5304561029922166, 0.20409891381384462)
    #     move_abb(-0.2954407349556111, 0.04948383891117021, -0.13609013596230293, -0.014355764789265181, 1.697397762145704, 0.20171135120658498)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 63):
    #     abb_screw_pickup_cycyles()
    # if (k == 64):
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.3734130223393051, 0.2712198298017933, -0.3904592702306068, -0.017634220664045394, 1.7286263084696405, 0.12307814497074991)
    #     move_abb(-0.37341302232011325, 0.21739169934931274, -0.15787609227009686, -0.017418820610049283, 1.549898568202225, 0.12621411924464632)
    #     move_abb(-0.3734359325674495, 0.27131995450567076, -0.3903936390340022, -0.01754201860364104, 1.7286965538746315, 0.12309471596716112)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 65):
    #     abb_screw_pickup_cycyles()
    # if (k == 66):
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    #     move_abb(-0.5662867754719921, -0.17184080033520674, 0.07279224772685114, -0.024750736607854415, 1.704435533731802, -0.07027152902437248)
    #     move_abb(-0.5662867754842166, -0.21727489520547305, 0.27685870515960787, -0.024537639276258467, 1.5458511377296165, -0.06636087993849332)
    #     move_abb(-0.5662867754719921, -0.17184080033520674, 0.07279224772685114, -0.024750736607854415, 1.704435533731802, -0.07027152902437248)
    #     move_abb(-0.32061698,-0.46094146,0.27820548,-0.015708,1.79315127,0.17505652)
    # if (k == 67):
    #     move_abb(-2.6251, 0.18029, 0.315381, 0.0005235, 1.07634, 0.513475)



    

        

    k += 1
pass


    