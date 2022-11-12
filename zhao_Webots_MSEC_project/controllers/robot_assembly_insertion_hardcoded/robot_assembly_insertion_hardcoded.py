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
    
    motor_map[name].setVelocity(1.0)
    motor_value[name] = 0
    current_joint_states_motor_value[name] = 0
for name in gripper_name:
    gripper_motor[name] = []
    gripper_motor_values[name] = []
    for motor_name in gripper_moter_map[name]:
        device = robot.getDevice(motor_name)
        device.setVelocity(1.0)
        gripper_motor[name].append(device)
        # gripper_motor_values[name].append(0)

def green_kuka_gripper (distance1, distance2):
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
                print("motor1_position")
                print(motor1_position)
                print("distance1")
                print(distance1)
                if (abs(motor1_position) < abs(distance1) + 0.01) and (abs(motor1_position) > abs(distance1) - 0.01):
                        break
    

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
        print(name)
        print(s5_actual_joint[name])      
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
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0
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
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0

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
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0
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
        if (flag >= len(joint_names)):
         break
        else:
            flag = 0
while robot.step(timestep) != -1:
    #battery assembly insertion
    if (k == 1):
        move_iiwa2(-1.4853187178251799, 0.18084977243333122, -0.012792435939004644, -1.3012541372757405, 0.003175527864322066, 1.6499244066476821, -1.477611111424677)
        move_iiwa2(-1.4890070375165199, 0.26456664269610725, -0.015458958710279766, -1.198977656616544, 0.004988373101269204, 1.668504204755015, -1.4833842345833228)
        move_iiwa2(-0.9626884782834847, 0.13160593278664376, 0.17109617469509666, -1.2616165252941482, -0.028178007638044948, 1.7423638851213479, -0.7779989419109549)
        move_iiwa2(-0.9945558048844286, 0.1765686480889582, 0.16522442659956288, -1.285981845145522, -0.03395276420070758, 1.6731740921454277, 0.7275898242450933)
        move_iiwa2(-0.9970087097183195, 0.1704300002849107, 0.16405568554206829, -1.3908970850356888, -0.03255660338883964, 1.5743271060269182, 0.727521881618171)
        move_iiwa2(-0.9632022326266327, 0.548208006821397, 0.1283291855102678, -1.890043937722389, -0.11097908902158392, 0.7017644614226658, 0.794)
    if (k == 2):
        if (connector_iiwa2.getPresence() == 1):
            connector_iiwa2.lock() #battery_pickup
    if (k == 3):
        move_iiwa2(-0.9945558048844286, 0.1765686480889582, 0.16522442659956288, -1.285981845145522, -0.03395276420070758, 1.6731740921454277, 0.7275898242450933)
        # move_iiwa2(-0.9945558048844286, 0.1765686480889582, 0.16522442659956288, -1.285981845145522, -0.03395276420070758, 1.6731740921454277,-0.855)
        move_iiwa2(-0.3717424899879282, 0.44245231528084505, 0.377105415244032, -1.7821307502434782, -0.20622570390093206, 0.952307403872376, 0.07299423703622572)
        move_iiwa2(-0.28270265013890733, 0.44254595579653755, 0.3943094426721484, -1.7854027637073504, -0.2148406156101614, 0.953417067857509, 0.18322737424185576)
    if (k == 4):
        connector_iiwa2.unlock()
    # if (k == 4):
    #     connector_iiwa2.unlock()
    # if (k == 5):
    #     move_iiwa2(-0.3717424899879282, 0.44245231528084505, 0.377105415244032, -1.7821307502434782, -0.20622570390093206, 0.952307403872376, 0.07299423703622572)
    #     move_iiwa2(-0.6098872515351488, 0.5057826882680602, 0.3237828346750281, -1.677518949782331, -0.19599096903419913, 0.9845335228855924, -0.23487733167552544)
    #     move_iiwa2(-0.5848858601477773, 0.6505445153989624, 0.29233663535664334, -1.7255968924946545, -0.2579539666451226, 0.7975517446900237, -0.1876286017841119)
    #     green_kuka_gripper(-0.05,0.05)
    #     move_iiwa2(-0.33666349422808334, 0.4412990855092408, 0.3842752000897208, -1.7854105063457224, -0.20932945842204775, 0.9520858055883404, 0.11673354599501448)
       
    

        

    k += 1
pass


    