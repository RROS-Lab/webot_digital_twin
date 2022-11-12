"""all_controler controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector
from controller import Supervisor
import rospy
from sensor_msgs.msg import JointState
# from robotiq_gripper.msg import grip_state

# create the Robot instance.
robot = Robot()
rospy.init_node("all_robot_webots")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_names = [
    "joint_1",       "joint_2",       "joint_3",       "joint_4",
    "joint_5",       "joint_6",       "c3_joint_1",    "c3_joint_2",
    "c3_joint_3",    "c3_joint_4",    "c3_joint_5",    "c3_joint_6",
    "s5_joint_1",    "s5_joint_2",    "s5_joint_3",    "s5_joint_4",
    "s5_joint_5",    "s5_joint_6",    "iiwa1_joint_1", "iiwa1_joint_2",
    "iiwa1_joint_3", "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
    "iiwa1_joint_7", "iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6", "iiwa2_joint_7"]
gripper_name = ["bkuka","gkuka"]
linear_motor_name = ["blue_kuka_right_gripper_motor","blue_kuka_left_gripper_motor","green_kuka_right_gripper_motor","green_kuka_left_gripper_motor"]

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
connector_iiwa1 = Connector("connector_iiwa1")
connector_iiwa1.enablePresence(timestep)
connector_iiwa2 = Connector("connector_iiwa2")
connector_iiwa2.enablePresence(timestep)

motor_map = {}
motor_value = {}
gripper_states = {}
linear_motor_name = {}
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    motor_map[name].setVelocity(1.0)
    motor_value[name] = 0
for names in linear_motor_name:
    linear_motor_name[names] = robot.getDevice(names)
    linear_motor_name[names].setVelocity(1.0)
    linear_motor_name[names] = 0

def joint_state(msg):
    for i in range(len(msg.name)):
        if msg.name[i] in motor_value:
            motor_value[msg.name[i]] = msg.position[i]

# def gripper_states_checker (msg):
#     for i in range(len(msg.gripper_name)):
#         gripper_states[msg.gripper_name[i]] = msg.position_state[i]

pub_state = rospy.Subscriber('joint_states', JointState, joint_state)

# pub_state = rospy.Subscriber('gripper_state', grip_state, gripper_states_checker)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    for name in motor_names:
        motor_map[name].setPosition(motor_value[name])
    # for name in gripper_name:
    #     if gripper_states[name] < 230:
    #             if name == "iiwa_blue":
    #               linear_motor_name["blue_kuka_right_gripper_motor"].setPosition(0.5)
    #               linear_motor_name["blue_kuka_left_gripper_motor"].setPosition(-0.5)
    #               if (connector_iiwa1.getPresence() ==1):
    #                     connector_iiwa1.lock()
    #             if name == "iiwa_green":
    #                 linear_motor_name["green_kuka_right_gripper_motor"].setPosition(0.5)
    #                 linear_motor_name["green_kuka_left_gripper_motor"].setPosition(-0.5)
    #                 if (connector_iiwa2.getPresence() ==1):
    #                     connector_iiwa2.lock()
    #     if gripper_states[name] > 230:
    #         if name == "iiwa_blue":
    #               linear_motor_name["blue_kuka_right_gripper_motor"].setPosition(-0.1)
    #               linear_motor_name["blue_kuka_left_gripper_motor"].setPosition(0.1)
                  
    #               connector_iiwa1.unlock()
    #         if name == "iiwa_green":
    #                 linear_motor_name["green_kuka_right_gripper_motor"].setPosition(-0.1)
    #                 linear_motor_name["green_kuka_left_gripper_motor"].setPosition(0.1)
                    
    #                 connector_iiwa2.unlock()
            


    # motor_map["joint_1"].setPosition(2)
    # motor_map["joint_2"].setPosition(2)
    # motor_map["joint_3"].setPosition(2)
    # motor_map["joint_4"].setPosition(2)
    # motor_map["joint_5"].setPosition(2)
    pass
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass
# Enter here exit cleanup code.
