"""all_controler controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
import rospy
from sensor_msgs.msg import JointState

# create the Robot instance.
robot = Robot()
rospy.init_node("all_robot_webots")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print(timestep)

motor_names = [
    "joint_1",       "joint_2",       "joint_3",       "joint_4",
    "joint_5",       "joint_6",       "c3_joint_1",    "c3_joint_2",
    "c3_joint_3",    "c3_joint_4",    "c3_joint_5",    "c3_joint_6",
    "s5_joint_1",    "s5_joint_2",    "s5_joint_3",    "s5_joint_4",
    "s5_joint_5",    "s5_joint_6",    "iiwa1_joint_1", "iiwa1_joint_2",
    "iiwa1_joint_3", "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
    "iiwa1_joint_7", "iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6", "iiwa2_joint_7"]

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

motor_map = {}
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    motor_map[name].setVelocity(1.0)
def joint_state(msg):
    for i in range(len(msg.name)):
        if msg.name[i] in motor_map:
            motor_map[msg.name[i]].setPosition(msg.position[i])
    
pub_state = rospy.Subscriber('joint_states', JointState, joint_state)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
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
