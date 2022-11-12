"""human_dummy_operation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import rospy
from std_msgs.msg import Bool
import time
# create the Robot instance.
robot = Supervisor()
human = False
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
rospy.init_node("human_actions")
rate = rospy.Rate(20)

human = False
human_checks = True
last_msg = True
def human_callback(msg):
    global last_msg
    global human
    if (last_msg != msg):
        last_msg = msg
        first_time = True
        if (msg.data == True):
            human = True
            first_time_true = False
            print("human is here")  
            # # appearing(10)
            # appearing(10)
            # transparent_node.setSFFloat(0)
        
        if (msg.data == False):
            human = False
            print("human is not here")  
            # disappearing(10)
            # transparent_node.setSFFloat(1)
  
def i_want_to_wait (times):
    current_time = time.time()
    new_time = time.time()+times
    while robot.step(timestep) != -1:
                if (time.time() >= new_time):
                    break
def disappearing(values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                print(set_values)
                if (set_values >= 1):
                    transparent_node.setSFFloat(1)
                    break
                transparent_node.setSFFloat(set_values)
def appearing(values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                print(set_values)
                if (set_values <= 0):
                    transparent_node.setSFFloat(0)
                    break
                transparent_node.setSFFloat(set_values)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
shape_node = robot.getFromDef("human_dummy_appearance")
transparent_node = shape_node.getField("transparency")

sub_human_actions = rospy.Subscriber("/human_operation_que", Bool,human_callback)
timmer_dummy = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    if (human != human_checks):
        human_checks = human
        if (human == True):
            appearing(20)
        if (human == False):
            disappearing(20)
        
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
