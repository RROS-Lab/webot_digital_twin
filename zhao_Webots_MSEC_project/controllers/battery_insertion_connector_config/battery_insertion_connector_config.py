"""human_battery_subassembly_pickup_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Field
from controller import Connector
import time
from std_msgs.msg import String
import rospy
solar_procedure=""
last_msg =""
# create the Robot instance.
robot = Supervisor()


shape_node = robot.getFromDef("insertion_battery_assembly_appearence")
transparent_node = shape_node.getField("transparency")


rospy.init_node("Battery_insertion_inidcator")
rate = rospy.Rate(20)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
appearance_que = Connector("appearance_que")
appearance_que.enablePresence(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

def i_want_to_wait (times):
    current_time = time.time()
    new_time = time.time()+times
    while robot.step(timestep) != -1:
                if (time.time() >= new_time):
                    break
def appearing(values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                i_want_to_wait(0.1)
                set_values = set_values-appearing_rate
                if (set_values <= 0):
                    transparent_node.setSFFloat(0)
                    break
                transparent_node.setSFFloat(set_values)
                
def disappearing(values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                i_want_to_wait(0.1)
                set_values = set_values+appearing_rate
                if (set_values >= 1):
                    transparent_node.setSFFloat(1)
                    break
                transparent_node.setSFFloat(set_values)
def human_callback(msg):
    global last_msg
    global solar_procedure
    if (last_msg != msg):
        last_msg = msg
        first_time = True
        if (msg.data == "solar_start"):
            solar_procedure = "solar_start"
          
            print("human is here")  
            # # appearing(10)
            # appearing(10)
            # transparent_node.setSFFloat(0)
        if (msg.data == "frame4_finished"):
            solar_procedure = "frame4_finished"
# Main loop:
# - perform simulation steps until Webots is stopping the controller
sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)
f = 1
g = 1
disappear_bool = True
first_time = True
begining = True
time_to_move = False
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # x_motor_movement(-1)
    # if (disappear_bool == True) and (begining == True):
    #     disappearing(1)
    #     begining = False
    if (appearance_que.getPresence() == 1):
        print("battery subassembly is ready to appear")
        disappear_bool = False
    if (disappear_bool == False ) and (first_time == True):
        appearing(1)
        time_to_move = True
        first_time = False
    if (solar_procedure == "frame4_finished"):
        disappearing(2)
        break


    

    pass

# Enter here exit cleanup code.
