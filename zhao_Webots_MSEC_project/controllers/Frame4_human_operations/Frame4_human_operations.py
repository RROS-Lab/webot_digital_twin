"""human_battery_subassembly_pickup_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Field
from controller import Connector
import time
import rospy
from std_msgs.msg import String

# create the Robot instance.
robot = Supervisor()
human_x_axis_motor = robot.getDevice("frame4_linear_motor")



human_x_axis_sensor = robot.getDevice("frame4_linear_sensor")


shape_node = robot.getFromDef("Frame4_appearance")
transparent_node = shape_node.getField("transparency")



# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

rospy.init_node("frame4")
rate = rospy.Rate(20)
solar_procedure = ""

last_msg = ""
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
def x_motor_movement (distance):
        while robot.step(timestep) != -1:
                human_x_axis_motor.setPosition(distance)
                human_x_axis_sensor.enable(timestep)
                x_position = human_x_axis_sensor.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break

def human_callback(msg):
    global last_msg
    global solar_procedure
    print("I read your mind")
    if (last_msg != msg):
        last_msg = msg
        
       
        if (msg.data == "frame4_start"):
            solar_procedure = "frame4_start"
            print("Frame4 engaged")
        if (msg.data == "frame4_finished"):
            solar_procedure = "frame4_finished"
            
        

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
# Main loop:
# - perform simulation steps until Webots is stopping the controller
f = 1
g = 1
disappear_bool = True
standby = False
first_time = True
time_to_move = False
sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)


while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # x_motor_movement(-1)
    if (disappear_bool == True) and (standby == False):
        disappearing(1)
        standby = True
    if (solar_procedure == "frame4_start") and (time_to_move == False):
 
        print(" disappearing is finished")
        disappear_bool == False
    # if (disappear_bool == False) and (first_time == True):
        print(" frame4_start_appearing")
        appearing(20)
        time_to_move = True
        
    if (time_to_move == True) and (first_time == True):
        
            x_motor_movement(0.09)
            first_time = False
    if (solar_procedure == "frame4_finished"):
        disappearing(10)
        break



    f +=1
    

    pass

# Enter here exit cleanup code.
