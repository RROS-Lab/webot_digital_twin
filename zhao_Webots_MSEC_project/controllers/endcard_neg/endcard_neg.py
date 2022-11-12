"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Connector
from std_msgs.msg import String
import rospy

# create the Robot instance.
robot = Supervisor()
rospy.init_node("endcard_neg")
rate = rospy.Rate(20)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
solar_procedure = ""
last_msg = ""
firsttime = True
def disappearing(values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    transparent_node.setSFFloat(1)
                    break
                transparent_node.setSFFloat(set_values)
def appearing(values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    transparent_node.setSFFloat(0)
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
        if (msg.data == "endcard_negative_finished_half"):
            solar_procedure = "endcard_negative_finished_half"
            
shape_node = robot.getFromDef("endcap_neg_appearance")
transparent_node = shape_node.getField("transparency")
sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    if (solar_procedure == "endcard_negative_finished_half") and (firsttime == True):
        disappearing(2)
        firsttime = False
        break
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
