"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Connector
import rospy
from std_msgs.msg import String
# create the Robot instance.
robot = Supervisor()
rospy.init_node("star_tracker_pcb")
rate = rospy.Rate(20)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
connector_startracker_to_camera = Connector("connector_star_tracker_pcb_to_camera")
connector_startracker_to_camera.enablePresence(timestep)
connector_star_tracker_stabilization = Connector("connector_star_tracker_stabilization ")
connector_star_tracker_stabilization.enablePresence(timestep)
connector_s5_pickup = Connector("connector_s5_pickup")
connector_s5_pickup.enablePresence(timestep)
connector_iiwa2_pickup = Connector("connector_iiwa2_pickup")
connector_iiwa2_pickup.enablePresence(timestep)
iiwa2_star_tracker_pickup_notice = Connector("iiwa2_star_tracker_pickup_notice")
iiwa2_star_tracker_pickup_notice.enablePresence(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
solar_procedure = ""
last_msg =""
firsttime = True
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

shape_node = robot.getFromDef("star_tracker_pcb_appearance")
transparent_node = shape_node.getField("transparency")
sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # print("connector_s5_pickup.getPresence()")
    # print(connector_s5_pickup.getPresence())
 
    if (solar_procedure == "frame4_finished")and (firsttime == True):
        disappearing(2)
        firsttime = False
        break
    if connector_startracker_to_camera.getPresence() == 1:
            connector_startracker_to_camera.lock()
            if (connector_star_tracker_stabilization.getPresence() == 1):
                connector_star_tracker_stabilization.lock()
    if iiwa2_star_tracker_pickup_notice.getPresence() == 1:
        connector_star_tracker_stabilization.unlock()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
