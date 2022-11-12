"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Connector
import time
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# connector_battery_PCB = Connector("connector_battery_PCB_with_Battery_holder_top")
# connector_battery_PCB.enablePresence(timestep)
disppearance_que_pcb = Connector("Battery_PCB_disappearing_cue")
disppearance_que_pcb.enablePresence(timestep)
shape_node = robot.getFromDef("battery_PCB_appearance")
transparent_node = shape_node.getField("transparency")


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
disappear_bool = False
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # if connector_battery_PCB.getPresence() == 1:
    #         connector_battery_PCB.lock()
    if (disppearance_que_pcb.getPresence() == 1):
        print("battery pcb ready to disappear")
        disappear_bool = True
    if (disappear_bool == True):
        i_want_to_wait(4)
        disappearing(20)
        break
    
    
   
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
