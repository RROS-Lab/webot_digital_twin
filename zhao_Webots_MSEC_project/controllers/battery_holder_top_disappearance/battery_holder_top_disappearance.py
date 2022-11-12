from controller import Supervisor
from controller import Connector
import time
# create the Robot instance.
robot = Supervisor()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
disppearance_que = Connector("disppearance_que")
disppearance_que.enablePresence(timestep)

shape_node = robot.getFromDef("battery_holder_top_appearance")
transparent_node = shape_node.getField("transparency")

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

    # Process sensor data here.
    if (disppearance_que.getPresence() == 1):
        print("battery holder top ready to disappear")
        disappear_bool = True
    if (disappear_bool == True):
        i_want_to_wait(4)
        disappearing(20)
        break
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
