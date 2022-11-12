"""human_battery_subassembly_pickup_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Field
from controller import Connector
import time

# create the Robot instance.
robot = Supervisor()
human_x_axis_motor = robot.getDevice("x_axis_motor")
human_z_axis_motor = robot.getDevice("z_axis_motor")


human_x_axis_sensor = robot.getDevice("x_axis_sensor")
human_z_axis_sensor = robot.getDevice("z_axis_sensor")

shape_node = robot.getFromDef("battery_assembly_appearence")
transparent_node = shape_node.getField("transparency")



# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
disppearance_que = Connector("disppearance_que")
disppearance_que.enablePresence(timestep)
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

def z_motor_movement (distance):
        while robot.step(timestep) != -1:
                human_z_axis_motor.setPosition(distance)
                human_z_axis_sensor.enable(timestep)
                z_position = human_z_axis_sensor.getValue()
                if (z_position < distance + 0.01) and (z_position > distance - 0.01):
                        break
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
first_time = True
time_to_move = False
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # x_motor_movement(-1)
    if (disppearance_que.getPresence() == 1):
        print("battery subassembly is ready to appear")
        disappear_bool = False
    if (disappear_bool == False ) and (first_time == True):
        i_want_to_wait(4)
        appearing(20)
        time_to_move = True
        first_time = False
    if (time_to_move == True):
        if (f == 1):
            x_motor_movement(-0.4)
            z_motor_movement(-2.9)
            x_motor_movement(0.8)
            z_motor_movement(-2.04)
            x_motor_movement(1.02)
        if (f == 2):
            disappearing(10)
            break



        f +=1
    

    pass

# Enter here exit cleanup code.
