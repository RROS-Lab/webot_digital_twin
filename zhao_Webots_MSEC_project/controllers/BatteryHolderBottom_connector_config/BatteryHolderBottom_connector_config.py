"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Connector
import time

# create the Robot instance.
robot = Supervisor()
# BatteryBottom_x_axis_motor = robot.getDevice("BatteryBottom_x_axis_motor")
# BatteryBottom_z_axis_motor = robot.getDevice("BatteryBottom_z_axis_motor")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

shape_node = robot.getFromDef("battery_holder_appearance")
transparent_node = shape_node.getField("transparency")

connector_for_battery1 = Connector("connector_for_battery1")
connector_for_battery1.enablePresence(timestep)
connector_for_battery2 = Connector("connector_for_battery2")
connector_for_battery2.enablePresence(timestep)
connector_for_battery3 = Connector("connector_for_battery3")
connector_for_battery3.enablePresence(timestep)
connector_for_battery4 = Connector("connector_for_battery4")
connector_for_battery4.enablePresence(timestep)
connector_for_batterytop = Connector("connector_for_batterytop")
connector_for_batterytop.enablePresence(timestep)
connector_battery_PCB_with_Battery_holder_top = Connector("connector_battery_PCB_with_Battery_holder_top")
connector_battery_PCB_with_Battery_holder_top.enablePresence(timestep)
disppearance_que = Connector("disppearance_que")
disppearance_que.enablePresence(timestep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# def x_motor_movement (distance):
#         while robot.step(timestep) != -1:
#                 BatteryBottom_x_axis_motor.setPosition(distance)
#                 BatteryBottom_x_axis_motor.getPositionSensor().enable(timestep)
#                 x_position = BatteryBottom_x_axis_motor.getPositionSensor().getValue()
#                 if (x_position < distance + 0.01) and (x_position > distance - 0.01):
#                         break

# def z_motor_movement (distance):
#         while robot.step(timestep) != -1:
#                 BatteryBottom_z_axis_motor.setPosition(distance)
#                 BatteryBottom_z_axis_motor.getPositionSensor().enable(timestep)
#                 z_position = BatteryBottom_x_axis_motor.getPositionSensor().getValue()
#                 if (z_position < distance + 0.01) and (z_position > distance - 0.01):
#                         break
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
    if connector_for_battery1.getPresence() == 1:
            connector_for_battery1.lock()
    if connector_for_battery2.getPresence() == 1:
            connector_for_battery2.lock()
    if connector_for_battery3.getPresence() == 1:
            connector_for_battery3.lock()
    if connector_for_battery4.getPresence() == 1:
            connector_for_battery4.lock()
    if connector_for_batterytop.getPresence() == 1:
            connector_for_batterytop.lock()
    if connector_battery_PCB_with_Battery_holder_top.getPresence() == 1:
            connector_battery_PCB_with_Battery_holder_top.lock()
    if (disppearance_que.getPresence() == 1):
        print("battery bottom ready to disappear")
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
