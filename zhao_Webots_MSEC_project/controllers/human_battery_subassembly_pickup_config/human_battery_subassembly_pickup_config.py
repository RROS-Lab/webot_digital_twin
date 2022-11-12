"""human_battery_subassembly_pickup_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector
from time import time, sleep

# create the Robot instance.
robot = Robot()
human_x_axis_motor = robot.getDevice("human_x_axiz_motor")
human_z_axis_motor = robot.getDevice("human_z_axiz_motor")
human_y_axis_motor = robot.getDevice("human_y_axiz_motor")

human_x_axis_sensor = robot.getDevice("human_x_axiz_sensor")
human_y_axis_sensor = robot.getDevice("human_y_axiz_sensor")
human_z_axis_sensor = robot.getDevice("human_z_axiz_sensor")

connector_Battery_delivery_pickup = Connector("Battery_delivery_pickup")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
connector_Battery_delivery_pickup.enablePresence(timestep)
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
def y_motor_movement (distance):
        while robot.step(timestep) != -1:
                human_y_axis_motor.setPosition(distance)
                human_y_axis_sensor.enable(timestep)
                y_position = human_y_axis_sensor.getValue()
                if (y_position < distance + 0.01) and (y_position > distance - 0.01):
                        break
def z_motor_movement (distance):
        while robot.step(timestep) != -1:
                human_z_axis_motor.setPosition(distance)
                human_z_axis_sensor.enable(timestep)
                z_position = human_z_axis_sensor.getValue()
                if (z_position < distance + 0.01) and (z_position > distance - 0.01):
                        break
# Main loop:
# - perform simulation steps until Webots is stopping the controller
f = 1
g = 1
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # x_motor_movement(-1)
    if (g == 4000):
    
        human_z_axis_motor.setPosition(-0.4)
    # Process sensor data here.
    if (connector_Battery_delivery_pickup.getPresence() == 1):
        connector_Battery_delivery_pickup.lock()
        sleep(2)
        f = 2
    if (f == 2):
        human_z_axis_motor.setPosition(1)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    g+=1
    pass

# Enter here exit cleanup code.
