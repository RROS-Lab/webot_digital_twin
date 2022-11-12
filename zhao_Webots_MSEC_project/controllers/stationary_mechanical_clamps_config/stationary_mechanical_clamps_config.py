"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector
from controller import Supervisor

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
connector_battery_PCB = Connector("connector_for_frame1_mechanical_clamps")
connector_battery_PCB.enablePresence(timestep)
right_clamps_motor = robot.getDevice("Mechanical_clamps_right_motor")
left_clamps_motor = robot.getDevice("Mechanical_clamps_left_motor")

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
    if connector_battery_PCB.getPresence() == 1:
            connector_battery_PCB.lock()
            right_clamps_motor.setPosition(0.005)
            left_clamps_motor.setPosition(0.005)
    

            
    
    
   
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
