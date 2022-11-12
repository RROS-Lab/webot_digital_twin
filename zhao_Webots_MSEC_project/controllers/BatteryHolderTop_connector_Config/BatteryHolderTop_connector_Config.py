"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
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
                
            
    
    
   
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
