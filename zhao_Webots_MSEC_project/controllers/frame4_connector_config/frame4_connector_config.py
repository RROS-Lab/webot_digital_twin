"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
connector_frame4_frame2 = Connector("connector_frame4_frame2")
connector_frame4_frame2.enablePresence(timestep)
connector_frame4_to_endcap_neg = Connector("connector_frame4_to_endcap_neg")
connector_frame4_to_endcap_neg.enablePresence(timestep)
connector_frame4_to_endcap_pos = Connector("connector_frame4_to_endcap_pos")
connector_frame4_to_endcap_pos.enablePresence(timestep)
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
    if connector_frame4_frame2.getPresence() == 1:
            connector_frame4_frame2.lock()
    if connector_frame4_to_endcap_neg.getPresence() == 1:
            connector_frame4_to_endcap_neg.lock()
    if connector_frame4_to_endcap_pos.getPresence() == 1:
            connector_frame4_to_endcap_pos.lock()
            
            
    
    
   
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
