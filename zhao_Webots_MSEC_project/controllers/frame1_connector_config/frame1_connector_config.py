"""frame1_connector_config controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import Connector
from std_msgs.msg import String
import rospy

# create the Robot instance.
robot = Supervisor()
rospy.init_node("frame1")
rate = rospy.Rate(20)
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
connector_frame1_backplane = Connector("connector_for_Backplane_PCB")
connector_frame1_backplane.enablePresence(timestep)
connector_frame1_frame3 = Connector("Frame_1_Frame_3_connector")
connector_frame1_frame3.enablePresence(timestep)
connector_frame1_frame2 = Connector("frame1_frame2_connector")
connector_frame1_frame2.enablePresence(timestep)
Antenna_insertion_connector = Connector("Antenna_insertion_connector")
Antenna_insertion_connector.enablePresence(timestep)
Card_With_Connector_insertion_connector = Connector("Card_With_Connector_insertion_connector")
Card_With_Connector_insertion_connector.enablePresence(timestep)
C3_CARD_insertion_connector = Connector("C3_CARD_insertion_connector")
C3_CARD_insertion_connector.enablePresence(timestep)
WIFI_CARD_insertion_connector = Connector("WIFI_CARD_insertion_connector")
WIFI_CARD_insertion_connector.enablePresence(timestep)
Star_Tracker_insertion_connector = Connector("Star_Tracker_insertion_connector")
Star_Tracker_insertion_connector.enablePresence(timestep)
ADAS_insertion_connector = Connector("ADAS_insertion_connector")
ADAS_insertion_connector.enablePresence(timestep)
Battery_insertion_connector = Connector("Battery_insertion_connector")
Battery_insertion_connector.enablePresence(timestep)



solar_procedure = ""
last_msg = ""
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
    global battery_connection
    print("Frame 1 got your message")
    if (last_msg != msg):
        last_msg = msg
        first_time = True
        if (msg.data == "antenna_inserted"):
                Antenna_insertion_connector.lock()
                solar_procedure = "antenna_inserted"
        if (msg.data == "battery_inserted"):
                Battery_insertion_connector.lock()
                solar_procedure = "battery_inserted"
                
                battery_connection = True
                print("solar_procedure")
                print (solar_procedure)
        if (msg.data == "star_tracker_inserted"):
                solar_procedure = "star_tracker_inserted"
                Star_Tracker_insertion_connector.lock()
        if (msg.data == "solar_start"):
            solar_procedure = "solar_start"
          
            print("human is here")  
            # # appearing(10)
            # appearing(10)
            # transparent_node.setSFFloat(0)
        if (msg.data == "frame4_finished"):
            solar_procedure = "frame4_finished"
            
shape_node = robot.getFromDef("Frame_1_appearance")
transparent_node = shape_node.getField("transparency")
sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
battery_connection = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    if (solar_procedure == "battery_inserted"):
        Battery_insertion_connector.lock()
        battery_connection = True
        # print("battery_connection")
        # print (battery_connection)
    if connector_frame1_backplane.getPresence() == 1:
            connector_frame1_backplane.lock()
    if connector_frame1_frame3.getPresence() == 1:
            connector_frame1_frame3.lock()
    if connector_frame1_frame2.getPresence() == 1:
            connector_frame1_frame2.lock()

#     if Antenna_insertion_connector.getPresence() == 1:
#             Antenna_insertion_connector.lock()
    if Card_With_Connector_insertion_connector.getPresence() == 1:
            Card_With_Connector_insertion_connector.lock()
    if C3_CARD_insertion_connector.getPresence() == 1:
            C3_CARD_insertion_connector.lock()   
    if WIFI_CARD_insertion_connector.getPresence() == 1:
            WIFI_CARD_insertion_connector.lock()
#     if Star_Tracker_insertion_connector.getPresence() == 1:
#             Star_Tracker_insertion_connector.lock()
    if ADAS_insertion_connector.getPresence() == 1:
            ADAS_insertion_connector.lock()  
#     if Battery_insertion_connector.getPresence() == 1 :
            
#             Battery_insertion_connector.lock()  
    if ( battery_connection == True):
        Battery_insertion_connector.lock()  
    # Process sensor data here.
    if (solar_procedure == "frame4_finished"):
        disappearing(2)
        break
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
