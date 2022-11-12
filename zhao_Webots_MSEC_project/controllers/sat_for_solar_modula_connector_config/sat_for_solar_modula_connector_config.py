"""human_dummy_operation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import rospy
from controller import Connector
from std_msgs.msg import String
import time
# create the Robot instance.
robot = Supervisor()
sat_sliding_y_axis_motor = robot.getDevice("sat_sliding_y_axis_motor")
sat_sliding_y_axis_sensor = robot.getDevice("sat_sliding_y_axis_sensor")


sat_sliding_z_axis_motor = robot.getDevice("sat_sliding_z_axis_motor")
sat_sliding_z_axis_sensor = robot.getDevice("sat_sliding_z_axis_sensor")

rotation_x_axis_motor = robot.getDevice("sat_x_axis_motor")
rotation_x_axis_sensor = robot.getDevice("sat_x_axis_sensor")

rotation_y_axis_motor_2 = robot.getDevice("sat_y_axis_motor_2")
rotation_y_axis_sensor_2 = robot.getDevice("sat_y_axis_sensor_2")

rotation_y_axis_motor = robot.getDevice("sat_y_axis_motor")
rotation_y_axis_sensor = robot.getDevice("sat_y_axis_sensor")

rotation_z_axis_motor = robot.getDevice("sat_z_axis_motor")
rotation_z_axis_sensor = robot.getDevice("sat_z_axis_sensor")

def rotation_x_motor_movement_solar_1_and_solar_3(distance):
     while robot.step(timestep) != -1:
                rotation_x_axis_motor.setPosition(distance)
                rotation_x_axis_sensor.enable(timestep)
                x_position = rotation_x_axis_sensor.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break
def rotation_y_motor_movement_solar_2_and_solar_4(distance):
     while robot.step(timestep) != -1:
                rotation_y_axis_motor_2.setPosition(distance)
                rotation_y_axis_sensor_2.enable(timestep)
                x_position = rotation_y_axis_sensor_2.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break
def rotation_z_motor_movement(distance):
     while robot.step(timestep) != -1:
                rotation_z_axis_motor.setPosition(distance)
                rotation_z_axis_sensor.enable(timestep)
                x_position = rotation_z_axis_sensor.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break

def rotation_y_motor_movement(distance):
     while robot.step(timestep) != -1:
                rotation_y_axis_motor.setPosition(distance)
                rotation_y_axis_sensor.enable(timestep)
                x_position = rotation_y_axis_sensor.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break

def slider_x_motor_movement (distance):
        while robot.step(timestep) != -1:
                sat_sliding_y_axis_motor.setPosition(distance)
                sat_sliding_y_axis_sensor.enable(timestep)
                x_position = sat_sliding_y_axis_sensor.getValue()
                if (x_position < distance + 0.01) and (x_position > distance - 0.01):
                        break

def slider_z_motor_movement (distance):
        while robot.step(timestep) != -1:
                sat_sliding_z_axis_motor.setPosition(distance)
                sat_sliding_z_axis_sensor.enable(timestep)
                z_position = sat_sliding_z_axis_sensor.getValue()
                if (z_position < distance + 0.01) and (z_position > distance - 0.01):
                        break
human = False
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

solar_4_connector = Connector("solar_4_connector")
solar_4_connector.enablePresence(timestep)
solar_3_connector = Connector("solar_3_connector")
solar_3_connector.enablePresence(timestep)
solar_2_connector = Connector("solar_2_connector")
solar_2_connector.enablePresence(timestep)
solar_1_connector = Connector("solar_1_connector")
solar_1_connector.enablePresence(timestep)
endcard_pos_connector = Connector("encard_pos_connector")
endcard_pos_connector.enablePresence(timestep)
encard_neg_connector = Connector("encard_neg_connector")
encard_neg_connector.enablePresence(timestep)

rospy.init_node("sat_solar")
rate = rospy.Rate(20)

solar_procedure = ""
human_checks = True
last_msg = ""
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
            
        if (msg.data == "solar_module_1_finished"):
            solar_procedure = "solar_module_1_finished"
        if (msg.data == "solar_module_2_finished"):
            solar_procedure = "solar_module_2_finished"
        if (msg.data == "solar_module_3_finished"):
            solar_procedure = "solar_module_3_finished"
        if (msg.data == "solar_module_4_finished"):
            solar_procedure = "solar_module_4_finished"
        if (msg.data == "endcard_negative_finished"):
            solar_procedure = "endcard_negative_finished"
        if (msg.data == "endcard_positive_finished"):
            solar_procedure = "endcard_positive_finished"

            
        if (msg.data == "solar_module_1_finished_half"):
            solar_procedure = "solar_module_1_finished_half"
        if (msg.data == "solar_module_2_finished_half"):
            solar_procedure = "solar_module_2_finished_half"
        if (msg.data == "solar_module_3_finished_half"):
            solar_procedure = "solar_module_3_finished_half"
        if (msg.data == "solar_module_4_finished_half"):
            solar_procedure = "solar_module_4_finished_half"
        if (msg.data == "endcard_negative_finished_half"):
            solar_procedure = "endcard_negative_finished_half"
        if (msg.data == "endcard_positive_finished_half"):
            solar_procedure = "endcard_positive_finished_half" 
  
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
def solar_1_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    solar_1_transparent_node.setSFFloat(0)
                    break
                solar_1_transparent_node.setSFFloat(set_values)
def solar_2_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    solar_2_transparent_node.setSFFloat(0)
                    break
                solar_2_transparent_node.setSFFloat(set_values)
def solar_3_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    solar_3_transparent_node.setSFFloat(0)
                    break
                solar_3_transparent_node.setSFFloat(set_values)
def solar_4_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    solar_4_transparent_node.setSFFloat(0)
                    break
                solar_4_transparent_node.setSFFloat(set_values)
def solar_1_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    solar_1_transparent_node.setSFFloat(1)
                    break
                solar_1_transparent_node.setSFFloat(set_values)
def solar_2_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    solar_2_transparent_node.setSFFloat(1)
                    break
                solar_2_transparent_node.setSFFloat(set_values)
def solar_3_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    solar_3_transparent_node.setSFFloat(1)
                    break
                solar_3_transparent_node.setSFFloat(set_values)
def solar_4_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    solar_4_transparent_node.setSFFloat(1)
                    break
                solar_4_transparent_node.setSFFloat(set_values)
def endcap_pos_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    endcap_pos_transparent_node.setSFFloat(0)
                    break
                endcap_pos_transparent_node.setSFFloat(set_values)
def endcap_pos_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    endcap_pos_transparent_node.setSFFloat(1)
                    break
                endcap_pos_transparent_node.setSFFloat(set_values)
def endcap_neg_appearing (values):
    set_values = 1
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values-appearing_rate
                #print(set_values)
                if (set_values <= 0):
                    endcap_neg_transparent_node.setSFFloat(0)
                    break
                endcap_neg_transparent_node.setSFFloat(set_values)
def endcap_neg_disappearing (values):
    set_values = 0
    while robot.step(timestep) != -1:
                appearing_rate = 1/values
                
                set_values = set_values+appearing_rate
                #print(set_values)
                if (set_values >= 1):
                    endcap_neg_transparent_node.setSFFloat(1)
                    break
                endcap_neg_transparent_node.setSFFloat(set_values)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
shape_node = robot.getFromDef("sat_solar_module_appearance")
transparent_node = shape_node.getField("transparency")


solar_1_shape_node = robot.getFromDef("display_solar_1")
solar_1_transparent_node = solar_1_shape_node.getField("transparency")
solar_2_shape_node = robot.getFromDef("display_solar_2")
solar_2_transparent_node = solar_2_shape_node.getField("transparency")
solar_3_shape_node = robot.getFromDef("display_solar_3")
solar_3_transparent_node = solar_3_shape_node.getField("transparency")
solar_4_shape_node = robot.getFromDef("display_solar_4")
solar_4_transparent_node = solar_4_shape_node.getField("transparency")
endcap_pos_shape_node = robot.getFromDef("endcap_pos_display")
endcap_pos_transparent_node = endcap_pos_shape_node.getField("transparency")
endcap_neg_shape_node = robot.getFromDef("endcap_neg_display")
endcap_neg_transparent_node = endcap_neg_shape_node.getField("transparency")

sub_human_actions = rospy.Subscriber("/solar_module_procedure", String,human_callback)
timmer_dummy = 0
first_operation_done = False
Solar_module1_done = False
Solar_module2_done = False
Solar_module3_done = False
Solar_module4_done = False
endcap_pos_done = False
endcap_neg_done = False
Solar_module1_half_done = False
Solar_module2_half_done = False
Solar_module3_half_done = False
Solar_module4_half_done = False
endcap_pos_half_done = False
endcap_neg_half_done = False
solar_module_end_cap_not_ready = True

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    if (solar_module_end_cap_not_ready == True):
        solar_1_disappearing(1)
        solar_2_disappearing(1)
        solar_3_disappearing(1)
        solar_4_disappearing(1)
        endcap_pos_disappearing(1)
        endcap_neg_disappearing(1)
        disappearing(1)
        solar_module_end_cap_not_ready = False
    if (solar_1_connector.getPresence() == 1):
        solar_1_connector.lock()
    if (solar_2_connector.getPresence() == 1):
        solar_2_connector.lock()
    if (solar_3_connector.getPresence() == 1):
        solar_3_connector.lock()
    if (solar_4_connector.getPresence() == 1):
        solar_4_connector.lock()
    if (endcard_pos_connector.getPresence() == 1):
        endcard_pos_connector.lock()
    if (encard_neg_connector.getPresence() == 1):
        encard_neg_connector.lock()

    if (solar_procedure == "frame4_finished") and (first_operation_done == False):
        print("frame4_finished")
        appearing(2)
        slider_x_motor_movement(0.46)
        rotation_y_motor_movement(1.57)
        slider_z_motor_movement(-0.05)
        first_operation_done = True
    if (solar_procedure == "solar_module_1_finished_half") and (Solar_module1_half_done == False):
        
        print("solar_module_1_finished_half")
        solar_1_appearing(1)
        solar_1_connector.unlock()
        slider_z_motor_movement(0)
        
        rotation_x_motor_movement_solar_1_and_solar_3(3.14)
        slider_z_motor_movement(-0.05)
        Solar_module1_half_done = True
    if (solar_procedure == "solar_module_1_finished") and (Solar_module1_done == False):
        print("solar_module_1_finished")
        slider_z_motor_movement(0)
        rotation_x_motor_movement_solar_1_and_solar_3(0)
      
        rotation_z_motor_movement(1.57)
        slider_z_motor_movement(-0.05)
        Solar_module1_done = True
    if (solar_procedure == "solar_module_2_finished_half") and (Solar_module2_half_done == False):
        solar_2_appearing(1)
        solar_2_connector.unlock()
        print("solar_module_2_finished_half")
        slider_z_motor_movement(0)
        
        rotation_y_motor_movement_solar_2_and_solar_4(3.14)
        slider_z_motor_movement(-0.05)
        Solar_module2_half_done = True
    if (solar_procedure == "solar_module_2_finished") and (Solar_module2_done == False):
        print("solar_module_2_finished")
        slider_z_motor_movement(0)
        
        rotation_y_motor_movement_solar_2_and_solar_4(0)
        rotation_z_motor_movement(3.14)
        slider_z_motor_movement(-0.05)
        Solar_module2_done = True
    if (solar_procedure == "solar_module_3_finished_half") and (Solar_module3_half_done == False):
        solar_3_appearing(1)
        solar_3_connector.unlock()
        print("solar_module_3_finished_half")
        slider_z_motor_movement(0)
        
        rotation_x_motor_movement_solar_1_and_solar_3(3.14)
        slider_z_motor_movement(-0.05)
        Solar_module3_half_done = True
    if (solar_procedure == "solar_module_3_finished") and (Solar_module3_done == False):
        print("solar_module_3_finished")
        slider_z_motor_movement(0)
        
        rotation_x_motor_movement_solar_1_and_solar_3(0)
        rotation_z_motor_movement(4.712)
        slider_z_motor_movement(-0.05)
        Solar_module3_done = True
    if (solar_procedure == "solar_module_4_finished_half") and (Solar_module4_half_done == False):
        solar_4_appearing(1)
        solar_4_connector.unlock()
        print("solar_module_4_finished_half")
        slider_z_motor_movement(0)
        
        rotation_y_motor_movement_solar_2_and_solar_4(3.14)
        slider_z_motor_movement(-0.05)
        Solar_module4_half_done = True
    if (solar_procedure == "solar_module_4_finished") and (Solar_module4_done == False):
        print("solar_module_4_finished")
        slider_z_motor_movement(0)
        
        rotation_x_motor_movement_solar_1_and_solar_3(0)
        rotation_z_motor_movement(0)
        rotation_y_motor_movement(0)
        slider_z_motor_movement(-0.05)
        Solar_module4_done = True
    if (solar_procedure == "endcard_negative_finished_half") and (endcap_neg_half_done == False):
        endcap_neg_appearing(1)
        encard_neg_connector.unlock()
        print("endcard_negative_finished_half")
        slider_z_motor_movement(0)
        
       
        rotation_z_motor_movement(3.14)
       
        slider_z_motor_movement(-0.05)
        endcap_neg_half_done = True
    if (solar_procedure == "endcard_negative_finished") and (endcap_neg_done == False):
        print("endcard_negative_finished")
        slider_z_motor_movement(0)
        
       
        rotation_z_motor_movement(0)
        rotation_y_motor_movement_solar_2_and_solar_4(0)

        slider_z_motor_movement(-0.05)
        endcap_neg_done = True
    if (solar_procedure == "endcard_positive_finished_half") and (endcap_pos_half_done == False):
        endcap_pos_appearing(1)
        endcard_pos_connector.unlock()
        print("endcard_positive_finished_half")
        slider_z_motor_movement(0)
        
       
        rotation_z_motor_movement(3.14)
        

        slider_z_motor_movement(-0.05)
        endcap_pos_half_done = True
    if (solar_procedure == "endcard_positive_finished") and (endcap_pos_done == False):
        print("endcard_positive_finished")
        slider_z_motor_movement(0)
        
       
        rotation_z_motor_movement(0)
        

        slider_z_motor_movement(-0.05)
        slider_z_motor_movement(2)
        endcap_pos_done = True
    
    



    if (solar_procedure == "solar_start"):
        disappearing(1)

    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
