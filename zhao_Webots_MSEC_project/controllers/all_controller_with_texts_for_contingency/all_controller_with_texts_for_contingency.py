"""all_controler controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Connector
from controller import Supervisor
from controller import Keyboard
from time import time, sleep
import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from std_msgs.msg import Bool
from std_msgs.msg import String
# from robotiq_gripper.msg import change_state
# from robotiq_gripper.msg import grip_state

# from robotiq_gripper.msg import grip_state

# create the Robot instance.
robot = Supervisor()
keyboard_input = Keyboard()
rospy.init_node("all_robot_webots")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#setting up motor names with respects to the URDF definition 
motor_names = [
    "joint_1",       "joint_2",       "joint_3",       "joint_4",
    "joint_5",       "joint_6",       "c3_joint_1",    "c3_joint_2",
    "c3_joint_3",    "c3_joint_4",    "c3_joint_5",    "c3_joint_6",
    "s5_joint_1",    "s5_joint_2",    "s5_joint_3",    "s5_joint_4",
    "s5_joint_5",    "s5_joint_6","iiwa1_joint_1", "iiwa1_joint_2",
    "iiwa1_joint_3","iiwa1_joint_4","iiwa1_joint_5","iiwa1_joint_6",
    "iiwa1_joint_7", "iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6", "iiwa2_joint_7"]
gripper_name = ["bkuka","gkuka","s5"]
# linear_motor_name = ["blue_kuka_right_gripper_motor","blue_kuka_left_gripper_motor","green_kuka_right_gripper_motor","green_kuka_left_gripper_motor"]
gripper_moter_map = {
    "bkuka":["blue_kuka_left_gripper_motor", "blue_kuka_right_gripper_motor"],
    "gkuka":["green_kuka_left_gripper_motor", "green_kuka_right_gripper_motor"],
    "s5":[]
}

#setting up the connector for the robots manipulator
connector_iiwa1 = Connector("connector_iiwa1")
connector_iiwa1.enablePresence(timestep)
connector_iiwa2 = Connector("connector_iiwa2")
connector_iiwa2.enablePresence(timestep)
connector_c3 = Connector("connector_c3")
connector_c3.enablePresence(timestep)
connector_s5 = Connector("connector_s5")
connector_s5.enablePresence(timestep)

motor_map = {}
motor_value = {}
iiwa1_motor = {}
current_joint_states_motor_value = {}
gripper_states = {}
gripper_motor = {}
gripper_motor_values = {}
connector_map = {}
planned_motor_values = {}

#setting up the motor
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    motor_map[name].setVelocity(1.0)
    motor_value[name] = 0
    current_joint_states_motor_value[name] = 0
for name in gripper_name:
    gripper_motor[name] = []
    gripper_motor_values[name] = []
    for motor_name in gripper_moter_map[name]:
        device = robot.getDevice(motor_name)
        device.setVelocity(1.0)
        gripper_motor[name].append(device)
        gripper_motor_values[name].append(0)

#getting the current joint state of the robot
def joint_state(msg):
    for i in range(len(msg.name)):
        if msg.name[i] in motor_value:
            current_joint_states_motor_value[msg.name[i]] = msg.position[i]

#function for green kuka gripper movements
def iiwa2_gripper (distance1, distance2):
    motor1 = robot.getDevice("green_kuka_left_gripper_motor")
    motor2 = robot.getDevice("green_kuka_right_gripper_motor")
    sensor1 = robot.getDevice("green_kuka_right_gripper_sensor")
    sensor2 = robot.getDevice("green_kuka_left_gripper_sensor")
    while robot.step(timestep) != -1:
                motor1.setPosition(distance1)
                motor2.setPosition(distance2)
                sensor1.enable(timestep)
                sensor2.enable(timestep)
                motor1_position = sensor1.getValue()
              
                if (abs(motor1_position) < abs(distance1) + 0.01) and (abs(motor1_position) > abs(distance1) - 0.01):
                        break

#classes for prediciting the motion planning
class DisplayPreviewManager:
    def __init__(self):
        self.cur_trajectory_idx = 0
        self.trajectory_list = []
        self.trajectory_start_time_stamp = 0
        self.points_idx = 0

    def append_new_trajectory(self,msg):
        self.trajectory_list.append(msg)
        # print(msg)
    def update_motor(self,motor_value):
       
      
        if self.cur_trajectory_idx >= len(self.trajectory_list):
            return

        cur_trajectories = self.trajectory_list[self.cur_trajectory_idx].trajectory
        
        for cur_trajectory in cur_trajectories:
           
            cur_t = cur_trajectory.joint_trajectory
           
            point_length = len(cur_t.points)
            cur_t
            if point_length <=  self.points_idx:
                print("This got passed")
                self.points_idx = 0
                self.cur_trajectory_idx += 1
              
                return
            if self.points_idx == 0:
                self.trajectory_start_time_stamp = rospy.Time().now().to_nsec()
                print("self.trajectory_start_time_stamp")
                print(self.trajectory_start_time_stamp)
            if cur_t.points[self.points_idx].time_from_start.to_nsec() + self.trajectory_start_time_stamp <= rospy.Time().now().to_nsec():
                
                for idx in range(len(cur_t.joint_names)):
                    motor_value[cur_t.joint_names[idx]] = cur_t.points[self.points_idx].positions[idx]
                    
                   
                self.points_idx += 1            
        
manager = DisplayPreviewManager()

def planned_pathes(msg):
    manager.append_new_trajectory(msg)
    joint_state(msg.trajectory_start.joint_state)
    print("planned_pathes")
    # robot.setLabel(1,"Robot_in_Progress",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
    pass
# wb_supervisor_world_save
# wb_supervisor_world_reload


#To show the robot execution
def execute_callback(msg):
    global execution_signal
    print("------------------worldSave------------------")
    file_name = "../../worlds/" + "execute.wbt"
    print(file_name)
    # robot.worldSave(file_name)
    i = 1
    execution_signal = True
    while (i <= 700):
        robot.setLabel(1,"Robot_during_execution",0.01,0.05,0.15,0x00FF00,0,"Times New Roman")
        i+=1
    sleep(1)
    print("------------------done------------------")
    pass

def callback(msg):
    if msg.data:
        print("lock")
        connector_iiwa1.lock()
    else:
        print("unlock")
        connector_iiwa1.unlock()

#for gripper callback 
def gripper_callback(data):
    print("Gripper Moved")
    global gripper_position
    robot_name = data.robot_name
    # grip = robot_comms[robot_name]
    position= data.position_state
    gripper_position = position
    force = data.force_state
    speed = data.speed_state
    fullGrasp = data.justGrasp
    print("Current_states_of_the_gripper is:")
    print(position)
    if fullGrasp:
        # grip.justGrasp()
        gripper_motor_values[robot_name] = 0.3
        iiwa2_gripper (-0.5,0.5)
        # pass
    else:
        gripper_motor_values[robot_name] = position / 255 * (-0.04)
        if position >= 20:
            if robot_name == "gkuka":
                connector_iiwa2.lock()
                print("green Kuka connector engaged")
                # iiwa2_gripper (-position / 255 * (-0.04),position / 255 * (-0.04))
            elif robot_name == "bkuka":
                connector_iiwa1.lock()
                print("blue Kuka connector engaged")
            
            elif robot_name == "s5":
                connector_s5.lock()
                print("S5 connector engaged")
        else:
            if robot_name == "gkuka":
                connector_iiwa2.unlock()
                # iiwa2_gripper (-position / 255 * (-0.04),position / 255 * (-0.04))
                print("green Kuka connector disengaged")
            elif robot_name == "bkuka":
                connector_iiwa1.unlock()
                print("blue Kuka connector disengaged")

            elif robot_name == "s5":
                connector_s5.unlock()
        pass
        # grip.movePos(position, force, speed)

def gripper_State_callback(msg):
    print("Gripper_state_accepted")

def c3_gripper_callback(msg):
    print("C3 gripper got your call")
    activiation = msg.data
    if (activiation == True):
        connector_c3.lock()
        print ("C3_Connector_Engaged")
    else:   
        connector_c3.unlock()
        print ("C3_Connector_Disengaged")

#Digital Emergency Stop 
def estop_callback(msg):
    global stop_signal
    global last_msg
    global first_time_stopping
    global first_time_resuming
    estop_signal = msg.data
    if (last_msg != msg):
        last_msg = msg
        if (estop_signal == "stop"):
            stop_signal = "stop"
            print("e_stop_engaged")
            first_time_stopping = True
            first_time_resuming = False
            robot.setLabel(1,"Emergency_E-Stop_engaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
            
        if (estop_signal == "resume"):
            stop_signal = "resume"
            print("e_stop_disengaged")
            first_time_stopping = False
            first_time_resuming = True
            robot.setLabel(1,"Emergency_E-Stop_disengaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
            
#for updating the gripper states 
def updateGripperState(robot_name, position, speed, force, activated, pub):
    robot_name = robot_name
    position = position
    force = force
    speed = speed
    activated = activated

    #gets message structure
    pub_msg = grip_state()

    #sets up the message
    pub_msg.gripper_name = str(robot_name)
    pub_msg.position_state = int(position)
    pub_msg.force_state = int(force)
    pub_msg.speed_state = int(speed)
    pub_msg.activated = bool(activated)
    #publish the message
    pub.publish(pub_msg)

#contingency call back for multiple contingency plans generated from the current cell feedback
def contingency_callback(msg):
    global continegency_status
    global contingency_last_msg
    global keyboard_checker 
    continegency = msg.data
    if (contingency_last_msg != msg):
        contingency_last_msg = msg
        if (continegency == "kuka_failed"):
            continegency_status = "kuka failed"
            keyboard_checker = "Insertion_Failed"
        if (continegency == "human_entered"):
            continegency_status = "human_entered"
            keyboard_checker = "human_entered_zone"
        if (continegency == "no_motion_plan_found"):
            continegency_status = "no_motion_plan_found"
            keyboard_checker = "no_motion_planned_found"
        if (continegency == "wrong_part_identification"):
            continegency_status = "wrong_part_identification"
            keyboard_checker = "wrong_part_detected"


#for debugging the features
def keyboard_check():
    global keyboard_checker
    current_key=keyboard_input.getKey()
    if (current_key==keyboard_input.DOWN):
            keyboard_checker = "True"
            print("displaying planned pathes")
            
    elif (current_key==keyboard_input.UP):
            keyboard_checker = "False"
            print("displaying current joint states")
    elif (current_key == keyboard_input.LEFT):
            keyboard_checker = "wrong_part_detected"
            print("wrong_part_detected")
    elif (current_key == keyboard_input.RIGHT):
            keyboard_checker = "no_motion_planned_found"
            print("no_motion_planned_found")
    elif (current_key == keyboard_input.HOME):
            keyboard_checker = "Insertion_Failed"
            print("Insertion_Failed")
    elif (current_key == keyboard_input.END):
            keyboard_checker = "human_entered_zone"
            print("human_entered_zone")
    elif (current_key == keyboard_input.PAGEUP):
            keyboard_checker = "human_is_gone"
            print("human_is_gone")
            human_off()

#setting up ROS subscribers for multiple topics
sub_gripper = rospy.Subscriber('gripper_test_topic',Bool,callback)
sub_c3_gripper = rospy.Subscriber("/toggle_pneumatic", Bool,c3_gripper_callback)
pub_joint_states = rospy.Subscriber('/joint_states',JointState,joint_state)
pub_state = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, planned_pathes)
pub_executed_state = rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, execute_callback)
# sub_gripper_change_states = rospy.Subscriber("moveGripper", change_state, gripper_callback) 
# sub_gripper_with_gripper_states = rospy.Subscriber("/gripper_state", change_state, gripper_State_callback)
sub_contingency_planning = rospy.Subscriber('/contingency_planning', String, contingency_callback)
# sub_digital_estop = rospy.Subscriber("/stop",String,estop_callback)
human_pub = rospy.Publisher("/human_operation_que",Bool,queue_size=200)


#indicate to show up human 
def human_on():
            msg = Bool()
            msg.data = True
            human_pub.publish(msg)
            
#indicate to disappear human    
def human_off():
            msg = Bool()
            msg.data = False
            human_pub.publish(msg)

#setting up keys for debugging 
keyboard_input.enable(timestep)
print("-------Press up for current joint states and down for planned-------")
first_time_stopping = True
first_time_resuming = True
first_time_human = True
human_entered_parameter = False

#setting up cases of illustration for different contingency plans 
def wrong_part_detected ():
    robot.setLabel(1,"""Wrong Part Detected
Human Assistance Required""",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
def no_motion_plan_found ():
    robot.setLabel(1,"""No Motion Plan Found
Human Assistance Required""",0.01,0.05,0.15,0xff0080,0,"Times New Roman")
def motion_failed():
    robot.setLabel(1,"Motion Plan Failed, Human_Assistance_Required",0.01,0.05,0.15,0xcc0000,0,"Times New Roman")
def insertion_failed():
    robot.setLabel(1,"""KUKA Sequence Failed
Human Assitance Required""",0.01,0.05,0.15,0xff3333,0,"Times New Roman")
def human_entered():
    human_on()
    global human_entered_parameter
    human_entered_parameter = True
    robot.setLabel(1,"""Human Entered 
Non-Collaborative Zone
Cell Shut Down""",0.01,0.05,0.15,0xff7800,0,"Times New Roman")

#initalizing the inital parameters
g = 1
f = 1
h = 1
stop_signal = ""
last_msg = ""
contingency_last_msg = ""
continegency_status = ""
gripper_position = 0
execution_signal = False
keyboard_checker = "False"

while robot.step(timestep) != -1:
    # setLabel(self, id, label, xpos, ypos, size, color, transparency, font="Arial"):

    if (human_entered_parameter == False) and (first_time_human == True):
        human_off()
        first_time_human = False
    if (stop_signal == "stop") and (first_time_stopping == True) and (human_entered_parameter == False) and ((keyboard_checker == "True" ) or (keyboard_checker == "False")):
        
        robot.setLabel(1,"Emergency_E-Stop_engaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
     
        keyboard_checker = "False"
    if (stop_signal == "resume") and (first_time_resuming == True) and (human_entered_parameter == False):
        
        
        first_time_stop = True
        robot.setLabel(1,"Emergency_E-Stop_disengaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
        f +=1
        if (f > 100):
                first_time_resuming = False
                keyboard_checker = "True"
                f = 1
                h = 1 
        
        
    current_key=keyboard_check()
    if gripper_position >= 190:
        iiwa2_gripper(-0.05,0.05)
    if gripper_position <= 180:
        iiwa2_gripper(0,0)
    if (keyboard_checker=="True"):
        if (execution_signal == True):
            robot.setLabel(1,"Robot_during_execution",0.01,0.05,0.15,0x00FF00,0,"Times New Roman")
            g +=1
            if (stop_signal == "stop"):
                execution_signal = False
                g = 1 
            if (g > 500):
                execution_signal = False
                g = 1 
        else:
            robot.setLabel(1,"Robot_motion_planning",0.01,0.05,0.15,0x00994c,0,"Times New Roman")
        manager.update_motor(motor_value)

        for name in motor_names:
            motor_map[name].setPosition(motor_value[name])
        pass
    if (keyboard_checker=="False"):
        if (execution_signal == True):
            robot.setLabel(1,"Robot_during_execution",0.01,0.05,0.15,0x00FF00,0,"Times New Roman")
            g +=1
            if (stop_signal == "stop"):
                execution_signal = False
                g = 1 
            if (g > 100):
                execution_signal = False
                g = 1 
        else:
            if (stop_signal == "stop"):
                pass
            else :
                robot.setLabel(1,"Robot_current_position",0.01,0.05,0.15,0x00994c,0,"Times New Roman")
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        
    if (keyboard_checker == "wrong_part_detected"):
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        connector_iiwa2.lock()
        wrong_part_detected()
    if (keyboard_checker == "no_motion_planned_found"):
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        no_motion_plan_found()
    if (keyboard_checker == "Insertion_Failed"):
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        insertion_failed()
    if (keyboard_checker == "human_entered_zone"):
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        human_entered()

        pass