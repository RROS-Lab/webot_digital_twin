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
from robotiq_gripper.msg import change_state
from robotiq_gripper.msg import grip_state



# from robotiq_gripper.msg import grip_state

# create the Robot instance.
robot = Supervisor()
keyboard_input = Keyboard()
rospy.init_node("all_robot_webots")
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor_names = [
    "joint_1",       "joint_2",       "joint_3",       "joint_4",
    "joint_5",       "joint_6",       "c3_joint_1",    "c3_joint_2",
    "c3_joint_3",    "c3_joint_4",    "c3_joint_5",    "c3_joint_6",
    "s5_joint_1",    "s5_joint_2",    "s5_joint_3",    "s5_joint_4",
    "s5_joint_5",    "s5_joint_6",    "iiwa1_joint_1", "iiwa1_joint_2",
    "iiwa1_joint_3", "iiwa1_joint_4", "iiwa1_joint_5", "iiwa1_joint_6",
    "iiwa1_joint_7", "iiwa2_joint_1", "iiwa2_joint_2", "iiwa2_joint_3",
    "iiwa2_joint_4", "iiwa2_joint_5", "iiwa2_joint_6", "iiwa2_joint_7"]
gripper_name = ["bkuka","gkuka","s5"]
# linear_motor_name = ["blue_kuka_right_gripper_motor","blue_kuka_left_gripper_motor","green_kuka_right_gripper_motor","green_kuka_left_gripper_motor"]
#configuration of Gripper motors for bkuka + gkuka
gripper_moter_map = {
    "bkuka":["blue_kuka_left_gripper_motor", "blue_kuka_right_gripper_motor"],
    "gkuka":["green_kuka_left_gripper_motor", "green_kuka_right_gripper_motor"],
    "s5":[]
}

#setting up the connector on Epson and IIWA1 + 2 
connector_iiwa1 = Connector("connector_iiwa1")
connector_iiwa1.enablePresence(timestep)
connector_iiwa2 = Connector("connector_iiwa2")
connector_iiwa2.enablePresence(timestep)
connector_c3 = Connector("connector_c3")
connector_c3.enablePresence(timestep)
connector_s5 = Connector("connector_s5")
connector_s5.enablePresence(timestep)

#generationg dictionary for the motor value and joint value 
motor_map = {} #setting devices for the motor
motor_value = {} #for updating the motor value
current_joint_states_motor_value = {} #for getting the current state of the motor
gripper_states = {} #for getting the gripper states (disabled for this package but could be enabled for case study)
gripper_motor = {} #configuring getting gripper motors 
gripper_motor_values = {} #for getting the motor values to be executed 
connector_map = {} #disabled for the documents, but it could be used for case study
planned_motor_values = {} #disabled but could be use for case study

#setting up the motors only run once
for name in motor_names:
    motor_map[name] = robot.getDevice(name)
    motor_map[name].setVelocity(1.0)
    motor_value[name] = 0
    current_joint_states_motor_value[name] = 0

#setting up the gripper states, disabled but could be used in case study
for name in gripper_name:
    gripper_motor[name] = []
    gripper_motor_values[name] = []
    for motor_name in gripper_moter_map[name]:
        device = robot.getDevice(motor_name)
        device.setVelocity(1.0)
        gripper_motor[name].append(device)
        gripper_motor_values[name].append(0)

#call back function for current joint states
def joint_state(msg):
    for i in range(len(msg.name)):
        if msg.name[i] in motor_value:
            current_joint_states_motor_value[msg.name[i]] = msg.position[i]

#setting distance of iiwa2_gripper for insertion tasks
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
                # print("motor1_position")
                # print(motor1_position)
                # print("distance1")
                # print(distance1)
                if (abs(motor1_position) < abs(distance1) + 0.01) and (abs(motor1_position) > abs(distance1) - 0.01):
                        break

#setting up a class for getting planned pathes of the current planned path 
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
           

            

manager = DisplayPreviewManager()#setting up manager
          
        


def planned_pathes(msg):
    manager.append_new_trajectory(msg)
    joint_state(msg.trajectory_start.joint_state)
    print("planned_pathes")
    # robot.setLabel(1,"Robot_in_Progress",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
    pass
# wb_supervisor_world_save
# wb_supervisor_world_reload
execution_signal = False
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

#initalling testing the connector function, disabled 
def callback(msg):
    if msg.data:
        print("lock")
        connector_iiwa1.lock()
    else:
        print("unlock")
        connector_iiwa1.unlock()

#getting the motor current states and activate the connector in response to cases
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
    #full grasp for gripper to reach maximum position
    if fullGrasp:
        # grip.justGrasp()
        gripper_motor_values[robot_name] = 0.3
        iiwa2_gripper (-0.5,0.5)
        # pass
    else:
        gripper_motor_values[robot_name] = position / 255 * (-0.04)
        if position >= 200: #setting threeshold for holding the gripper (need to be tune to current setup of the gripper)
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
#debugging for if gripper state is received
def gripper_State_callback(msg):
    print("Gripper_state_accepted")

#c3 connector call back for vacuum gripper and connector activation
def c3_gripper_callback(msg):
    activiation = msg.data
    if (activiation == True):
        connector_c3.lock()
    else:   
        connector_c3.unlock()


#stopping the robot when emergency stop is engaged 
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
            robot.setLabel(1,"Emergency_E-Stop_disengaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")# this can be disable as it shows a split secs 
            


#inital planned for publish the joint state of the gripper to check if its consistant with hardware, Disable for this study
# def updateGripperState(robot_name, position, speed, force, activated, pub):
#     robot_name = robot_name
#     position = position
#     force = force
#     speed = speed
#     activated = activated

#     #gets message structure
#     pub_msg = grip_state()

#     #sets up the message
#     pub_msg.gripper_name = str(robot_name)
#     pub_msg.position_state = int(position)
#     pub_msg.force_state = int(force)
#     pub_msg.speed_state = int(speed)
#     pub_msg.activated = bool(activated)
#     #publish the message
#     pub.publish(pub_msg)

#keyboard switching from current joint states of the robot and the planning path of the robots
def keyboard_check():
    global keyboard_checker
    current_key=keyboard_input.getKey()
    if (current_key==keyboard_input.DOWN):
            keyboard_checker = True
            print("displaying planned pathes")
            
    elif (current_key==keyboard_input.UP):
            keyboard_checker = False
            print("displaying current joint states")

#setting up subscribers 
sub_gripper = rospy.Subscriber('gripper_test_topic',Bool,callback)
sub_c3_gripper = rospy.Subscriber("/toggle_pneumatic", Bool,c3_gripper_callback)
pub_joint_states = rospy.Subscriber('/joint_states',JointState,joint_state)
pub_state = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, planned_pathes)
pub_executed_state = rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, execute_callback)
sub_gripper_change_states = rospy.Subscriber("moveGripper", change_state, gripper_callback) 
sub_gripper_with_gripper_states = rospy.Subscriber("/gripper_state", change_state, gripper_State_callback)
sub_digital_estop = rospy.Subscriber("/stop",String,estop_callback)

keyboard_input.enable(timestep)

print("-------Press up for current joint states and down for planned-------")

#initalizing the global varible inital status before the while loop
keyboard_checker = True
first_time_stopping = True
first_time_resuming = True
g = 1
f = 1
h = 1
gripper_position = 0
stop_signal = ""
last_msg = ""

while robot.step(timestep) != -1:
    # setLabel(self, id, label, xpos, ypos, size, color, transparency, font="Arial"):
    
    #stop function
    if (stop_signal == "stop") and (first_time_stopping == True):
        
        
        
        robot.setLabel(1,"Emergency_E-Stop_engaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
     
        keyboard_checker = False
    #resume function
    if (stop_signal == "resume") and (first_time_resuming == True):
        
        
        first_time_stop = True
        robot.setLabel(1,"Emergency_E-Stop_disengaged",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
        f +=1
        if (f > 100):
                first_time_resuming = False
                keyboard_checker = True
                f = 1
                h = 1 
        
        
    current_key=keyboard_check()

    #IIWA2 gripper closing movement for insertion 
    if gripper_position >= 190:
        iiwa2_gripper(-0.05,0.05)

    #keyboard function to toggle between motion planning and current joint states
    if (keyboard_checker==True):
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
            robot.setLabel(1,"Robot_motion_planning",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
        
        manager.update_motor(motor_value)
        #set position for motion planning 
        for name in motor_names:
            motor_map[name].setPosition(motor_value[name])
        pass
    if (keyboard_checker==False):
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
                robot.setLabel(1,"Robot_current_position",0.01,0.05,0.15,0xff0000,0,"Times New Roman")
        
        #set position for current joint states
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        pass