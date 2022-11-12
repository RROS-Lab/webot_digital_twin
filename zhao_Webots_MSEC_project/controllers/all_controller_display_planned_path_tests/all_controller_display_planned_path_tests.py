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
gripper_moter_map = {
    "bkuka":["blue_kuka_left_gripper_motor", "blue_kuka_right_gripper_motor"],
    "gkuka":["green_kuka_left_gripper_motor", "green_kuka_right_gripper_motor"],
    "s5":[]
}

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
current_joint_states_motor_value = {}
gripper_states = {}
gripper_motor = {}
gripper_motor_values = {}
connector_map = {}

planned_motor_values = {}
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

def joint_state(msg):
    for i in range(len(msg.name)):
        if msg.name[i] in motor_value:
            current_joint_states_motor_value[msg.name[i]] = msg.position[i]

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
    pass
# wb_supervisor_world_save
# wb_supervisor_world_reload

def execute_callback(msg):
    print("------------------worldSave------------------")
    file_name = "../../worlds/" + "execute.wbt"
    print(file_name)
    # robot.worldSave(file_name)
    print("------------------done------------------")
    pass


def callback(msg):
    if msg.data:
        print("lock")
        connector_iiwa1.lock()
    else:
        print("unlock")
        connector_iiwa1.unlock()
def gripper_callback(data):
    print("Gripper Moved")
    robot_name = data.robot_name
    # grip = robot_comms[robot_name]
    position= data.position_state
    force = data.force_state
    speed = data.speed_state
    fullGrasp = data.justGrasp
    print("Current_states_of_the_gripper is:")
    print(position)
    if fullGrasp:
        # grip.justGrasp()
        gripper_motor_values[robot_name] = 0.3
        # pass
    else:
        gripper_motor_values[robot_name] = position / 255 * (-0.04)
        if position >= 90:
            if robot_name == "gkuka":
                connector_iiwa2.lock()
                print("green Kuka connector engaged")

            elif robot_name == "bkuka":
                connector_iiwa1.lock()
                print("blue Kuka connector engaged")
            
            elif robot_name == "s5":
                connector_s5.lock()
                print("S5 connector engaged")
        else:
            if robot_name == "gkuka":
                connector_iiwa1.unlock()
                print("green Kuka connector disengaged")
            elif robot_name == "bkuka":
                connector_iiwa1.unlock()
                print("blue Kuka connector disengaged")

            elif robot_name == "s5":
                connector_s5.unlock()
        pass
        # grip.movePos(position, force, speed)
def c3_gripper_callback(msg):
    activiation = msg.data
    if (activiation == True):
        connector_c3.lock()
    else:   
        connector_c3.unlock()

# def listener():
#     rospy.Subscriber("moveGripper", change_state, gripper_callback) 
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

keyboard_checker = True
def keyboard_check():
    global keyboard_checker
    current_key=keyboard_input.getKey()
    if (current_key==keyboard_input.DOWN):
            keyboard_checker = True
            print("displaying planned pathes")
            
    elif (current_key==keyboard_input.UP):
            keyboard_checker = False
            print("displaying current joint states")
sub_gripper = rospy.Subscriber('gripper_test_topic',Bool,callback)
sub_c3_gripper = rospy.Subscriber("/toggle_pneumatic", Bool,c3_gripper_callback)
pub_joint_states = rospy.Subscriber('/joint_states',JointState,joint_state)
pub_state = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, planned_pathes)
pub_executed_state = rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, execute_callback)
sub_gripper_change_states = rospy.Subscriber("moveGripper", change_state, gripper_callback) 

keyboard_input.enable(timestep)
print("-------Press up for current joint states and down for planned-------")



while robot.step(timestep) != -1:
    
    current_key=keyboard_check()
    
    if (keyboard_checker==True):
        manager.update_motor(motor_value)
    
        for name in motor_names:
            motor_map[name].setPosition(motor_value[name])
        pass
    if (keyboard_checker==False):
        for name in motor_names:
            motor_map[name].setPosition(current_joint_states_motor_value[name])
        pass