#include "ros/ros.h"
#include <vector>
#include <std_msgs/String.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/set_int.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>



ros::ServiceClient abb_motor1;
ros::ServiceClient abb_motor2;
ros::ServiceClient abb_motor3;
ros::ServiceClient abb_motor4;
ros::ServiceClient abb_motor5;
ros::ServiceClient abb_motor6;
ros::ServiceClient c3_motor1;
ros::ServiceClient c3_motor2;
ros::ServiceClient c3_motor3;
ros::ServiceClient c3_motor4;
ros::ServiceClient c3_motor5;
ros::ServiceClient c3_motor6;
ros::ServiceClient s5_motor1;
ros::ServiceClient s5_motor2;
ros::ServiceClient s5_motor3;
ros::ServiceClient s5_motor4;
ros::ServiceClient s5_motor5;
ros::ServiceClient s5_motor6;
ros::ServiceClient iiwa_blue_motor1;
ros::ServiceClient iiwa_blue_motor2;
ros::ServiceClient iiwa_blue_motor3;
ros::ServiceClient iiwa_blue_motor4;
ros::ServiceClient iiwa_blue_motor5;
ros::ServiceClient iiwa_blue_motor6;
ros::ServiceClient iiwa_blue_motor7;
ros::ServiceClient iiwa_green_motor1;
ros::ServiceClient iiwa_green_motor2;
ros::ServiceClient iiwa_green_motor3;
ros::ServiceClient iiwa_green_motor4;
ros::ServiceClient iiwa_green_motor5;
ros::ServiceClient iiwa_green_motor6;
ros::ServiceClient iiwa_green_motor7;



webots_ros::set_int timesteps;
webots_ros::set_float motor1_position;


using namespace std;

static int controllerCount;
static std::vector<std::string> controllerList;
std::vector<float> abb_joint_states;
std::vector<float> s5_joint_states;
std::vector<float> c3_joint_states;
std::vector<float> iiwa_blue_joint_states;
std::vector<float> iiwa_green_joint_states;

float abb_motor_joint_states_1;
float abb_motor_joint_states_2;
float abb_motor_joint_states_3;
float abb_motor_joint_states_4;
float abb_motor_joint_states_5;
float abb_motor_joint_states_6;







void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
  ROS_INFO("The Controller list is:", controllerList.data());
  std::cout<<"Controller_Count_is: "<<controllerCount<<std::endl;
}

void joint_states_call_back(const sensor_msgs::JointState &joint_states_array){
   int i = 1;
  //  abb_joint_states.resize(6);
  //  s5_joint_states.resize(6);
  //  c3_joint_states.resize(6);
  //  iiwa_blue_joint_states.resize(6);
  //  iiwa_green_joint_states.resize(6);

  // while (i <= sizeof(joint_states_array.position))
  // {
  //  if (i <= 5)
  //  {
  //  abb_joint_states[i] = joint_states_array.position[i];
  //  i++;
  //  }
  //   if (i <= 11)
  // {
  //  s5_joint_states[i] = joint_states_array.position[i];
  //  i++;
  // }
  // if (i <= 17)
  // {
  //  c3_joint_states[i] = joint_states_array.position[i];
  //  i++;
  // }

  // if (i <= 24)
  // {
  //  iiwa_blue_joint_states[i] = joint_states_array.position[i];
  //  i++;
  // }
 
  // if (i <= 31)
  // {
  //  iiwa_green_joint_states[i] = joint_states_array.position[i];
  //  i++;
  // } 
  // }
 abb_motor_joint_states_1 = joint_states_array.position[0];
 abb_motor_joint_states_2 = joint_states_array.position[1];
 abb_motor_joint_states_3 = joint_states_array.position[2];
 abb_motor_joint_states_4 = joint_states_array.position[3];
 abb_motor_joint_states_5 = joint_states_array.position[4];
 abb_motor_joint_states_6 = joint_states_array.position[5];
   

}

int main(int argc, char **argv) {
 std::string controllerName, motorName, rangeFinderName;
 std::vector<std::string> deviceList;
 ros::init(argc, argv, "full_tests", ros::init_options::AnonymousName);
 ros::NodeHandle n;
 abb_motor1 = n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_1/set_position");
 abb_motor2 = n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_2/set_position");
 abb_motor3 = n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_3/set_position");
 abb_motor4= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_4/set_position");;
 abb_motor5= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_5/set_position");;
 abb_motor6= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/joint_6/set_position");;
 c3_motor1= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_1/set_position");;
 c3_motor2= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_2/set_position");;
 c3_motor3= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_3/set_position");;
 c3_motor4= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_4/set_position");;
 c3_motor5= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_5/set_position");;
 c3_motor6= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/c3_joint_6/set_position");;
 s5_motor1= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_1/set_position");;
 s5_motor2= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_2/set_position");;
 s5_motor3= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_3/set_position");;
 s5_motor4= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_4/set_position");;
 s5_motor5= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_5/set_position");;
 s5_motor6= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/s5_joint_6/set_position");;
 iiwa_blue_motor1= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_1/set_position");;
 iiwa_blue_motor2= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_2/set_position");;
 iiwa_blue_motor3= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_3/set_position");;
 iiwa_blue_motor4= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_4/set_position");;
 iiwa_blue_motor5= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_5/set_position");;
 iiwa_blue_motor6= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_6/set_position");;
 iiwa_blue_motor7= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa1_joint_7/set_position");;
 iiwa_green_motor1= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_1/set_position");;
 iiwa_green_motor2= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_2/set_position");;
 iiwa_green_motor3= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_3/set_position");;
 iiwa_green_motor4= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_4/set_position");;
 iiwa_green_motor5= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_5/set_position");;
 iiwa_green_motor6= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_6/set_position");;
 iiwa_green_motor7= n.serviceClient<webots_ros::set_float>("/full_robot_assemblies_unique_base/iiwa2_joint_7/set_position");;
 
 
 
 
 
 
 
 
 
 
 ros::Subscriber nameSub = n.subscribe("model_name", 1000, controllerNameCallback);
 
 while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
     ROS_INFO("help me i have no control");
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();
     ros::spinOnce();

  }
  ros::spinOnce();
 ros::AsyncSpinner spinner(1);
 spinner.start();
 
 std::cout <<"I am here"<<std::endl;
if (controllerCount == 1)
    controllerName = controllerList[0];
 std::cout<<"The Motor_I_requested_is:"<<std::endl;
 std::cout<<controllerName<<std::endl;
 ros::Subscriber newnameSub = n.subscribe("joint_states", 1000, joint_states_call_back);

 if (abb_joint_states.empty())
  {
    ROS_INFO("help me i have no joint states");
     ros::spinOnce();
  }
 std::cout<<"The abb_motor_joint_states_1 i request is:"<<std::endl;
 std::cout<<abb_motor_joint_states_1<<std::endl;

//  ros::ServiceClient deviceListClient =
//     n.serviceClient<webots_ros::robot_get_device_list>(controllerName + "/robot/get_device_list");
//   webots_ros::robot_get_device_list deviceListSrv;
//   ros::ServiceClient motorGetTargetPositionClient =
//     n.serviceClient<webots_ros::get_float>(controllerName + '/' + motorName + "/get_target_position");
//   webots_ros::get_float motorGetTargetPositionSrv;
//  if (deviceListClient.call(deviceListSrv))
//     deviceList = deviceListSrv.response.list;
//  ROS_INFO("Device_Lists",deviceListSrv.response.list);
 
//  ros::ServiceClient motorSetPositionClient =
//     n.serviceClient<webots_ros::set_float>(controllerName + '/' + motorName + "/set_position");
//   webots_ros::set_float motorSetPositionSrv;
//   motorSetPositionSrv.request.value = 0;
//   std::cout<<"The Motor_I_requested_again_is:"<<std::endl;
//   std::cout<<motorSetPositionClient<<std::endl;
//   while (ros::ok())
//   {
//     motorSetPositionSrv.request.value = 2;
//     motorSetPositionClient.call(motorSetPositionSrv);
//     motorGetTargetPositionClient.call(motorGetTargetPositionSrv);
//     ros::spinOnce();
//   }

    webots_ros::set_float abb_joint_1_position_msg;
    webots_ros::set_float abb_joint_2_position_msg;
    webots_ros::set_float abb_joint_3_position_msg;
    webots_ros::set_float abb_joint_4_position_msg;
    webots_ros::set_float abb_joint_5_position_msg;
    webots_ros::set_float abb_joint_6_position_msg;
    webots_ros::set_float c3_joint_1_position_msg;
    webots_ros::set_float c3_joint_2_position_msg;
    webots_ros::set_float c3_joint_3_position_msg;
    webots_ros::set_float c3_joint_4_position_msg;
    webots_ros::set_float c3_joint_5_position_msg;
    webots_ros::set_float c3_joint_6_position_msg;
    webots_ros::set_float s5_joint_1_position_msg;
    webots_ros::set_float s5_joint_2_position_msg;
    webots_ros::set_float s5_joint_3_position_msg;
    webots_ros::set_float s5_joint_4_position_msg;
    webots_ros::set_float s5_joint_5_position_msg;
    webots_ros::set_float s5_joint_6_position_msg;
    webots_ros::set_float iiwa_blue_joint_1_position_msg;
    webots_ros::set_float iiwa_blue_joint_2_position_msg;
    webots_ros::set_float iiwa_blue_joint_3_position_msg;
    webots_ros::set_float iiwa_blue_joint_4_position_msg;
    webots_ros::set_float iiwa_blue_joint_5_position_msg;
    webots_ros::set_float iiwa_blue_joint_6_position_msg;
    webots_ros::set_float iiwa_blue_joint_7_position_msg;
    webots_ros::set_float iiwa_green_joint_1_position_msg;
    webots_ros::set_float iiwa_green_joint_2_position_msg;
    webots_ros::set_float iiwa_green_joint_3_position_msg;
    webots_ros::set_float iiwa_green_joint_4_position_msg;
    webots_ros::set_float iiwa_green_joint_5_position_msg;
    webots_ros::set_float iiwa_green_joint_6_position_msg;
    webots_ros::set_float iiwa_green_joint_7_position_msg;
while (ros::ok())
  {
  
    
    ros::spinOnce();

    abb_joint_1_position_msg.request.value = abb_motor_joint_states_1;
    abb_motor1.call(abb_joint_1_position_msg);
    ROS_INFO("%d",abb_joint_1_position_msg.response.success);
    abb_joint_2_position_msg.request.value = abb_motor_joint_states_2;
    abb_motor2.call(abb_joint_2_position_msg);
    ROS_INFO("%d",abb_joint_2_position_msg.response.success);
    abb_joint_3_position_msg.request.value = abb_motor_joint_states_3;
    abb_motor3.call(abb_joint_3_position_msg);
    ROS_INFO("%d",abb_joint_3_position_msg.response.success);
    abb_joint_4_position_msg.request.value = abb_motor_joint_states_4;
    abb_motor4.call(abb_joint_4_position_msg);
    ROS_INFO("%d",abb_joint_2_position_msg.response.success);
    abb_joint_5_position_msg.request.value = abb_motor_joint_states_5;
    abb_motor5.call(abb_joint_5_position_msg);
    ROS_INFO("%d",abb_joint_5_position_msg.response.success);
    abb_joint_6_position_msg.request.value = abb_motor_joint_states_6;
    abb_motor6.call(abb_joint_6_position_msg);
    ROS_INFO("%d",abb_joint_6_position_msg.response.success);

    // c3_joint_1_position_msg.request.value = 0;
    // c3_motor1.call(c3_joint_1_position_msg);
    // ROS_INFO("%d",c3_joint_1_position_msg.response.success);
    // c3_joint_2_position_msg.request.value = -0.0;
    // c3_motor2.call(c3_joint_2_position_msg);
    // ROS_INFO("%d",c3_joint_2_position_msg.response.success);
    // c3_joint_3_position_msg.request.value = -0.0;
    // c3_motor3.call(c3_joint_3_position_msg);
    // ROS_INFO("%d",c3_joint_3_position_msg.response.success);
    // c3_joint_4_position_msg.request.value = 0.0;
    // c3_motor4.call(c3_joint_4_position_msg);
    // ROS_INFO("%d",c3_joint_4_position_msg.response.success);
    // c3_joint_5_position_msg.request.value = -0.0;
    // c3_motor5.call(c3_joint_5_position_msg);
    // ROS_INFO("%d",c3_joint_5_position_msg.response.success);
    // c3_joint_6_position_msg.request.value = 0;
    // c3_motor6.call(c3_joint_6_position_msg);
    // ROS_INFO("%d",c3_joint_6_position_msg.response.success);

    // s5_joint_1_position_msg.request.value = 0;
    // s5_motor1.call(s5_joint_1_position_msg);
    // ROS_INFO("%d",s5_joint_1_position_msg.response.success);
    // s5_joint_2_position_msg.request.value = 0;
    // s5_motor2.call(s5_joint_2_position_msg);
    // ROS_INFO("%d",s5_joint_2_position_msg.response.success);
    // s5_joint_3_position_msg.request.value = 0;
    // s5_motor3.call(s5_joint_3_position_msg);
    // ROS_INFO("%d",s5_joint_3_position_msg.response.success);
    // s5_joint_4_position_msg.request.value = 0;
    // s5_motor4.call(s5_joint_4_position_msg);
    // ROS_INFO("%d",s5_joint_4_position_msg.response.success);
    // s5_joint_5_position_msg.request.value = 0;
    // s5_motor5.call(s5_joint_5_position_msg);
    // ROS_INFO("%d",s5_joint_5_position_msg.response.success);
    // s5_joint_6_position_msg.request.value = 0;
    // s5_motor6.call(s5_joint_6_position_msg);
    // ROS_INFO("%d",s5_joint_6_position_msg.response.success);

    // iiwa_blue_joint_1_position_msg.request.value = 0;
    // iiwa_blue_motor1.call(iiwa_blue_joint_1_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_1_position_msg.response.success);
    // iiwa_blue_joint_2_position_msg.request.value = 0;
    // iiwa_blue_motor2.call(iiwa_blue_joint_2_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_2_position_msg.response.success);
    // iiwa_blue_joint_3_position_msg.request.value = 0;
    // iiwa_blue_motor3.call(iiwa_blue_joint_3_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_3_position_msg.response.success);
    // iiwa_blue_joint_4_position_msg.request.value = 0;
    // iiwa_blue_motor4.call(iiwa_blue_joint_4_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_4_position_msg.response.success);
    // iiwa_blue_joint_5_position_msg.request.value = 0;
    // iiwa_blue_motor5.call(iiwa_blue_joint_5_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_5_position_msg.response.success);
    // iiwa_blue_joint_6_position_msg.request.value = 0;
    // iiwa_blue_motor6.call(iiwa_blue_joint_6_position_msg);
    // ROS_INFO("%d",iiwa_blue_joint_6_position_msg.response.success);


    // iiwa_green_joint_1_position_msg.request.value = 0;
    // iiwa_green_motor1.call(iiwa_green_joint_1_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_1_position_msg.response.success);
    // iiwa_green_joint_2_position_msg.request.value = 0;
    // iiwa_green_motor2.call(iiwa_green_joint_2_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_2_position_msg.response.success);
    // iiwa_green_joint_3_position_msg.request.value = 0;
    // iiwa_green_motor3.call(iiwa_green_joint_3_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_3_position_msg.response.success);
    // iiwa_green_joint_4_position_msg.request.value = 0;
    // iiwa_green_motor4.call(iiwa_green_joint_4_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_4_position_msg.response.success);
    // iiwa_green_joint_5_position_msg.request.value = 0;
    // iiwa_green_motor5.call(iiwa_green_joint_5_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_5_position_msg.response.success);
    // iiwa_green_joint_6_position_msg.request.value = 0;
    // iiwa_green_motor6.call(iiwa_green_joint_6_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_6_position_msg.response.success);
    // iiwa_green_joint_7_position_msg.request.value = 0;
    // iiwa_green_motor7.call(iiwa_green_joint_7_position_msg);
    // ROS_INFO("%d",iiwa_green_joint_7_position_msg.response.success);

  
  }


}