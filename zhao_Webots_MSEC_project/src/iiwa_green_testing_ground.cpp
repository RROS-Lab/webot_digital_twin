#include "ros/ros.h"
#include <std_msgs/String.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>

ros::ServiceClient motor1;
webots_ros::set_int timesteps;
webots_ros::set_float motor1_position;
static int controllerCount;
static std::vector<std::string> controllerList;

void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

int main(int argc, char **argv) {
 std::string controllerName, motorName, rangeFinderName;
 std::vector<std::string> deviceList;
 ros::init(argc, argv, "iiwa7", ros::init_options::AnonymousName);
 ros::NodeHandle n;
 ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
 ros::AsyncSpinner spinner(1);
 spinner.start();
 ros::ServiceClient motorSetPositionClient =
    n.serviceClient<webots_ros::set_float>(controllerName + '/' + motorName + "/set_position");
 std::cout <<"I am here"<<std::endl;
 
 motor1_position.request.value=3;
 motor1.call(motor1_position);
 std::cout<<"The Motor_I_requested_is:"<<std::endl;
 std::cout<<motor1_position.request<<std::endl;
 std::cout<<motor1<<std::endl;
}