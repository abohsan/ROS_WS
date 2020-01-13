
#ifndef JAGUAR_NODES_HPP
#define JAGUAR_NODES_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

#include <arm/ArmMotorInfo.h>
#include <arm/ArmMotorInfoArray.h>
#include <arm/MotorCmd.h>
#include <arm/JointAngleCmd.h>
#include <arm/ArmPositionCmd.h>

#include <arm/JointAngle.h>
#include <arm/singleJoint.h>

#include <sensor_msgs/JointState.h>

#include "variables.hpp"
#include "Arm.hpp"

namespace arm_ns
{

class Nodes
{
public:
  Nodes(const ros::NodeHandle &node_handle,
        const ros::NodeHandle &private_node_handle);
  ~Nodes() = default;
  void init();

  void keyboard_Callback(const std_msgs::String::ConstPtr &msg);
  // void join_sub_Callback(const arm::JointAngle::ConstPtr &msg);
  void join_0_sub_Callback(const arm::singleJoint::ConstPtr &msg);
  void join_1_sub_Callback(const arm::singleJoint::ConstPtr &msg);
  void join_2_sub_Callback(const arm::singleJoint::ConstPtr &msg);
  void join_3_sub_Callback(const arm::singleJoint::ConstPtr &msg);
  void join_4_sub_Callback(const arm::singleJoint::ConstPtr &msg);
  void ping_timer_callback(const ros::TimerEvent &event);
  void pub_timer_callback(const ros::TimerEvent &event);
  void arm_publisher(int pos[], int vel[], int pwm[], double temp[], double angle[], int len);
  void publisherArmJointStatus();


    std::shared_ptr<Arm> arm;
  // private:

  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;
  std::string node_name_{"nodes"};
  // timer
  //  ros::Timer periodic_timer_;
  ros::Timer ping_timer;
  ros::Timer pub_timer;

  // subscriber and publisher
  ros::Subscriber keyboard_sub;
  // ros::Subscriber joint_sub;
  ros::Subscriber joint_0_sub;
  ros::Subscriber joint_1_sub;
  ros::Subscriber joint_2_sub;
  ros::Subscriber joint_3_sub;
  ros::Subscriber joint_4_sub;
  //  ros::Subscriber flipers_sub;
  // publisher

  ros::Publisher arm_motorInfo_pub_;
  ros::Publisher joint_pub;

  void arm_dealWithPackage1(std::string);
  void arm_processRobotData1();
  // void arm_processRobotData2();
  // void arm_processRobotData3();

}; // class Nodes

} // namespace arm_ns

#endif // JAGAUR_NODES_HPP
