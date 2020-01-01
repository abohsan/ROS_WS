
#ifndef JAGUAR_NODES_HPP
#define JAGUAR_NODES_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

#include <jaguar/GPSInfo.h>
#include <jaguar/MotorInfoArray.h>
#include <jaguar/MotorBoardInfoArray.h>
#include <jaguar/FlipMotor.h>
#include <jaguar/IMUInfo.h>

#include "Jaguar.hpp"
#include "variables.hpp"

namespace jaguar_ns
{

class Nodes
{
public:
   Nodes(const ros::NodeHandle &node_handle,
         const ros::NodeHandle &private_node_handle);
   ~Nodes() = default;
   /**
     *  Initialize the publisher, subscribers, timers
     *  and parameters from the yaml files
     */
   void init();
   /**
     * Subscriber callback function
     * @param msg
     */
   void keyboard_Callback(const std_msgs::String::ConstPtr &msg);
   void moveWheels_callback(const geometry_msgs::Twist::ConstPtr &msg);
   void flipers_sub_Callback(const jaguar::FlipMotor::ConstPtr &msg);
   /**
     * Timer callback function
     * @param event
     */
   void ping_timer_callback(const ros::TimerEvent &event);
   void pub_timer_callback(const ros::TimerEvent &event);

   void publisherIMUData(IMUData imuData);
   void publisherGPSInfo(GPSData gpsData);
   void publisherMotorData(MotorData motorData[], int len);
   void publisherMotorBoardInfoArray(MotorBoardData motorBoardData[], int len);

   Jaguar *jaguar;
   int conn;
   bool isConnected;
   IMUData imuData;
   GPSData gpsData;

   MotorData flipArmMotor[4];
   MotorData motorData[8];
   MotorBoardData motorBoardData[4];

private:
   // public ros node handle
   ros::NodeHandle nh_;
   // private ros node handle
   ros::NodeHandle pnh_;
   std::string node_name_{"nodes"};
   // timer
   //  ros::Timer periodic_timer_;
   ros::Timer ping_timer;
   ros::Timer pub_timer;

   // moveWheels variable start;
   int x;
   int z;
   int cmdValue1;
   int cmdValue2;

   // moveWheels variable end;

   // subscriber and publisher
   ros::Subscriber keyboard_sub;
   ros::Subscriber cmd_vl_Jaguar;
   ros::Subscriber flipers_sub;
   // publisher
   // ros::Publisher wheelStatus;

   ros::Publisher imuInfo_pub_;
   ros::Publisher gpsInfo_pub_;
   ros::Publisher motorInfo_pub_;
   ros::Publisher motorboardInfoArray_pub_;

   unsigned int sub1_callback_count_;
   unsigned int sub2_callback_count_;
   unsigned int sub3_callback_count_;
   unsigned int pub_periodic_count_;

   void processRobotData();
   void dealWithPackage(std::string, int);

   void getFrontFlipAngle();
   void getRearFlipAngle();
   double ad2Temperature(int value);

}; // class Nodes

} // namespace jaguar_ns

#endif // JAGAUR_NODES_HPP
