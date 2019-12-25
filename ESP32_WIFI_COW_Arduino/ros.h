#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "Arduino.h"
#include "Robot.h"
#include "pub.h"


#define PWMRANGE 254
#define PWM_MIN 0

float mapPwm(float x, float out_min, float out_max);
void stopRobot();
void onTwist(const geometry_msgs::Twist &msg);
bool _connected = false;

ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);

const uint8_t R_PWM = 25;
const uint8_t R_BACK = 16;
const uint8_t R_FORW = 4;

const uint8_t L_BACK = 2;
const uint8_t L_FORW = 0;
const uint8_t L_PWM = 26;

int freq = 10000;
Motor myMotorR( R_PWM, R_BACK, R_FORW, 1, 0 , freq, 8);
Motor myMotorL( L_PWM, L_BACK, L_FORW, 1, 1 , freq, 8 );

Robot myRobot(myMotorL, myMotorR);

void onTwist(const geometry_msgs::Twist &msg) {
  if (!_connected)
  {
    stopRobot();
    return;
  }

  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);
  myRobot.setDir(l < 0, l > 0, r < 0, r > 0 );
  myRobot.movee(rPwm, lPwm );
}

void ros_setup(IPAddress server) {
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

bool rosConnected()
{
  bool connected = node.connected();
  if (_connected != connected)
  {
    _connected = connected;
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  return connected;
}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void stopRobot()
{
  myRobot.setDir(0, 0, 0, 0);
  myRobot.movee(0, 0);
}
