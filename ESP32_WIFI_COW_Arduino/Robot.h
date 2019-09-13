#ifndef ROBOT_H
#define ROBOT_H
//#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include "Motor.h"
#define PWM_MIN 0
#define PWMRANGE 255
class Robot {
  public:
    Robot(Motor m1, Motor m2);
    void movee(int,int);
    void halt();
  private:
    Motor motor_1;
    Motor motor_2;
};

#endif
