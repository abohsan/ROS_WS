#include "Robot.h"
#include "Arduino.h"


Robot::Robot(Motor m11, Motor m22) {
  motor_1 = m11;
  motor_2 = m22;
}

void Robot::movee(int pwms1,int pwms2) {
//  Serial.println("Movee");
  motor_1.move(pwms1);
  motor_2.move(pwms2);
}
void Robot::setDir(bool b_1, bool f_1,bool b_2, bool f_2) {
  Serial.println("Halt");
  motor_1.Direction(b_1,f_1);
  motor_2.Direction(b_2,f_2);
}
void Robot::halt() {
  Serial.println("Halt");
  motor_1.move(0);
  motor_2.move(0);
}
