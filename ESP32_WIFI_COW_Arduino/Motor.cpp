#include "Motor.h"
#include "Arduino.h"
//
Motor::Motor() {} // Important to be hear. never delete it unless you know what you are doing.
Motor::Motor( int pwm1, int b, int f, int offset1, int theChannal, int theFreq, int theResolution) {

  b_dir = b;
  f_dir = f;
  channel = theChannal;
  freq = theFreq;
  resolution = theResolution;
  pinMode(pwm1, OUTPUT);
  pinMode(b_dir, OUTPUT);
  pinMode(f_dir, OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pwm1, channel);
}

void Motor::move(int pwms) {
  ledcWrite(channel, pwms);
}

void Motor::Direction( bool dir) {
    digitalWrite(f_dir,  dir);
    digitalWrite(b_dir, !dir);
}
