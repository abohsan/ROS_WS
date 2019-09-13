#include "Motor.h"
#include "Arduino.h"
//
Motor::Motor() {} // Important to be hear. never delete it unless you know what you are doing.
Motor::Motor( int pwm1, int offset1, int theChannal, int theFreq, int theResolution) {

  channel = theChannal;
  freq = theFreq;
  resolution = theResolution;
  pinMode(pwm1, OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pwm1, channel);
}

void Motor::move(int pwms) {
  ledcWrite(channel, pwms);
}
