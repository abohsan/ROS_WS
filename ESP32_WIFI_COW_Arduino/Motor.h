#ifndef MOTOR_H
#define MOTER_H

class Motor {
  public:
    Motor();
    Motor( int , int , int , int, int );
    void move(int);
  private:
    int channel;
    int freq;
    int resolution;
};

#endif
