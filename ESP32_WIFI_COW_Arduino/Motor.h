#ifndef MOTOR_H
#define MOTER_H

class Motor {
  public:
    Motor();
    Motor( int ,int,int, int , int , int, int );
    void move(int);
    void Direction(bool);
  private:
    int b_dir;
    int f_dir;
    int channel;
    int freq;
    int resolution;
};

#endif
