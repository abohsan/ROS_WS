#ifndef ARM_H
#define ARM_H

#include "tcpSocket.hpp"
#include "variables.hpp"
#include "shareMethods.hpp"
#include "File.hpp"

// #include <QObject>
// #include <QtNetwork>
// #include <QTimer>
// #include "protocol.h"

class Arm {
public:
    Arm();
    ~Arm();
    //    // Controls
    void connectToRobot1();
private:
    bool m_motor1Connected;
    bool m_motor1Connected2;
    bool m_allArmMotorState;
    bool initial;

    struct MotorDriverData {
        double motAmp1;     // motor1 current value
        double motAmp2;     // motor2 current value
        int motPower1;      //motor1 output power -1000 ~ +1000
        int motPower2;      //motor2 output power -1000 ~ +1000
        int motEncP1;       //motor1 encoder position value
        int motEncP2;       //motor2 encoder position value
        int motEncS1;       //motor1 encoder velocity value
        int motEncS2;       //motor2 encoder velocity value
        double motTemp1;    //motor1 temperature value
        double motTemp2;    //motor2 temperature value
        double drvVoltage;  //driver board voltage, around 12V
        double batVoltage;  // main power battery voltage
        double reg5VVoltage;//driver broad 5V
        double ai3;         // A/D channel 3 raw A/D value, full range is 4095, will translate to motor 1temperature value
        double ai4;         // A/D channel 4 raw A/D value, full range is 4095, will translate to motor2 temperature value
        double intTemp;     // on driver baord temperature sensor reading
        double ch1Temp;     // channel 1 driver chip temperature
        double ch2Temp;     // channel 2 driver chip temperature
        int statusFlag ;     // motor driver baord status
        int mode1;          // channel 1 working mode, 0--open loop, 1 -- close speed control, 2,3 -position control, 4-torgue control
        int mode2;
    };

    struct JointMotorData {
        int pwmOutput;
        int encodeSpeed ;
        int encoderPos;
        int encoderDir ;
        double circleCnt;
        double resolution;
        bool protect;
        double angle;
        int iniPos;
        int limitPos1;
        int limitPos2;
        bool stuckFlag;
        int cmdDir;
        int preCmdDir;
        int manipulatorStuckCnt;
        int jointCmd;
        bool jointJoy;
        bool jointCtrl;
        double motorTemperature;
        double currentAmp;
    };

    JointMotorData jointMotorData[6];
    // QTcpSocket *m_tcpRobot1;
    // QTcpSocket *tcpRobot2;
    // QTcpSocket *tcpRobot3;
    // QTcpSocket *tcpRobot4;

    // QTimer m_pingTimer;
    // QString m_receivedData1;
    // QString m_receivedData2;
    // QString m_receivedData3;

    void dealWithPackage1( std::string received);
    void dealWithPackage2( std::string received);
    void dealWithPackage3( std::string received);

    MotorDriverData motorData1;
    MotorDriverData motorData2;
    MotorDriverData motorData3;

    double ad2Temperature(int value);
    int m_watchDogCnt1;
    int m_watchDogCnt2;
    double trans2Angle(int channel);
    double manipulatorPos[3];
    void getPositionXY();
    double tipAngle;
    bool armIniFlag;

    // Message form MainThread
    void executeCommand(const char &cmd);
    // end Message form MainThread
    void cmdSend(int channel,int cmdValue,int motorCtrl);
    void jointAngleCmdSend(int channel,double angle);

    void armPosCmdSend(double x, double y, double tipAngle);

    void connectToRobot2();
    void processRobotData1();
    void processRobotData2();
    void processRobotData3();
    void sendPing();
    void motor1Up();
    void motor1Down();
    void motor1Stop();

    void motor2Up();
    void motor2Down();
    void motor2Stop();

    void motor3Up();
    void motor3Down();
    void motor3Stop();

    void motor4Up();
    void motor4Down();
    void motor4Stop();

    void motor5Up();
    void motor5Down();
    void motor5Stop();

    void motor6Up();
    void motor6Down();
    void motor6Stop();

    void motorReleaseAll();
    void motorStopAll();
    void sendQuery1();
    void sendQuery2();
    void sendQuery3();
    void sendQuery4();
    void j1GoCmd();
    void j2GoCmd();
    void rotateGoCmd();
    void tipGoCmd();
    void panGoCmd();
    void armPosGoCmd();
    void armResetCmd();
    void armSetIniCmd();

};

#endif // ARM_H
