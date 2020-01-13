#ifndef ARM_H
#define ARM_H

#include "tcpSocket.hpp"
#include "variables.hpp"
#include "shareMethods.hpp"
#include "File.hpp"
#include <memory>
namespace arm_ns
{
class Arm {
public:
    Arm();
    ~Arm();
    //    // Controls
    void sendPings();

    void motorReleaseAll();
    void motor1Up();
    void motor1Down();
    void j0Go_to_degree(double targetAngle);
    void j1Go_to_degree(double targetAngle);
    void j2Go_to_degree(double targetAngle);
    void j3Go_to_degree(double targetAngle);
    void j4Go_to_degree(double targetAngle);
    void move_arm_degree(double joint_0,double joint_1,double joint_2,double joint_3,double joint_4,double joint_5);
   
    void j0Go_to_radians(double targetAngle);
    void j1Go_to_radians(double targetAngle);
    void j2Go_to_radians(double targetAngle);
    void j3Go_to_radians(double targetAngle);
    void j4Go_to_radians(double targetAngle);
    void move_arm_radians(double joint_0,double joint_1,double joint_2,double joint_3,double joint_4,double joint_5);

    void moveMotor_0(int PWM);
    void moveMotor_1(int PWM);
    void moveMotor_2(int PWM);
    void moveMotor_3(int PWM);
    void moveMotor_4(int PWM);
    void moveMotor_5(int PWM);

    // void moveMotor_5(int PWM);
    // void moveMotor_6(int PWM);
    
    //start used by Node class
    std::string read_ipTCP_1();
    std::string read_ipTCP_2();
    std::string read_ipTCP_3();
    double get_JOINT_INI_ANGLE(int i);
    int get_MOTOR_NUM();
    // End used by Node class
// private:
    std::string ip_1 ;
    std::string ip_2 ;
    int ip_1_port_1 ;
    int ip_1_port_2  ;
    int ip_2_port_1 ;
    int ip_2_port_2  ; // not used
    std::shared_ptr<tcpSocket> ipTCP_1;
    std::shared_ptr<tcpSocket> ipTCP_2;
    std::shared_ptr<tcpSocket> ipTCP_3;
    std::shared_ptr<tcpSocket> ipTCP_4;  // not used
    
    


    void connect();
    bool m_motor1Connected;
    bool m_motor1Connected2;
    bool m_allArmMotorState;
    bool pushButtonArmSetIni;

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
        double iangle;
        int iniPos;
        double limitPosStart; // Added By abdul
        double limitPosEnd;  // Added By abdul
        double limitPos1;
        double limitPos2;
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


 
    // std::string m_receivedData1;
    std::string m_receivedData2;
    std::string m_receivedData3;

    // void dealWithPackage1( std::string received);
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

    void cmdSend(int channel,int cmdValue,int motorCtrl);
    void jointAngleCmdSend(int channel,double angle);

    void armPosCmdSend(double x, double y, double tipAngle);
    // void processRobotData1();
    void processRobotData2();
    void processRobotData3();
    // void sendPing();
 

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

 
    void motorStopAll();
    void sendQuery1();
    void sendQuery2();
    void sendQuery3();
    void sendQuery4();

    double lineEditTipAngle ;


    double lineEditArmTargetX ;
    double lineEditArmTargetY ;

    double lineEditJ1TargetAngle;
    double lineEditJ2TargetAngle;
    double lineEditRotateTargetAngle;
    double lineEditPanTargetAngle;
    double lineEditTipTargetAngle;
    
    std::string lineEditChannel1State;
    std::string lineEditChannel2State;
    std::string lineEditChannel3State;



    
    void j1GoCmd();
    void j2GoCmd();
    void rotateGoCmd();
    void tipGoCmd();
    void panGoCmd();
    void armPosGoCmd();
    void armResetCmd();
    void armSetIniCmd();
    void armReleaseInitial();

}; // class arm
}// namespace jaguar_ns
#endif // ARM_H
