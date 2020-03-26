#include "../include/Arm.hpp"
#include <iostream>
namespace arm_ns
{
#define MOTOR_NUM 6

#define J1_UP_CMD       "!PR 1 30\r"
#define J1_DOWN_CMD     "!PR 1 -30\r"
#define J1_CMD 30
#define J2_UP_CMD       "!PR 2 -75\r"
#define J2_DOWN_CMD     "!PR 2 75\r"
#define J2_CMD 75
#define J3_UP_CMD       "!PR 1 10\r"
#define J3_DOWN_CMD     "!PR 1 -10\r"
#define J3_CMD 30
#define J4_UP_CMD       "!G 2 250\r"
#define J4_DOWN_CMD     "!G 2 -250\r"
#define J4_CMD 250
#define J5_UP_CMD       "!PR 1 30\r"
#define J5_DOWN_CMD     "!PR 1 -30\r"
#define J5_CMD 30
#define J6_UP_CMD       "!PR 2 10\r"
#define J6_DOWN_CMD     "!PR 2 -10\r"
#define J6_CMD 10

const double J1_OFFSET = 0.0;   // 0,
const double J2_OFFSET = 10.0;  //joint 2
const double J6_OFFSET = 0;    //tip
const double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
const double jointStartLimit[6] = {0,0,-PI,0,-PI/4,-PI};
const double jointEndtLimit[6] = {PI,PI,PI,0,PI/4,PI};
const double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
const double FULLAD = 4095;
const double JOINT_CIRCLECNT[6] = {19000,53706.7,3724,756,14275.333,3724};      //1:285 40/12,1:212  40/12, 1:49, 1:27, 1:49 46/12, 1:49  2/1
const double JOINT_INI_ANGLE[6] = { M_PI - M_PI * J1_OFFSET / 180, M_PI * J2_OFFSET / 180, 0, 0, 0, M_PI * J6_OFFSET / 180 };
const double ARMLEN1 = 0.455;
const double ARMLEN2 = 0.43;
const double ARMLEN31 = 0.20;
const double ARMLEN32 = 0.28;


Arm::Arm() {
    std::cout << "Arm::Arm()"  << std::endl;

    ip_1 = "192.168.0.63";
    ip_2 = "192.168.0.62";
    ip_1_port_1 = 10001 ;
    ip_1_port_2 = 10002 ;
    ip_2_port_1 = 10001 ;
    // ip_2_port_2 = 10002 ; // not used
    connect();

    m_motor1Connected = false;
    m_allArmMotorState = false;
    m_watchDogCnt1 = 2;
    m_watchDogCnt2 = 2;



    tipAngle = 0;
    armIniFlag = true;
    pushButtonArmSetIni = false;
    for (int i = 0; i < 6; i++)
    {
        jointMotorData[i].circleCnt = JOINT_CIRCLECNT[i];
        jointMotorData[i].resolution = jointMotorData[i].circleCnt / (2 * M_PI);
        jointMotorData[i].iniPos = 0;
        jointMotorData[i].angle = JOINT_INI_ANGLE[i];
        jointMotorData[i].iangle = File::getInstance()->getArm(i);
        jointMotorData[i].motorTemperature = 0;
        jointMotorData[i].cmdDir = 1;
        jointMotorData[i].encoderDir = 1;
        jointMotorData[i].pwmOutput = 0;
        jointMotorData[i].encodeSpeed = 0;
        jointMotorData[i].protect = false;
        jointMotorData[i].limitPosStart = jointStartLimit[i];
        jointMotorData[i].limitPosEnd = jointEndtLimit[i];
        jointMotorData[i].limitPos1 = 0;
        jointMotorData[i].limitPos2 = 0;
        jointMotorData[i].stuckFlag = false;
        jointMotorData[i].preCmdDir = 1;
        jointMotorData[i].manipulatorStuckCnt = 0;
        jointMotorData[i].jointCmd = 0;
        jointMotorData[i].jointJoy = false;
        jointMotorData[i].jointCtrl = false;
        jointMotorData[i].motorTemperature = 0;
        jointMotorData[i].currentAmp = 0;

    }
}

void Arm::connect()
{

    ipTCP_1 = std::make_shared<tcpSocket>(ip_1, ip_1_port_1);
    ipTCP_2 = std::make_shared<tcpSocket>(ip_1, ip_1_port_2);
    ipTCP_3 = std::make_shared<tcpSocket>(ip_2, ip_1_port_1);


    print("connecting to Arm ipTCP_1 ...");
    if (ipTCP_1->connect())
        print("Arm ipTCP_1 is connected");
    else
        print("Failed to connect to Arm ipTCP_1");
    
    print("connecting to Arm ipTCP_2 ...");
    if (ipTCP_2->connect())
        print("Arm ipTCP_2 is connected");
    else
        print("Failed to connect to Arm ipTCP_2");
    

    print("connecting to Arm ipTCP_3 ...");
    if (ipTCP_3->connect())
        print("Arm ipTCP_3 is connected");
    else
        print("Failed to connect to Arm ipTCP_3");
}

void Arm::sendPings()
{
	ipTCP_1->send("PING\r\n");
    ipTCP_2->send("PING\r\n");
    ipTCP_3->send("PING\r\n");
}

Arm::~Arm() {
    // if ( ipTCP_1 ) {
    //     ipTCP_1->close();
    //     delete ipTCP_1;
    // }
    // if (ipTCP_2){
    //     ipTCP_2->close();
    //     delete ipTCP_2;
    // }
    // if (ipTCP_3){
    //     ipTCP_3->close();
    //     delete ipTCP_3;
    // }
    std::cout << "Arm::~Arm()"  << std::endl;
}




// void Arm::processRobotData1() {
//     std::string received = "";
//     int count = 0;
//     m_watchDogCnt1 = 0;
//     // while( (count = m_tcpRobot1->bytesAvailable()) > 0 ) {
//         received = ipTCP_1->read();
//         m_receivedData1.append(received);
//     // }
//     if ( endWith(m_receivedData1,"\r") ) {
//         dealWithPackage1(m_receivedData1);
//         m_receivedData1 = "";
//     }
// }

void Arm::processRobotData2()
{
    std::string received = "";
    int count = 0;
    m_watchDogCnt1 = 0;
    // while( (count = ipTCP_2->bytesAvailable()) > 0)
    // {
        received = ipTCP_2->read();
        m_receivedData2.append(received);
    // }
    if (endWith(m_receivedData2,"\r"))
    {
        dealWithPackage2(m_receivedData2);
        m_receivedData2 = "";
    }
}

void Arm::processRobotData3()
{
    std::string received = "";
    int count = 0;

    m_watchDogCnt2 = 0;
    // while( (count = ipTCP_3->bytesAvailable()) > 0)
    // {

        received = ipTCP_3->read();
        m_receivedData3.append(received);
    // }
    if (endWith(m_receivedData3 , "\r"))
    {
        dealWithPackage3(m_receivedData3);
        m_receivedData3 = "";
    }
}

// void Arm:: dealWithPackage1(std::string received) {
//     std::string temp;
//     std::vector<std::string> rev = splitByString(received,"\r");
     
//     for (int i = 0; i < rev.size(); i++)
//     {
//         received = rev.at(i);

//         if (startWith(received,"A="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string>  strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 1)
//                 {
//                     motorData1.motAmp1 = std::stod(strData.at(0))/10;

//                     motorData1.motAmp2 = std::stod(strData.at(1))/10;
//                     // temp.setNum(motorData1.motAmp1,'g',4);
//                     //                     ui->lineEditM1Current->setText(temp);
//                     // temp.setNum(motorData1.motAmp2,'g',4);
//                     //                     ui->lineEditM2Current->setText(temp);
//                     jointMotorData[0].currentAmp = motorData1.motAmp1;
//                     jointMotorData[1].currentAmp = motorData1.motAmp2;

//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"AI="))
//         {
//             received.erase(0,3);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string> strData = split(received, ':');
//             try
//             {
//                 if (strData.size() > 3)
//                 {
//                     motorData1.ai3 = std::stod(strData.at(2));
//                     motorData1.motTemp1 = ad2Temperature(motorData1.ai3);
//                     motorData1.ai4 = std::stod(strData.at(3));

//                     motorData1.motTemp2 = ad2Temperature(motorData1.ai4);
//                     // temp.setNum(motorData1.motTemp1,'g',4);
//                     //                     ui->lineEditM1Temp->setText(temp);
//                     // temp.setNum(motorData1.motTemp2,'g',4);
//                     //                     ui->lineEditM2Temp->setText(temp);
//                     jointMotorData[0].motorTemperature = motorData1.motTemp1;
//                     jointMotorData[1].motorTemperature = motorData1.motAmp2;
//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"C="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//              std::vector<std::string>  strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 1)
//                 {
//                     motorData1.motEncP1 =  std::stoi(strData.at(0));
//                     motorData1.motEncP2 =  std::stoi(strData.at(1));
//                     // temp.setNum(motorData1.motEncP1);
//                     //                     ui->lineEditM1Pos->setText(temp);
//                     // temp.setNum(motorData1.motEncP2);
//                     //                     ui->lineEditM2Pos->setText(temp);
//                     jointMotorData[0].encoderPos = motorData1.motEncP1;
//                     jointMotorData[1].encoderPos = motorData1.motEncP2;
//                     jointMotorData[0].angle = trans2Angle(0);
//                     jointMotorData[1].angle = -(trans2Angle(1) - JOINT_INI_ANGLE[1]) + jointMotorData[0].angle;
//                     jointMotorData[1].angle = (trans2Angle(1));
//                     // temp.setNum(jointMotorData[0].angle * 180/M_PI,'f',1);
//                     //                     ui->lineEditJ1Angle->setText(temp);
//                     // temp.setNum(jointMotorData[1].angle * 180 /M_PI,'f',1);
//                     //                     ui->lineEditJ2Angle->setText(temp);
//                     //get arm tip end position
//                     getPositionXY();
//                     // temp.setNum(manipulatorPos[0],'f',2);
//                     //                     ui->lineEditArmPosX->setText(temp);
//                     // temp.setNum(manipulatorPos[1],'f',2);
//                     //                     ui->lineEditArmPosY->setText(temp);
//                     //publish all the joint information here
//                     //publish sensor data here
//                     int motorPos[MOTOR_NUM];
//                     int motorVel[MOTOR_NUM];
//                     int motorPWM[MOTOR_NUM];
//                     double motorTemperature[MOTOR_NUM];
//                     double jointAngle[MOTOR_NUM];
//                     for(int i = 0; i < MOTOR_NUM; i++)
//                     {
//                         motorPos[i] = jointMotorData[i].encoderPos;
//                         motorVel[i] = jointMotorData[i].encodeSpeed;
//                         motorPWM[i] = jointMotorData[i].pwmOutput;
//                         motorTemperature[i] = jointMotorData[i].motorTemperature;
//                         jointAngle[i] = jointMotorData[i].angle;
//                     }
//                     //     						 qnode.publisher(motorPos,motorVel,motorPWM,motorTemperature,jointAngle,MOTOR_NUM) ;

//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"P="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string> strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 1)
//                 {
//                     motorData1.motPower1 = std::stoi(strData.at(0));
//                     motorData1.motPower2 = std::stoi(strData.at(1));
//                     // temp.setNum(motorData1.motPower1);
//                     //                     ui->lineEditM1Power->setText(temp);
//                     // temp.setNum(motorData1.motPower2);
//                     //                     ui->lineEditM2Power->setText(temp);
//                     jointMotorData[0].pwmOutput = motorData1.motPower1;
//                     jointMotorData[1].pwmOutput = motorData1.motPower2;
//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"S="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string>  strData = split(received,':');
//             try
//             {
//                 if (strData.size() > 1)
//                 {
//                     motorData1.motEncS1 = std::stoi(strData.at(0));
//                     motorData1.motEncS2 = std::stoi(strData.at(1));
//                     // temp.setNum(motorData1.motEncS1);
//                     //                     ui->lineEditM1Vel->setText(temp);
//                     // temp.setNum(motorData1.motEncS2);
//                     //                     ui->lineEditM2Vel->setText(temp);
//                     jointMotorData[0].encodeSpeed = motorData1.motEncS1;
//                     jointMotorData[1].encodeSpeed = motorData1.motEncS2;
//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"T="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//               std::vector<std::string> strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 1)
//                 {
//                     motorData1.ch1Temp = std::stoi(strData.at(0));
//                     motorData1.ch2Temp =  std::stoi(strData.at(1));
//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"V="))
//         {
//             received.erase(0,2);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string> strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 2)
//                 {
//                     motorData1.drvVoltage = std::stod(strData.at(0))/10;
//                     motorData1.batVoltage = std::stod(strData.at(1))/10;
//                     motorData1.reg5VVoltage = std::stod(strData.at(2))/1000;
//                     // temp.setNum(motorData1.batVoltage,'g',4);
//                     //                     ui->lineEditBatVol->setText(temp);

//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         else if (startWith(received,"MMOD="))
//         {
//             received.erase(0,5);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string>  strData = split(received,':');
//             try
//             {
//                 if (strData.size()> 1)
//                 {
//                     motorData1.mode1 =  std::stoi(strData.at(0));
//                     motorData1.mode2 = std::stoi(strData.at(1));
//                 }
//             }
//             catch(...)
//             {

//             }
//         }
//         // else if (startWith(received,"FF="))
//         // {
//         //     received.erase(0,3);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);

//             // try
//             // {
//             //     motorData1.statusFlag = std::stoi(received);
//             //     temp="";
//             //     if((motorData1.statusFlag & 0x01) != 0)
//             //     {
//             //         temp.append("OverHeat+");
//             //     }
//             //     if((motorData1.statusFlag & 0x02) != 0)
//             //     {
//             //         temp.append("OverVoltage+");
//             //     }
//             //     if((motorData1.statusFlag & 0x04) != 0)
//             //     {
//             //         temp.append("UnderVol+");
//             //     }
//             //     if((motorData1.statusFlag & 0x08) != 0)
//             //     {
//             //         temp.append("Short+");
//             //     }
//             //     if((motorData1.statusFlag & 0x10) != 0)
//             //     {
//             //         temp.append("ESTOP+");
//             //     }
//             //     if (motorData1.statusFlag == 0)
//             //     {
//             //         temp.append("OK");
//             //     }

//             //     //                 ui->lineEditChannel1State->setText(temp);
//             // }
//             // catch(...)
//             // {

//             // }
//         // }
//     }
// }

std::string Arm:: read_ipTCP_1()
{
    return ipTCP_1->read();
}

std::string Arm:: read_ipTCP_2()
{
    return ipTCP_2->read();
}

std::string Arm:: read_ipTCP_3()
{
    return ipTCP_3->read();
}

void Arm:: dealWithPackage2(std::string received)
{
    std::string temp;
    std::vector<std::string> rev = splitByString(received,"\r");
    for (int i = 0; i < rev.size(); i++)
    {
        received = rev.at(i);
        if (startWith(received,"A="))
        {
            received.erase(0,2);
            std::vector<std::string>  strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.motAmp1 = std::stod(strData.at(0))/10;
                    motorData2.motAmp2 = std::stod(strData.at(1))/10;
                    jointMotorData[2].currentAmp = motorData2.motAmp1;
                    jointMotorData[3].currentAmp = motorData2.motAmp2;

                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"AI="))
        {
            received.erase(0,3);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 3)
                {
                    motorData2.ai3 = std::stod(strData.at(2));
                    motorData2.motTemp1 = ad2Temperature(motorData2.ai3);
                    motorData2.ai4 = std::stod(strData.at(3));
                    motorData2.motTemp2 = ad2Temperature(motorData2.ai4);
                    jointMotorData[2].motorTemperature = motorData2.motTemp1;
                    jointMotorData[3].motorTemperature = motorData2.motTemp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"C="))
        {
            received.erase(0,2);
             std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.motEncP1 = std::stoi(strData.at(0));
                    motorData2.motEncP2 = std::stoi(strData.at(1));
                    jointMotorData[2].encoderPos = motorData2.motEncP1;
                    jointMotorData[3].encoderPos = motorData2.motEncP2;
                    jointMotorData[2].angle = trans2Angle(2);
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"P="))
        {
            received.erase(0,2);
            std::vector<std::string>  strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.motPower1 = std::stoi(strData.at(0));
                    motorData2.motPower2 = std::stoi(strData.at(1));
                    jointMotorData[2].pwmOutput = motorData2.motPower1;
                    jointMotorData[3].pwmOutput = motorData2.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"S="))
        {
            received.erase(0,2);
             std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.motEncS1 = std::stoi(strData.at(0));
                    motorData2.motEncS2 = std::stoi(strData.at(1));
                    jointMotorData[2].encodeSpeed = motorData2.motEncS1;
                    jointMotorData[3].encodeSpeed = motorData2.motEncS2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"T="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.ch1Temp = std::stod(strData.at(0));
                    motorData2.ch2Temp = std::stoi(strData.at(1));
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"V="))
        {
            received.erase(0,2);
            std::vector<std::string>  strData = split(received,':');
            try
            {
                if (strData.size()> 2)
                {
                    motorData2.drvVoltage = std::stod(strData.at(0))/10;
                    motorData2.batVoltage =  std::stod(strData.at(1))/10;
                    motorData2.reg5VVoltage =  std::stod(strData.at(2))/1000;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"MMOD="))
        {
            received.erase(0,5);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData2.mode1 = std::stoi(strData.at(0));
                    motorData2.mode2 = std::stoi(strData.at(1));
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"FF="))
        {
            received.erase(0,3);
            try
            {
                motorData2.statusFlag = std::stoi(received);
                temp="";
                if((motorData2.statusFlag & 0x01) != 0)
                {
                    lineEditChannel2State = ("OverHeat+");
                }
                if((motorData2.statusFlag & 0x02) != 0)
                {
                   lineEditChannel2State = ("OverVoltage+");
                }
                if((motorData2.statusFlag & 0x04) != 0)
                {
                    lineEditChannel2State = ("UnderVol+");
                }
                if((motorData2.statusFlag & 0x08) != 0)
                {
                   lineEditChannel2State = ("Short+");
                }
                if((motorData2.statusFlag & 0x10) != 0)
                {
                    lineEditChannel2State = ("ESTOP+");
                }
                if (motorData1.statusFlag == 0)
                {
                    lineEditChannel2State = ("OK");
                }
            }
            catch(...)
            {

            }
        }
    }

}

void Arm:: dealWithPackage3(std::string received)
{
    std::string temp;
    std::vector<std::string> rev = splitByString(received,"\r");
    for (int i = 0; i < rev.size(); i++)
    {
        received = rev.at(i);

        if (startWith(received,"A="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.motAmp1 = std::stod(strData.at(0))/10;

                    motorData3.motAmp2 =  std::stod(strData.at(1))/10;
                    jointMotorData[4].currentAmp = motorData3.motAmp1;
                    jointMotorData[5].currentAmp = motorData3.motAmp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"AI="))
        {
            received.erase(0,3);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 3)
                {
                    motorData3.ai3 = std::stod(strData.at(2));
                    motorData3.motTemp1 = ad2Temperature(motorData3.ai3);
                    motorData3.ai4 = std::stod(strData.at(3));
                    motorData3.motTemp2 = ad2Temperature(motorData3.ai4);
                    jointMotorData[4].motorTemperature = motorData3.motTemp1;
                    jointMotorData[5].motorTemperature = motorData3.motTemp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"C="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.motEncP1 = std::stoi(strData.at(0));
                    motorData3.motEncP1 = std::stoi(strData.at(0));
                    motorData3.motEncP2 = std::stoi(strData.at(1));
                    jointMotorData[4].encoderPos = motorData3.motEncP1;
                    jointMotorData[5].encoderPos = motorData3.motEncP2;
                    jointMotorData[4].angle = -trans2Angle(4);
                    jointMotorData[5].angle = -trans2Angle(5);
                    tipAngle = jointMotorData[5].angle + (jointMotorData[1].angle);
                    lineEditTipAngle = tipAngle * 180 /M_PI;

                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"P="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.motPower1 = std::stoi(strData.at(0));
                    motorData3.motPower2 = std::stoi(strData.at(1));
                    jointMotorData[4].pwmOutput = motorData3.motPower1;
                    jointMotorData[5].pwmOutput = motorData3.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"S="))
        {
            received.erase(0,2);
            std::vector<std::string>strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.motEncS1 = std::stoi(strData.at(0));
                    motorData3.motEncS2 = std::stoi(strData.at(1));
                    jointMotorData[4].encodeSpeed = motorData3.motEncS1;
                    jointMotorData[5].encodeSpeed = motorData3.motEncS2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"T="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.ch1Temp = std::stod(strData.at(0));
                    motorData3.ch2Temp = std::stoi(strData.at(1));
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"V="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received,':');
            try
            {
                if (strData.size()> 2)
                {
                    motorData3.drvVoltage = std::stod(strData.at(0))/10;
                    motorData3.batVoltage = std::stod(strData.at(1))/10;
                    motorData3.reg5VVoltage = std::stod(strData.at(2))/1000;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"MMOD="))
        {
            received.erase(0,5);
            std::vector<std::string>  strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                    motorData3.mode1 = std::stoi(strData.at(0));
                    motorData3.mode2 = std::stoi(strData.at(1));
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"FF="))
        {
            received.erase(0,3);

            try
            {
                motorData3.statusFlag = std::stoi(received);
                temp="";
                if((motorData3.statusFlag & 0x01) != 0)
                {
                  lineEditChannel3State = ("OverHeat+");
                }
                if((motorData3.statusFlag & 0x02) != 0)
                {
                   lineEditChannel3State = ("OverVoltage+");
                }
                if((motorData3.statusFlag & 0x04) != 0)
                {
                    lineEditChannel3State = ("UnderVol+");
                }
                if((motorData3.statusFlag & 0x08) != 0)
                {
                    lineEditChannel3State = ("Short+");
                }
                if((motorData3.statusFlag & 0x10) != 0)
                {
                   lineEditChannel3State = ("ESTOP+");
                }
                if (motorData1.statusFlag == 0)
                {
                   lineEditChannel3State = ("OK");
                }
            }
            catch(...)
            {

            }
        }
    }
}

double Arm::ad2Temperature(int adValue)
{
    //for new temperature sensor
    double tempM = 0;
    double k = (adValue / FULLAD);
    double resValue = 0;
    if (k != 0)
    {
        resValue = (10000 / k -10000);      //AD value to resistor
    }
    else
    {
        resValue = resTable[0];
    }


    int index = -1;
    if (resValue >= resTable[0])       //too lower
    {
        tempM = -20;
    }
    else if (resValue <= resTable[24])
    {
        tempM = 100;
    }
    else
    {
        for (int i = 0; i < 24; i++)
        {
            if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
            {
                index = i;
                break;
            }
        }
        if (index >= 0)
        {
            tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
        }
        else
        {
            tempM = 0;
        }

    }

    return tempM;
}

void Arm::moveMotor_0(int PWM){
   
    double deltaAngle = (PWM / jointMotorData[4].resolution);
    double endPostion = jointMotorData[4].iangle + deltaAngle;
    if (endPostion <= jointMotorData[4].limitPosStart)
    {
       
        deltaAngle = jointMotorData[4].limitPosStart - jointMotorData[4].iangle;
        PWM = int(deltaAngle * jointMotorData[4].resolution); // converting the angle to PWM
    }

    if (endPostion >= jointMotorData[4].limitPosEnd)
    {
        deltaAngle = jointMotorData[4].limitPosEnd - jointMotorData[4].iangle;
        PWM = int(deltaAngle * jointMotorData[4].resolution); // converting the angle to PWM
    }

    if (ipTCP_3)
    {
        if(PWM != 0) {
            ipTCP_3->send("!PR 1 " + toString(PWM) + "\r");
            jointMotorData[4].iangle += deltaAngle;
            File::getInstance()->setArm(jointMotorData[4].iangle, 4);
        } 
    }
}
void Arm::moveMotor_1(int PWM)
{
    double deltaAngle = (PWM / jointMotorData[0].resolution);
    double endPostion = jointMotorData[0].iangle + deltaAngle;
    if (endPostion <= jointMotorData[0].limitPosStart)
    {
        deltaAngle = jointMotorData[0].limitPosStart - jointMotorData[0].iangle;
        PWM = int(deltaAngle * jointMotorData[0].resolution); // converting the angle to PWM
    }
    if (endPostion >= jointMotorData[0].limitPosEnd)
    {
        deltaAngle = jointMotorData[0].limitPosEnd - jointMotorData[0].iangle;
        PWM = int(deltaAngle * jointMotorData[0].resolution); // converting the angle to PWM
    }

    if (ipTCP_1)
    {

        if(PWM != 0){

                ipTCP_1->send("!PR 1 " + toString(PWM) + "\r");
                jointMotorData[0].iangle += deltaAngle;
                File::getInstance()->setArm(jointMotorData[0].iangle, 0);
                // if ((jointMotorData[1].iangle + deltaAngle) < (jointMotorData[1].limitPosStart))
                // {
                //     moveMotor_2(0);
                // }
        }
    }
}

void Arm::moveMotor_2(int PWM)
{  
    double startLimit = jointMotorData[1].limitPosStart - jointMotorData[0].iangle; //offset the start limit
    double endLimit = jointMotorData[1].limitPosEnd - jointMotorData[0].iangle;//offset the end limit
    double deltaAngle = (PWM / jointMotorData[1].resolution); //offset the end limit 
    deltaAngle -= jointMotorData[0].iangle; // offset the delta andgle relative to motor 1
    double endPostion =  jointMotorData[1].iangle + deltaAngle ;
    
    if (endPostion <= (startLimit ) )  deltaAngle = startLimit - jointMotorData[1].iangle; 
    if( endPostion >= (endLimit  ) ) deltaAngle = endLimit - jointMotorData[1].iangle; 
    
    PWM = (deltaAngle * jointMotorData[1].resolution);

    if (ipTCP_1)
    {
        if(PWM != 0){
            ipTCP_1->send("!PR 2 " + toString(-PWM) + "\r");
            jointMotorData[1].iangle += deltaAngle;
            File::getInstance()->setArm(jointMotorData[1].iangle, 1); 
        }
          
    }
}


void Arm::moveMotor_3(int PWM)
{
   
   double deltaAngle = (PWM / jointMotorData[5].resolution);
   double endPostion =  jointMotorData[5].iangle + deltaAngle;

   if(jointMotorData[1].iangle != 0 || jointMotorData[0].iangle != 0 ){

        if (endPostion <= jointMotorData[5].limitPosStart){
            deltaAngle = jointMotorData[5].limitPosStart - jointMotorData[5].iangle; 
            PWM = int(deltaAngle * jointMotorData[5].resolution);        // converting the angle to PWM  
        }

        if( endPostion >= jointMotorData[5].limitPosEnd ){
            deltaAngle = jointMotorData[5].limitPosEnd - jointMotorData[5].iangle;  
            PWM = int(deltaAngle * jointMotorData[5].resolution);        // converting the angle to PWM  
        }

        if (ipTCP_3)
        {
             if(PWM != 0){
                ipTCP_3->send("!PR 2 " + toString(PWM) + "\r");
                jointMotorData[5].iangle += deltaAngle;
                File::getInstance()->setArm(jointMotorData[5].iangle, 5);
             }  
        }
    }
}

void Arm::moveMotor_4(int PWM){
    double deltaAngle = (PWM / jointMotorData[2].resolution);
    double endPostion = jointMotorData[2].iangle + deltaAngle;
    if (endPostion <= jointMotorData[2].limitPosStart)
    {
        deltaAngle = jointMotorData[2].limitPosStart - jointMotorData[2].iangle;
        PWM = int(deltaAngle * jointMotorData[2].resolution); // converting the angle to PWM
    }
    if (endPostion >= jointMotorData[2].limitPosEnd)
    {
        deltaAngle = jointMotorData[2].limitPosEnd - jointMotorData[2].iangle;
        PWM = int(deltaAngle * jointMotorData[2].resolution); // converting the angle to PWM
    }

    if (ipTCP_2)
    {
        if(PWM != 0)   ipTCP_2->send("!PR 1 " + toString(PWM) +"\r");
        jointMotorData[2].iangle += deltaAngle;
        File::getInstance()->setArm(jointMotorData[2].iangle, 2);
    }  
}

void Arm::moveMotor_5(int PWM){
    if (ipTCP_2){
           if(PWM != 0) ipTCP_2->send("!G 2 "+ toString(PWM)+ "\r");
    } 
}


void Arm::move_arm_degree(double joint_0,double joint_1, double joint_2,double joint_3,double joint_4,double joint_5)
{
        j0Go_to_degree(joint_0 );
        j1Go_to_degree(joint_1 );
        j2Go_to_degree(joint_2 );
        j3Go_to_degree(joint_3 );
        j4Go_to_degree(joint_4 );
        
        moveMotor_5(joint_5);
}

void Arm::move_arm_radians(double joint_0,double joint_1, double joint_2,double joint_3,double joint_4,double joint_5)
{

        j1Go_to_radians(joint_1 );
        j2Go_to_radians(joint_2 );
        j3Go_to_radians(joint_3 );
        j4Go_to_radians(joint_4 );
        j0Go_to_radians(joint_0 );
        moveMotor_5(joint_5);
}

void Arm::motor1Down()
{
    if (ipTCP_1)
    {
        if (armIniFlag)
        {
            if ((jointMotorData[0].encoderPos - J1_CMD) >= jointMotorData[0].limitPos1)
            {
                ipTCP_1->send(J1_DOWN_CMD);
            }
        }
        else
        {
            ipTCP_1->send(J1_DOWN_CMD);
        }
    }
}

void Arm::motor1Up()
{
    if (ipTCP_1)
    {
        if (armIniFlag)
        {
            if ((jointMotorData[0].encoderPos + J1_CMD) <= jointMotorData[0].limitPos2)
            {
                ipTCP_1->send(J1_UP_CMD);
            }
        }
        else
        {
            ipTCP_1->send(J1_UP_CMD);
        }
    }
}

void Arm::motor2Down()
{
    if (ipTCP_1){
            if (armIniFlag){
                if ((jointMotorData[1].encoderPos + J2_CMD) <= jointMotorData[1].limitPos2)
                {
                    ipTCP_1->send(J2_DOWN_CMD);
                }
            }
            else{
                ipTCP_1->send(J2_DOWN_CMD);
            }
    }
}

void Arm::motor2Up()
{
    if (ipTCP_1){
            if (armIniFlag){
                if ((jointMotorData[1].encoderPos - J2_CMD) >= jointMotorData[1].limitPos1)
                {
                    ipTCP_1->send(J2_UP_CMD);
                }
            }
            else{
                ipTCP_1->send(J2_UP_CMD);
            }
    }
}


void Arm::motor3Down()
{
    if (ipTCP_2){
            if (armIniFlag){
                if ((jointMotorData[2].encoderPos - J3_CMD) >= jointMotorData[2].limitPos1)
                {
                    ipTCP_2->send(J3_DOWN_CMD);
                }
            }
            else{
                ipTCP_1->send(J3_DOWN_CMD);
            }
        
    }
}

void Arm::motor3Up()
{
    if (ipTCP_2){

            if (armIniFlag){
                if ((jointMotorData[2].encoderPos + J3_CMD) <= jointMotorData[2].limitPos2)
                {
                    ipTCP_2->send(J3_UP_CMD);
                }
            }
            else{
                ipTCP_2->send(J3_UP_CMD);
            }
    }
}


void Arm::motor4Down()
{
    if (ipTCP_2 ){
            ipTCP_2->send(J4_DOWN_CMD);
    }
}

void Arm::motor4Up()
{
    if (ipTCP_2){
            ipTCP_2->send(J4_UP_CMD);
    }
}

void Arm::motor5Down()
{
    if (ipTCP_3 ){
            if (armIniFlag){
                if ((jointMotorData[4].encoderPos - J5_CMD) >= jointMotorData[4].limitPos1)
                {
                    ipTCP_3->send(J5_DOWN_CMD);
                }
            }
            else{
                ipTCP_3->send(J5_DOWN_CMD);
            }  
    }
}

void Arm::motor5Up()
{
    if (ipTCP_3 ){
            if (armIniFlag){
                if ((jointMotorData[4].encoderPos + J5_CMD) <= jointMotorData[4].limitPos2)
                {
                    ipTCP_3->send(J5_UP_CMD);
                }
            }
            else{
                ipTCP_3->send(J5_UP_CMD);
            }
    }
}

void Arm::motor6Down()
{
    if (ipTCP_3 ){
            if (armIniFlag){
                if ((jointMotorData[5].encoderPos - J6_CMD) >= jointMotorData[5].limitPos1)
                {
                    ipTCP_3->send(J6_DOWN_CMD);
                }
            }
            else{
                ipTCP_3->send(J6_DOWN_CMD);
            }
    }
}

void Arm::motor6Up()
{
    if (ipTCP_3 ){
            if (armIniFlag){
                if ((jointMotorData[5].encoderPos + J6_CMD) <= jointMotorData[5].limitPos2)
                {
                    ipTCP_3->send(J6_UP_CMD);
                }
            }
            else{
                ipTCP_3->send(J6_UP_CMD);
            }
            ipTCP_3->send(J6_UP_CMD);
    }
}

void Arm::motor1Stop()
{
    if (ipTCP_1){
            ipTCP_1->send("!PR 1 0\r");
        }
}

void Arm::motor2Stop()
{
    if (ipTCP_1){
            ipTCP_1->send("!PR 2 0\r");
    }
}

void Arm::motor3Stop()
{
    if (ipTCP_2 ){
            ipTCP_2->send("!PR 1 0\r");
    }
}

void Arm::motor4Stop()
{
    if (ipTCP_2){
            ipTCP_2->send("!G 2 0\r");
    }

}

void Arm::motor5Stop()
{
    if (ipTCP_3){
            ipTCP_3->send("!PR 1 0\r");
    }

}

void Arm::motor6Stop()
{
    if (ipTCP_3 ){
            ipTCP_3->send("!PR 2 0\r");
    }

}

void Arm::motorStopAll()
{
    if (m_allArmMotorState)
    {
        if(ipTCP_1){
                ipTCP_1->send("!EX\r");
        }

        if(ipTCP_2 ){
                ipTCP_2->send("!EX\r");
        }

        if(ipTCP_3 ){
                ipTCP_3->send("!EX\r");
        }
        m_allArmMotorState = false;
    }
}
void Arm::motorReleaseAll()
{
    if (!m_allArmMotorState)
    {
        if(ipTCP_1 != nullptr){
                ipTCP_1->send("!MG\r");
        }

        if(ipTCP_2 != nullptr){
                ipTCP_2->send("!MG\r");
        }

        if(ipTCP_3 != nullptr){
                ipTCP_3->send("!MG\r");
        }
        m_allArmMotorState = true;
    }
}

void Arm::sendQuery1() {
    if ( ipTCP_1 ) {
        // if (ipTCP_1->isWritable())
            ipTCP_1->send("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
    }
}

void Arm::sendQuery2()
{
    if (ipTCP_2){
        // if (tcpRobot2->isWritable()){
            ipTCP_2->send("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        // }
    }
}

void Arm::sendQuery3()
{
    if (ipTCP_3 ){
        // if (ipTCP_3->isWritable()){
            ipTCP_3->send("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        // }
    }
}
void Arm::sendQuery4()
{
    if (ipTCP_4){
        // if (ipTCP_4->isWritable()){
            ipTCP_4->send("# C_?A_?A_?AI_?C_?FF_?P_?S_?T_?V_# 20\r");
        // }
    }
}

void Arm::jointAngleCmdSend(int channel,double angle)
{
    // std::string cmdStr = "";
   // cmdStr.setNum(angle / M_PI * 180, 'f',2);
    double  cmdStr = angle / M_PI * 180 ;
    if(armIniFlag)
    {
        if (channel == 1)
        {
            lineEditJ1TargetAngle = cmdStr;
            j1GoCmd();
        }
        else if(channel == 2)
        {
            lineEditJ2TargetAngle = cmdStr;
            j2GoCmd();
        }
        else if(channel == 3)
        {
            lineEditRotateTargetAngle = cmdStr;
            rotateGoCmd();
        }
        else if(channel == 4)
        {
            //no position contor??//
        }
        else if(channel == 5)
        {
            lineEditPanTargetAngle = cmdStr;
            panGoCmd();
        }
        else if(channel == 6)
        {
            lineEditTipTargetAngle = cmdStr;
            tipGoCmd();
        }

    }
}

void Arm::armPosCmdSend(double x, double y, double tipAngle)
{
    std::string cmdStr = "";

    if(armIniFlag)
    {
        if ((x == 0) && (y == 0) && (tipAngle == 0))
        {
            //means reset arm
            armResetCmd();
        }
        else
        {
            lineEditArmTargetX = x;
            lineEditArmTargetY = y;
            lineEditTipAngle = tipAngle * M_PI / 180 ;
            armPosGoCmd();
        }

    }

}

void Arm::cmdSend(int channel,int cmdValue,int motorCtrl)
{

    if (channel == 0)
    {
        if (motorCtrl == 3)
        {
            ipTCP_1->send("!PR 1 " + toString(cmdValue) + "\r");
        }

    }
    else if(channel == 1)
    {
        if (motorCtrl == 3)
        {
            ipTCP_1->send("!PR 2 " + toString(cmdValue) + "\r");

        }
    }
    else if(channel == 2)
    {
        if (motorCtrl == 3)
        {
            ipTCP_2->send("!PR 1 " + toString(cmdValue) + "\r");
        }
    }
    else if(channel == 3)
    {
        if (motorCtrl == 0)
        {
            ipTCP_2->send("!G 2 " + toString(cmdValue) + "\r");

        }
    }
}



double Arm:: trans2Angle(int channel)
{
    double angle = 0;
    //calculate the angle and position
    double delta = jointMotorData[channel].encoderPos - jointMotorData[channel].iniPos;
    delta =  delta / jointMotorData[channel].resolution;
    angle = JOINT_INI_ANGLE[channel ] - delta;
    return angle;
}

void Arm:: getPositionXY()
{
    double x = 0;
    double y = 0;
    double thelta1 = jointMotorData[0].angle;
    double thelta2 = jointMotorData[1].angle;
    double armX1 = ((ARMLEN1 * cos(thelta1)) );
    double armY1 = ((ARMLEN1 * sin(thelta1)) );

    double armX2 = armX1 + (ARMLEN2 * cos(thelta2) );
    double armY2 = armY1 + (ARMLEN2 * sin(thelta2) );
    x = armX2 + ARMLEN31 * cos(tipAngle);
    y = armY2 + ARMLEN31 * sin(tipAngle);
    manipulatorPos[0] = x;
    manipulatorPos[1] = y;
}

void Arm::j0Go_to_degree(double targetAngle){
    double deltaAngle = 0;
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
  
        deltaAngle = degreesToRadians(targetAngle) - jointMotorData[4].iangle;
        cmd = (int)(deltaAngle * jointMotorData[4].resolution);

                if (ipTCP_3){
                        cmdStr = "!PR 1 " + toString(cmd) +"\r";
                          moveMotor_0(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm::j1Go_to_degree(double targetAngle)
{

    double deltaAngle = 0;
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
        // targetAngle = ui->lineEditJ1TargetAngle->text().toDouble();
        targetAngle = degreesToRadians(targetAngle);                       // converting to radians 
        deltaAngle = targetAngle - jointMotorData[0].iangle;          // calculating the angle different between current angle and the target angle
        cmd = int(deltaAngle * jointMotorData[0].resolution);        // converting the angle to PWM
        // targetPos = cmd + jointMotorData[0].encoderPos;
                if (ipTCP_1){
                        moveMotor_1(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm::j2Go_to_degree(double targetAngle)
{

    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        // targetAngle = ui->lineEditJ1TargetAngle->text().toDouble();
        targetAngle = degreesToRadians(targetAngle);                       // converting to radians 
        deltaAngle = targetAngle - jointMotorData[1].iangle;          // calculating the angle different between current angle and the target angle
        cmd = int(deltaAngle * jointMotorData[1].resolution);        // converting the angle to PWM
        // targetPos = cmd + jointMotorData[1].encoderPos;
                if (ipTCP_1){
                        moveMotor_2(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm:: j3Go_to_degree(double targetAngle)
{
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
        cmd = (int)(degreesToRadians(targetAngle) * jointMotorData[5].resolution);
        moveMotor_3(cmd);
    }
    catch(...)
    {

    }

}

void Arm::j4Go_to_degree(double targetAngle){
    // double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    // QByteArray cmdByte;
    try
    {
        deltaAngle =  degreesToRadians(targetAngle) - jointMotorData[2].iangle;
        cmd = (int)(-deltaAngle * jointMotorData[2].resolution);
        moveMotor_4(-cmd);
    }
    catch(...)
    {

    }
}


void Arm::j0Go_to_radians(double targetAngle){
    double deltaAngle = 0;
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
  
        deltaAngle = targetAngle - jointMotorData[4].iangle;
        cmd = (int)(deltaAngle * jointMotorData[4].resolution);

                if (ipTCP_3){
                    cmdStr = "!PR 1 " + toString(cmd) +"\r";
                    moveMotor_0(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm::j1Go_to_radians(double targetAngle)
{

    double deltaAngle = 0;
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
        deltaAngle = targetAngle - jointMotorData[0].iangle;          // calculating the angle different between current angle and the target angle
        cmd = int(deltaAngle * jointMotorData[0].resolution);        // converting the angle to PWM
                if (ipTCP_1){
                    moveMotor_1(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm::j2Go_to_radians(double targetAngle)
{

    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        deltaAngle = targetAngle - jointMotorData[1].iangle;          // calculating the angle different between current angle and the target angle
        cmd = int(deltaAngle * jointMotorData[1].resolution);        // converting the angle to PWM
                if (ipTCP_1){
                    moveMotor_2(cmd);
                }
    }
    catch(...)
    {

    }
}

void Arm:: j3Go_to_radians(double targetAngle)
{
    int cmd = 0;
    std::string cmdStr = "";
    try
    {
        cmd = (int)(targetAngle * jointMotorData[5].resolution);
        moveMotor_3(cmd);
    }
    catch(...)
    {

    }
}

void Arm::j4Go_to_radians(double targetAngle){
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        deltaAngle =  targetAngle - jointMotorData[2].iangle;
        cmd = (int)(-deltaAngle * jointMotorData[2].resolution);
        moveMotor_4(-cmd);
    }
    catch(...)
    {

    }
}



void Arm:: j1GoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        targetAngle = targetAngle /180 * M_PI;
        // targetAngle = lineEditJ1TargetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[0].angle; //
        cmd = (int)(-deltaAngle * jointMotorData[0].resolution);
        targetPos = cmd + jointMotorData[0].encoderPos;
        if ((targetPos >= jointMotorData[0].limitPos1) && (targetPos <= jointMotorData[0].limitPos2))
        {
            if (!jointMotorData[0].protect)
            {
                if (ipTCP_1){
                    cmdStr = "!PR 1 " + toString(cmd) +"\r";
                    ipTCP_1->send(cmdStr);
                }
            }
        }

    }
    catch(...)
    {

    }
}

void Arm:: j2GoCmd()
{
    double targetAngle = 20;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        targetAngle = lineEditJ2TargetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[1].angle; 
        cmd = -int(deltaAngle * jointMotorData[1].resolution);
        targetPos = cmd + jointMotorData[1].encoderPos;
        if ((targetPos >= jointMotorData[1].limitPos1) && (targetPos <= jointMotorData[1].limitPos2))
        {
            if (!jointMotorData[1].protect)
            {
                if (ipTCP_1){
                        cmdStr = "!PR 2 " + toString(cmd) +"\r";
                        ipTCP_1->send(cmdStr);
                }
            }
        }

    }
    catch(...)
    {

    }
}

void Arm:: rotateGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    // QByteArray cmdByte;
    try
    {
        targetAngle = lineEditRotateTargetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[2].angle;
        cmd = (int)(-deltaAngle * jointMotorData[2].resolution);
        targetPos = cmd + jointMotorData[2].encoderPos;
        if ((targetPos >= jointMotorData[2].limitPos1) && (targetPos <= jointMotorData[2].limitPos2))
        {
            if (!jointMotorData[2].protect)
            {
                if (ipTCP_2){
                    if (ipTCP_2){
                        cmdStr = "!PR 1 " + toString(cmd) +"\r";
                        ipTCP_2->send(cmdStr);
                    }
                }
            }
        }
    }
    catch(...)
    {

    }
}

void Arm:: tipGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        targetAngle = lineEditTipTargetAngle /180 * M_PI;
        deltaAngle = targetAngle - tipAngle;
        cmd = (int)(deltaAngle * jointMotorData[5].resolution);
        targetPos = cmd + jointMotorData[5].encoderPos;
        if ((targetPos >= jointMotorData[5].limitPos1) && (targetPos <= jointMotorData[5].limitPos2))
        {
            if (!jointMotorData[5].protect)
            {
                if (ipTCP_3){
                    if (ipTCP_3){
                        cmdStr = "!PR 2 " + toString(cmd) +"\r";
                        ipTCP_3->send(cmdStr);
                    }
                }
            }
        }

    }
    catch(...)
    {

    }

}

void Arm:: panGoCmd()
{
    double targetAngle = 0;
    double deltaAngle = 0;
    int cmd = 0;
    int targetPos = 0;
    std::string cmdStr = "";
    try
    {
        targetAngle = lineEditPanTargetAngle /180 * M_PI;
        deltaAngle = targetAngle - jointMotorData[4].angle;
        cmd = (int)(deltaAngle * jointMotorData[4].resolution);
        targetPos = cmd + jointMotorData[4].encoderPos;
        if ((targetPos >= jointMotorData[4].limitPos1) && (targetPos <= jointMotorData[4].limitPos2))
        {
            if (!jointMotorData[4].protect)
            {
                if (ipTCP_3){
                        cmdStr = "!PR 1 " + toString(cmd) +"\r";
                        ipTCP_3->send(cmdStr);
                }
            }
        }
    }
    catch(...)
    {

    }
}

void Arm:: armPosGoCmd()
{
    std::string QStrTemp = "";
    std::string  cmdStr = "";
    double tipTargetAngle = 0;
    double x = 0;
    double y = 0;
    try
    {
        tipTargetAngle = lineEditTipAngle* M_PI/180;
        x = lineEditArmTargetX - ARMLEN31 * cos(tipTargetAngle);
        y = lineEditArmTargetY - ARMLEN31 * sin(tipTargetAngle);
        double l1 = ARMLEN1;
        double l2 = ARMLEN2;
        double s2 = 0;
        double thelta1 = 0;     //joint 1 angle

        double thelta2 = 0;     //joint 2 angle

        double belta = 0;
        double alpha = 0;
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        if (abs(c2) > 1)
        {
            //out of space
            thelta1 = -100;
            thelta2 = -100;

        }
        else
        {
            double r_2 = x*x + y * y;
            double l_sq = l1 * l1 + l2 * l2;
            double term2 = (r_2 -l_sq)/(2 *l1 * l2);
            double term1 = - (sqrt(1 - term2 * term2));
            thelta2 = atan2(term1,term2);
            //option
            //thelta2 = - thelta2;
            double k1 = l1 + l2 * cos(thelta2);
            double k2 = l2 * sin(thelta2);
            double r = sqrt(k1* k1 + k2 * k2);
            double gamma = atan2(k2,k1);
            thelta1 = atan2(y,x) - gamma;
            thelta2 = thelta1 + thelta2;

        }
        lineEditJ1TargetAngle = thelta1 / M_PI * 180;
        lineEditJ2TargetAngle = thelta2 / M_PI * 180;

        double tipDeltaAngle = tipTargetAngle - thelta2;
        lineEditTipTargetAngle = tipDeltaAngle / M_PI * 180;


        //translate the joint1,2 target angle to target endoer position
        double deltaAngle = thelta1 - jointMotorData[0].angle; //
        int cmd = (int)(-deltaAngle * jointMotorData[0].resolution);
        int targetPos = cmd + jointMotorData[0].encoderPos;
        if ((targetPos >= jointMotorData[0].limitPos1) && (targetPos <= jointMotorData[0].limitPos2))
        {
            if (!jointMotorData[0].protect)
            {
                if (ipTCP_1){
                        cmdStr = "!PR 1 " + toString(cmd) +"\r";
                        ipTCP_1->send(cmdStr);
                }
            }
        }

        deltaAngle = thelta2 - jointMotorData[1].angle;  //[1].
        cmd = -(int)(deltaAngle * jointMotorData[1].resolution);
        targetPos = cmd + jointMotorData[1].encoderPos;
        if ((targetPos >= jointMotorData[1].limitPos1) && (targetPos <= jointMotorData[1].limitPos2))
        {
            if (!jointMotorData[1].protect)
            {
                if (ipTCP_1){
                   
                        cmdStr = "!PR 2 " + toString(cmd) +"\r";

                        ipTCP_1->send(cmdStr);
                }
            }
        }
         //tip angle translate to joint 5 angle
        deltaAngle = tipDeltaAngle - JOINT_INI_ANGLE[5];
        cmd = (int)(deltaAngle * jointMotorData[5].resolution);
        targetPos = cmd + jointMotorData[5].iniPos;
        if ((targetPos >= jointMotorData[5].limitPos1) && (targetPos <= jointMotorData[5].limitPos2))
        {
            if (!jointMotorData[5].protect)
            {
                if (ipTCP_3){
                   
                        cmdStr = "!P 2 " + toString(targetPos) +"\r";

                        ipTCP_1->send(cmdStr);
                }
            }
        }
        for (int i = 0; i < 2; i++)
        {
            jointMotorData[i].stuckFlag = false;
            jointMotorData[i].protect = false;
            jointMotorData[i].manipulatorStuckCnt = 0;
        }
    }
    catch(...){

    }
}



void Arm:: armResetCmd()
{
    std::string cmdStr = "";
    for (int i = 0; i < 6; i++)
    {
        jointMotorData[i].protect = false;
        jointMotorData[i].stuckFlag = false;
        jointMotorData[i].manipulatorStuckCnt = 0;
        jointMotorData[i].jointJoy = false;
    }
    if (ipTCP_1){
            cmdStr = "!P 1 " + toString(jointMotorData[0].iniPos)
                    + "_!P 2 " + toString(jointMotorData[1].iniPos) +  "\r";
            ipTCP_1->send(cmdStr);
        
    }

    if (ipTCP_2){
        if (ipTCP_2){
            cmdStr = "!P 1 " + toString(jointMotorData[2].iniPos) + "\r";
            ipTCP_2->send(cmdStr);
        }
    }
    if (ipTCP_3 ){
        if (ipTCP_3){
            cmdStr = "!P 1 " + toString(jointMotorData[4].iniPos)
                    + "_!P 2 " + toString(jointMotorData[5].iniPos) +  "\r";
            ipTCP_3->send(cmdStr);
        }
    }
}

void Arm:: armSetIniCmd()
{
    int temp = 0;
    // if (pushButtonArmSetIni)
    // {
        // armIniFlag = true;
        // jointMotorData[0].iniPos = jointMotorData[0].encoderPos;
        // jointMotorData[1].iniPos = jointMotorData[1].encoderPos;
        // jointMotorData[2].iniPos = jointMotorData[2].encoderPos;
        // jointMotorData[3].iniPos = jointMotorData[3].encoderPos;
        // jointMotorData[4].iniPos = jointMotorData[4].encoderPos;
        // jointMotorData[5].iniPos = jointMotorData[5].encoderPos;
        // jointMotorData[0].angle = M_PI - M_PI * J1_OFFSET / 180;
        // jointMotorData[1].angle = M_PI - M_PI * J2_OFFSET / 180;
        // jointMotorData[2].angle = 0;
        // jointMotorData[4].angle = 0;
        // jointMotorData[5].angle = 0;
        // //set some limitation here
        // //joint1 half circle
        // jointMotorData[0].limitPos1 = jointMotorData[0].iniPos ;
        // temp  = (int)(jointMotorData[0].iniPos + jointMotorData [0].circleCnt/2);
        // jointMotorData[0].limitPos2 = temp;
        // //joint2 almost one circle
        // jointMotorData[1].limitPos2 = int(jointMotorData[1].iniPos + jointMotorData[1].circleCnt/2);
        // temp = jointMotorData[1].iniPos - int(jointMotorData[1].circleCnt / 2);
        // jointMotorData[1].limitPos1 = temp;

        // //joint3 rotate +/- one circle
        // temp = int(jointMotorData[2].iniPos + jointMotorData [2].circleCnt) ;
        // jointMotorData[2].limitPos2 = temp;
        // temp = int(jointMotorData[2].iniPos - jointMotorData[2].circleCnt);
        // jointMotorData[2].limitPos1 = temp;

        // //open/close clipper
        // temp = jointMotorData[3].iniPos + 5 * jointMotorData[3].circleCnt;
        // jointMotorData[3].limitPos2 = temp;
        // temp = jointMotorData[3].iniPos;
        // jointMotorData[3].limitPos1 = temp;

        // //joint5 pan
        // jointMotorData[4].limitPos1 = int(jointMotorData[4].iniPos - jointMotorData[4].circleCnt / 2);
        // jointMotorData[4].limitPos2 = int(jointMotorData[4].iniPos + jointMotorData[4].circleCnt / 2);


        // //joint6 tilt
        // jointMotorData[5].limitPos1 = int(jointMotorData[5].iniPos - jointMotorData[5].circleCnt / 2);
        // jointMotorData[5].limitPos2 = int(jointMotorData[5].iniPos + jointMotorData[5].circleCnt / 2);
        // print("armSetIniCmd is done");
        // pushButtonArmSetIni = false;
    // }
}
void Arm:: armReleaseInitial()
{
    int temp = 0;
    if (!pushButtonArmSetIni)
    {
        pushButtonArmSetIni = true;
        armIniFlag = false;
        print("armReleaseInitial is done");
    }
}
double Arm::get_JOINT_INI_ANGLE(int i)
{
    return JOINT_INI_ANGLE[i];
}
 int Arm::get_MOTOR_NUM(){
     return MOTOR_NUM;
 }
}// namespace jaguar_ns