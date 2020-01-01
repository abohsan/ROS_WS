
#include <string>
#include "../include/Nodes.hpp"

namespace jaguar_ns
{

int countt = 0;
double resTable[25] = {114660, 84510, 62927, 47077, 35563, 27119, 20860, 16204, 12683, 10000, 7942, 6327, 5074, 4103, 3336, 2724, 2237, 1846, 1530, 1275, 1068, 899.3, 760.7, 645.2, 549.4};
double tempTable[25] = {-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
double FULLAD = 4095;
int pub_periodic_count_2 = 0;
int pub_periodic_count_3 = 0;

Nodes::Nodes(const ros::NodeHandle &node_handle,
             const ros::NodeHandle &private_node_handle)
    : nh_(node_handle),
      pnh_(private_node_handle),
      pub_periodic_count_(0),
      sub1_callback_count_(0),
      sub2_callback_count_(0),
      sub3_callback_count_(0)
{
    jaguar = new Jaguar("192.168.0.60", 10001);
    conn = 0;
    isConnected = true;
    this->init();
}

void Nodes::init()
{
    imuInfo_pub_ = pnh_.advertise<jaguar::IMUInfo>("/jaguar_imu_sensor", 100);
    gpsInfo_pub_ = pnh_.advertise<jaguar::GPSInfo>("/jaguar_gps_sensor", 100);
    motorInfo_pub_ = pnh_.advertise<jaguar::MotorInfoArray>("/jaguar_motor_sensor", 1);
    motorboardInfoArray_pub_ = pnh_.advertise<jaguar::MotorBoardInfoArray>("/jaguar_motorboard_sensor", 1);

    keyboard_sub = pnh_.subscribe("/keyboard_command", 1, &Nodes::keyboard_Callback, this);
    cmd_vl_Jaguar = pnh_.subscribe("/cmd_vel", 1, &Nodes::moveWheels_callback, this);
    flipers_sub = pnh_.subscribe("/flipers", 1, &Nodes::flipers_sub_Callback, this);
    ping_timer = pnh_.createTimer(ros::Duration(0.3), &Nodes::ping_timer_callback, this);

    pub_timer = pnh_.createTimer(ros::Duration(0.03), &Nodes::pub_timer_callback, this);
}

void Nodes::ping_timer_callback(const ros::TimerEvent &event)
{
    jaguar->sendPing();
}
void Nodes::pub_timer_callback(const ros::TimerEvent &event)
{
    dealWithPackage(jaguar->read(), 0);
}

void Nodes::keyboard_Callback( const std_msgs::String::ConstPtr &msg)
{
    // ROS_INFO_STREAM("KeyPressed : " << msg->data);
    if (msg->data == "9")
    {
        jaguar->lightON();

        ROS_INFO_STREAM("ON ");
    }
    else if (msg->data == "0")
    {
        jaguar->lightOff();
        ROS_INFO_STREAM("off");
    }
    else if (msg->data == "z")
    {
        jaguar->releaseWheels();
        jaguar->releaseFrontFlipers();
        jaguar->releaseRearFlipers();
        ROS_INFO_STREAM("wheels Released");
    }
    else if (msg->data == "f")
    {
        jaguar->forward();
    }
    else if (msg->data == "b")
    {
        jaguar->backward();
    }
    else if (msg->data == "r")
    {
        jaguar->turnRight();
    }
    else if (msg->data == "l")
    {
        jaguar->turnLeft();
    }
    else if (msg->data == "s")
    {
        jaguar->stopWheels();
    }
    else if (msg->data == "f1u")
    {
        jaguar->moveFlipers_degree(10,10,10,10);
    }
    else if (msg->data == "f1d")
    {
       jaguar->moveFlipers_degree(-10,-10,-10,-10);
    }
    else if (msg->data == "f2u")
    {
        jaguar->go_To_Flipers_degree(0,0,0,0);
    }
    // else if (msg->data == "f2d")
    // {
    //     jaguar->moveFlipers_degree(0,-7000,0,0);
    // }
    // else if (msg->data == "f3u")
    // {
    //     jaguar->moveFlipers_degree(0,0,7000,0);
    // }
    // else if (msg->data == "f3d")
    // {
    //     jaguar->moveFlipers_degree(0,0,-7000,0);
    // }
    // else if (msg->data == "f5u")
    // {
    //     jaguar->moveFlipers_degree(0,0,0,7000);
    // }
    // else if (msg->data == "f5d")
    // {
    //     jaguar->moveFlipers_degree(0,0,0,-7000);
    // }
    else
    {
        ROS_INFO_STREAM("KeyPressed : " << msg->data);
    }

    ros::Rate loop_rate(50);
    // sleep for 200ms
    loop_rate.sleep();
}

void Nodes::moveWheels_callback(const geometry_msgs::Twist::ConstPtr &msg)
{

    // ROS_INFO_STREAM(" moveWheels_callback " << conn++);
    x = int(((msg->linear.x) / 2) * 1000);
    z = int(((msg->angular.z) / 2) * 1000);
    cmdValue1 = -(x - z);
    cmdValue2 = (x + z);

    jaguar->moveWheels(cmdValue1, cmdValue2);
    ros::Rate loop_rate(50);
    // sleep for 200ms
    loop_rate.sleep();
}
void Nodes::flipers_sub_Callback(const jaguar::FlipMotor::ConstPtr &msg)
{
    jaguar->moveFlipers_degree(msg->rightFront, msg->leftFront, msg->rightRear, msg->leftRear);
    jaguar->go_To_Flipers_degree(msg->go_to_rightFront, msg->go_to_leftFront, msg->go_to_rightRear, msg->go_to_leftRear);

    
    ros::Rate loop_rate(5);
    // sleep for 200ms
    loop_rate.sleep();
}

void Nodes::publisherIMUData(IMUData imuData)
{
    jaguar::IMUInfo imuInfo;
    imuInfo.header.stamp = ros::Time::now();
    imuInfo.header.frame_id = "/jaguar_imu";
    imuInfo.seq = imuData.seqNo;  //0 ~ 255
    imuInfo.yaw = imuData.estYaw; //radian
    imuInfo.pitch = 0;            //not used now
    imuInfo.roll = 0;             //not used now
    imuInfo.gyro_x = imuData.gyroRaw[0];
    imuInfo.gyro_y = imuData.gyroRaw[1];
    imuInfo.gyro_z = imuData.gyroRaw[2];
    imuInfo.accel_x = imuData.accelRaw[0];
    imuInfo.accel_y = imuData.accelRaw[1];
    imuInfo.accel_z = imuData.accelRaw[2];
    imuInfo.comp_x = imuData.compassRaw[0];
    imuInfo.comp_y = imuData.compassRaw[1];
    imuInfo.comp_z = imuData.compassRaw[2];
    imuInfo_pub_.publish(imuInfo);
}

void Nodes::publisherGPSInfo(GPSData gpsData)
{
    jaguar::GPSInfo gpsInfo;
    gpsInfo.header.stamp = ros::Time::now();
    gpsInfo.header.frame_id = "/jaguar_gps";
    gpsInfo.status = gpsData.gpsState;
    gpsInfo.gpsTimeStamp = gpsData.gpsTimeStamp;
    gpsInfo.latitude = gpsData.gpsLat; //
    gpsInfo.longitude = gpsData.gpsLong;
    gpsInfo.vog = gpsData.gpsVog;
    gpsInfo.cog = gpsData.gpsCog;
    gpsInfo_pub_.publish(gpsInfo);
}
void Nodes::processRobotData()
{
    int count = 0;
    // char revData[512];
    // watchDogCnt = 0;
    // do {
    // count = m_tcpRobot->readLine(revData,512);
    // dealWithPackage(QString::fromUtf8(revData),int(count));
    dealWithPackage(jaguar->read(), int(count));
    // } while ( m_tcpRobot->canReadLine() );
}
double Nodes::ad2Temperature(int adValue)
{
    //for new temperature sensor
    double tempM = 0;
    double k = (adValue / FULLAD);
    double resValue = 0;
    if (k != 1.0)
    {
        resValue = 10000 * k / (1 - k); //AD value to resistor
    }
    else
    {
        resValue = resTable[0];
    }

    int index = -1;
    if (resValue >= resTable[0]) //too lower
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

void Nodes::getFrontFlipAngle()
{
    int deltaEncoder = 0;
    if (flipArmMotor[0].iniFlag)
    { //leftFront
        deltaEncoder = flipArmMotor[0].encoderPos - flipArmMotor[0].preEncoder;
        flipArmMotor[0].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT) / FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[0].angle;
        if (flipArmMotor[0].angle > M_PI)
            flipArmMotor[0].angle = -(2 * M_PI - flipArmMotor[0].angle);
        if (flipArmMotor[0].angle < -M_PI)
            flipArmMotor[0].angle = (2 * M_PI + flipArmMotor[0].angle);
        flipArmMotor[0].preEncoder = flipArmMotor[0].encoderPos;
    }

    if (flipArmMotor[1].iniFlag)
    { //rightFront
        deltaEncoder = flipArmMotor[1].encoderPos - flipArmMotor[1].preEncoder;
        flipArmMotor[1].angle = (double)(-deltaEncoder % FLIPARM_CIRCLE_CNT) / FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[1].angle;
        if (flipArmMotor[1].angle > M_PI)
            flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[1].angle);
        if (flipArmMotor[1].angle < -M_PI)
            flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[1].angle);
        flipArmMotor[1].preEncoder = flipArmMotor[1].encoderPos;
    }
}

void Nodes::getRearFlipAngle()
{
    int deltaEncoder = 0;
    if (flipArmMotor[2].iniFlag)
    { //leftRear
        deltaEncoder = flipArmMotor[2].encoderPos - flipArmMotor[2].preEncoder;
        flipArmMotor[2].angle = (double)(-deltaEncoder % FLIPARM_CIRCLE_CNT) / FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[2].angle;
        if (flipArmMotor[2].angle > M_PI)
            flipArmMotor[2].angle = -(2 * M_PI - flipArmMotor[2].angle);
        if (flipArmMotor[2].angle < -M_PI)
            flipArmMotor[2].angle = (2 * M_PI + flipArmMotor[2].angle);
        flipArmMotor[2].preEncoder = flipArmMotor[2].encoderPos;
    }

    if (flipArmMotor[3].iniFlag)
    { //rightRear
        deltaEncoder = flipArmMotor[3].encoderPos - flipArmMotor[3].preEncoder;
        flipArmMotor[3].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT) / FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[3].angle;
        if (flipArmMotor[3].angle > M_PI)
            flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[3].angle);
        if (flipArmMotor[3].angle < -M_PI)
            flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[3].angle);
        flipArmMotor[3].preEncoder = flipArmMotor[3].encoderPos;
    }
}

void Nodes::publisherMotorBoardInfoArray(MotorBoardData motorBoardData[], int len)
{
    jaguar::MotorBoardInfoArray motorBoardInfoArray;
    motorBoardInfoArray.motorBoardInfo.resize(len);
    for (uint32_t i = 0; i < len; i++)
    {

        motorBoardInfoArray.motorBoardInfo[i].header.stamp = ros::Time::now();
        motorBoardInfoArray.motorBoardInfo[i].header.frame_id = "/jaguar_motorboard";
        motorBoardInfoArray.motorBoardInfo[i].status = motorBoardData[i].driverState;
        motorBoardInfoArray.motorBoardInfo[i].temp1 = 0;                              // temperature 1, internal chip temperature, no used now
        motorBoardInfoArray.motorBoardInfo[i].temp2 = motorBoardData[i].ch1Temp;      // driver channel 1 temperature
        motorBoardInfoArray.motorBoardInfo[i].temp3 = motorBoardData[i].ch2Temp;      // driver channel 2 temperature
        motorBoardInfoArray.motorBoardInfo[i].volMain = motorBoardData[i].drvVoltage; //main power voltage, default is battery voltage
        motorBoardInfoArray.motorBoardInfo[i].vol12V = motorBoardData[i].motVoltage;
        motorBoardInfoArray.motorBoardInfo[i].vol5V = motorBoardData[i].reg5Voltage; // 5V power
        motorBoardInfoArray.motorBoardInfo[i].dinput = 0;                            // digital input, not used now
        motorBoardInfoArray.motorBoardInfo[i].doutput = 0;                           // digital output, not used now
        motorBoardInfoArray.motorBoardInfo[i].ack = 0;                               //not used now 0- right command received(receive "+") -1 wrong("-")
    }
    motorboardInfoArray_pub_.publish(motorBoardInfoArray);
}

void Nodes::publisherMotorData(MotorData motorData[], int len)
{
    jaguar::MotorInfoArray motorInfoArray;
    motorInfoArray.motorInfo.resize(len);
    for (uint32_t i = 0; i < len; ++i)
    {

        motorInfoArray.motorInfo[i].header.stamp = ros::Time::now();
        motorInfoArray.motorInfo[i].header.frame_id = "/jaguar_motor";
        motorInfoArray.motorInfo[i].encoderPos = motorData[i].encoderPos;
        motorInfoArray.motorInfo[i].encoderVel = motorData[i].encoderSpeed;
        motorInfoArray.motorInfo[i].motorPower = motorData[i].motorPower;
        motorInfoArray.motorInfo[i].motorTemp = motorData[i].motorTemp;   // motor temperature reading
        motorInfoArray.motorInfo[i].motorCurrent = motorData[i].motorAmp; //motor current feedback reading
        motorInfoArray.motorInfo[i].encoderDiff = 0;                      //not used now
    }

    // ROS_INFO("publish motor info array");
    motorInfo_pub_.publish(motorInfoArray);
}


void Nodes::dealWithPackage(std::string revData, int len)
{

    // int index;
    // std::string prefix_1 = "#";      // IMU
    // std::string prefix_2 = "$GPRMC"; //GPS
    // std::vector<std::string> data;
    // if (startWith(revData, prefix_1))
    // {
    //     //IMU sensor data package
    //     data = split(revData, prefix_1, ",");
    //     if (data.size() > 15)
    //     {
    //         imuData.seqNo = std::stoi(data.at(0));
    //         imuData.estYaw = std::stod(data.at(2));
    //         imuData.gyroRaw[0] = std::stoi(data.at(4));
    //         imuData.gyroRaw[1] = std::stoi(data.at(5));
    //         imuData.gyroRaw[2] = std::stoi(data.at(6));
    //         imuData.accelRaw[0] = std::stoi(data.at(8));
    //         imuData.accelRaw[1] = std::stoi(data.at(9));
    //         imuData.accelRaw[2] = std::stoi(data.at(10));
    //         imuData.compassRaw[0] = std::stoi(data.at(12));
    //         imuData.compassRaw[1] = std::stoi(data.at(13));
    //         imuData.compassRaw[2] = std::stoi(data.at(14));

    //         publisherIMUData(imuData);
    //     }
    // }
    // else if (startWith(revData, prefix_2))
    // {
    //     // GPS 
    //     data = split(revData, prefix_2, ",");
    //     if (data.size() > 9)
    //     {
    //         gpsData.gpsTimeStamp = std::stod(data.at(1));
    //         //            ui->gpsTimeStampLineEdit->setText(data[1]);
    //         if (data[2] == "A")
    //         {
    //             gpsData.gpsState = 1;
    //         }
    //         else if (data[2] == "V")
    //         {
    //             gpsData.gpsState = 0;
    //         }
    //         if (gpsData.gpsState > 0)
    //         {
    //             //                ui->gpsStateLineEdit->setText("Valid");
    //         }
    //         else
    //         {
    //             //                ui->gpsStateLineEdit->setText("InValid");
    //         }
    //         gpsData.gpsLat = std::stod(data.at(3));
    //         if (data[4] == "S")
    //         {
    //             gpsData.gpsLat = -gpsData.gpsLat;
    //         }
    //         //            ui->gpsLatLineEdit->setText(data[4] + ":" + data[3]);
    //         gpsData.gpsLong = std::stod(data.at(5));
    //         if (data[6] == "W")
    //         {
    //             gpsData.gpsLong = -gpsData.gpsLong;
    //         }
    //         //            ui->gpsLongLineEdit->setText(data[6] + ":" + data[5]);
    //         if (!data.at(7).empty())
    //         {
    //             gpsData.gpsVog = std::stod(data.at(7)) * KNNOT2MS;
    //             //                ui->gpsVogLineEdit->setText(QString::number(gpsData.gpsVog,'f',2));
    //         }
    //         if (!data[8].empty())
    //         {
    //             gpsData.gpsCog = std::stod(data.at(8));
    //             //                ui->gpsCogLineEdit->setText(data[8]);
    //         }

    //         publisherGPSInfo(gpsData);
    //     }
    // }
    // else if (startWith(revData, "MM"))
    // {
    //   // motor and driver board data package
    //     if (startWith(revData, "MM0"))
    //     {
    //         index = 0;
    //     }
    //     else if (startWith(revData, "MM1"))
    //     {
    //         index = 1;
    //     }
    //     else if (startWith(revData, "MM2"))
    //     {
    //         index = 2;
    //     }
    //     else if (startWith(revData, "MM3"))
    //     {
    //         index = 3;
    //     }
    //     //driver 1 and front motors
    //     revData = revData.erase(0, 4);

    //     if (startWith(revData, "A="))
    //     {
          
    //         //current data
    //         revData.erase(0, 2);

    //         data = split(revData, ":");
    //         motorData[index * 2 + 0].motorAmp = std::stod(data.at(0)) / 10;
    //         motorData[index * 2 + 1].motorAmp = std::stod(data.at(1)) / 10;
    //     }
        //     else if (startWith(revData, "AI="))
        //     {
        //         // A/D data, here 3,4 will be motor temperature sensor
        //         revData.erase(0, 3);
        //         data = split(revData, ":");
        //         motorData[index * 2 + 0].motorTemp = ad2Temperature(std::stoi(data[2]));
        //         motorData[index * 2 + 1].motorTemp = ad2Temperature(std::stod(data[3]));
        //     }
        //     else if (startWith(revData, "C="))
        //     {
        //         // encoder position data
        //         revData.erase(0, 2);
        //         data = split(revData, ":");
        //         motorData[index * 2 + 0].encoderPos = std::stoi(data[0]);
        //         motorData[index * 2 + 1].encoderPos = std::stoi(data[1]);
        //         if (index == 2)
        //         {
        //             flipArmMotor[0].encoderPos = motorData[4].encoderPos;
        //             flipArmMotor[1].encoderPos = motorData[5].encoderPos;
        //             getFrontFlipAngle();
        //         }
        //         else if (index == 3)
        //         {
        //             flipArmMotor[2].encoderPos = motorData[6].encoderPos;
        //             flipArmMotor[3].encoderPos = motorData[7].encoderPos;
        //             getRearFlipAngle();
        //         }
        //     }
        //     else if (startWith(revData, "P="))
        //     {
        //         // output PWM value, 0 ~ 1000
        //         revData.erase(0, 2);
        //         data = split(revData, ":");
        //         motorData[index * 2 + 0].motorPower = std::stoi(data[0]);
        //         motorData[index * 2 + 1].motorPower = std::stoi(data[1]);
        //     }
        //     else if (startWith(revData, "S="))
        //     {
        //         // encoder velocity data RPM
        //         revData.erase(0, 2);
        //         data = split(revData, ":");
        //         motorData[index * 2 + 0].encoderSpeed = std::stoi(data[0]);
        //         motorData[index * 2 + 1].encoderSpeed = std::stoi(data[1]);
        //     }
        //     else if (startWith(revData, "T="))
        //     {
        //         // motor driver board temperature
        //         revData = revData.erase(0, 2);
        //         data = split(revData, ":");
        //         motorBoardData[index].ch1Temp = std::stoi(data[0]);
        //         motorBoardData[index].ch2Temp = std::stoi(data[1]);
        //     }
        //     else if (startWith(revData, "V="))
        //     {
        //         // voltage data
        //         revData = revData.erase(0, 2);
        //         data = split(revData, ":");
        //         motorBoardData[index].drvVoltage = std::stod(data[0]) / 10;
        //         motorBoardData[index].motVoltage = std::stod(data[1]) / 10;
        //         motorBoardData[index].reg5Voltage = std::stod(data[2]) / 1000;
        //     }
        //     else if (startWith(revData, "CR="))
        //     {
        //         // here is the encoder relative difference reading,
        //         // very useful to estimate the encoder/motor traveling distance
        //     }
        //     else if (startWith(revData, "FF="))
        //     {
        //         // driver board state
        //         revData = revData.erase(0, 3);
        //         // motorBoardData[index].driverState = std::stoi(revData);
        //     }

        //        if (index == 0)
        //        {
        //            ui->lfmPosLineEdit->setText(QString::number(motorData[0].encoderPos));
        //            ui->lfmCurrentLineEdit->setText(QString::number(motorData[0].motorAmp,'f',2));
        //            ui->lfmPWMLineEdit->setText(QString::number(motorData[0].motorPower));
        //            ui->lfmTempLineEdit->setText(QString::number(motorData[0].motorTemp,'f',2));
        //            ui->lfmVelLineEdit->setText(QString::number(motorData[0].encoderSpeed));
        //
        //            ui->motorVolLineEdit->setText(QString::number(motorBoardData[0].motVoltage,'f',2));
        //            ui->rfmPosLineEdit->setText(QString::number(motorData[1].encoderPos));
        //            ui->rfmCurrentLineEdit->setText(QString::number(motorData[1].motorAmp,'f',2));
        //            ui->rfmPWMLineEdit->setText(QString::number(motorData[1].motorPower));
        //            ui->rfmTempLineEdit->setText(QString::number(motorData[1].motorTemp,'f',2));
        //            ui->rfmVelLineEdit->setText(QString::number(motorData[1].encoderSpeed));
        //
        //        }
        //        else if(index == 1)
        //        {
        //            ui->lrmPosLineEdit->setText(QString::number(motorData[2].encoderPos));
        //            ui->lrmCurrentLineEdit->setText(QString::number(motorData[2].motorAmp,'f',2));
        //            ui->lrmPWMLineEdit->setText(QString::number(motorData[2].motorPower));
        //            ui->lrmTempLineEdit->setText(QString::number(motorData[2].motorTemp,'f',2));
        //            ui->lrmVelLineEdit->setText(QString::number(motorData[2].encoderSpeed));
        //
        //            ui->rrmPosLineEdit->setText(QString::number(motorData[3].encoderPos));
        //            ui->rrmCurrentLineEdit->setText(QString::number(motorData[3].motorAmp,'f',2));
        //            ui->rrmPWMLineEdit->setText(QString::number(motorData[3].motorPower));
        //            ui->rrmTempLineEdit->setText(QString::number(motorData[3].motorTemp,'f',2));
        //            ui->rrmVelLineEdit->setText(QString::number(motorData[3].encoderSpeed));
        //        }
        //        else if(index == 2)
        //        {
        //            ui->lffmPosLineEdit->setText(QString::number(motorData[4].encoderPos));
        //            ui->lffmCurrentLineEdit->setText(QString::number(motorData[4].motorAmp,'f',2));
        //            ui->lffmPWMLineEdit->setText(QString::number(motorData[4].motorPower));
        //            ui->lffmTempLineEdit->setText(QString::number(motorData[4].motorTemp,'f',2));
        //            ui->lffmVelLineEdit->setText(QString::number(motorData[4].encoderSpeed));
        //
        //            ui->rffmPosLineEdit->setText(QString::number(motorData[5].encoderPos));
        //            ui->rffmCurrentLineEdit->setText(QString::number(motorData[5].motorAmp,'f',2));
        //            ui->rffmPWMLineEdit->setText(QString::number(motorData[5].motorPower));
        //            ui->rffmTempLineEdit->setText(QString::number(motorData[5].motorTemp,'f',2));
        //            ui->rffmVelLineEdit->setText(QString::number(motorData[5].encoderSpeed));
        //        }
        //        else if(index == 3)
        //        {
        //            ui->lrfmPosLineEdit->setText(QString::number(motorData[6].encoderPos));
        //            ui->lrfmCurrentLineEdit->setText(QString::number(motorData[6].motorAmp,'f',2));
        //            ui->lrfmPWMLineEdit->setText(QString::number(motorData[6].motorPower));
        //            ui->lrfmTempLineEdit->setText(QString::number(motorData[6].motorTemp,'f',2));
        //            ui->lrfmVelLineEdit->setText(QString::number(motorData[6].encoderSpeed));
        //
        //            ui->rrfmPosLineEdit->setText(QString::number(motorData[7].encoderPos));
        //            ui->rrfmCurrentLineEdit->setText(QString::number(motorData[7].motorAmp,'f',2));
        //            ui->rrfmPWMLineEdit->setText(QString::number(motorData[7].motorPower));
        //            ui->rrfmTempLineEdit->setText(QString::number(motorData[7].motorTemp,'f',2));
        //            ui->rrfmVelLineEdit->setText(QString::number(motorData[7].encoderSpeed));
        //        }

        //     std::string strError = "";
        //     for (int i = 0; i < 4; i++)
        //     {
        //         strError.clear();
        //         if ((motorBoardData[i].driverState & 0x1) != 0)
        //         {
        //             strError = "OH";
        //         }

        //         if ((motorBoardData[i].driverState & 0x2) != 0)
        //         {
        //             strError += "OV";
        //         }
        //         if ((motorBoardData[i].driverState & 0x4) != 0)
        //         {
        //             strError += "UV";
        //         }
        //         if ((motorBoardData[i].driverState & 0x8) != 0)
        //         {
        //             strError += "SHT";
        //         }
        //         if ((motorBoardData[i].driverState & 0x10) != 0)
        //         {
        //             strError += "ESTOP";
        //         }
        //         if ((motorBoardData[i].driverState & 0x20) != 0)
        //         {
        //             strError += "SEPF";
        //         }
        //         if ((motorBoardData[i].driverState & 0x40) != 0)
        //         {
        //             strError += "PromF";
        //         }
        //         if ((motorBoardData[i].driverState & 0x80) != 0)
        //         {
        //             strError += "ConfF";
        //         }
        //         if (strError.length() < 1)
        //         {
        //             strError = "OK";
        //         }
        //         //            if (i == 0){
        //         //                ui->driver1StateLineEdit->setText(strError);
        //         //            }
        //         //            else if (i == 1){
        //         //                ui->driver2StateLineEdit->setText(strError);
        //         //            }
        //         //            else if (i == 2){
        //         //                ui->driver3StateLineEdit->setText(strError);
        //         //            }
        //         //            else if (i == 3){
        //         //                ui->driver4StateLineEdit->setText(strError);
        //         //            }
        // }
        //     //publish sensor here
    //         publisherMotorData(motorData, 8);
    //         publisherMotorBoardInfoArray(motorBoardData, 4);
    // }
} // namespace jaguar
} // namespace jaguar_ns