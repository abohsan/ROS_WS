
#include <string>
#include "../include/Nodes.hpp"

namespace jaguar_ns
{
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
     imuInfo_pub_ = pnh_.advertise<jaguar::IMUInfo>("/jaguar_imu_sensor",100);
     gpsInfo_pub_ = pnh_.advertise<jaguar::GPSInfo>("/jaguar_gps_sensor",100);

    keyboard_sub = pnh_.subscribe("/keyboard_command", 1,  &Nodes::keyboard_Callback, this);
    cmd_vl_Jaguar = pnh_.subscribe("/cmd_vel", 1, &Nodes::moveWheels_callback,  this);
    sub3_ = pnh_.subscribe("/subscriber_2", 1, &Nodes::subscriberCallback3,this);
    ping_timer = pnh_.createTimer(ros::Duration(0.3),   &Nodes::ping_timer_callback, this);


    pub_timer = pnh_.createTimer(ros::Duration(0.03),   &Nodes::pub_timer_callback, this);
}


void Nodes::ping_timer_callback(const ros::TimerEvent &event)
{
    jaguar->sendPing();
}
void Nodes::pub_timer_callback(const ros::TimerEvent &event)
{
    dealWithPackage(jaguar->read(),0); 
}

void Nodes::keyboard_Callback(
    const std_msgs::String::ConstPtr &msg)
{
    // ROS_INFO_STREAM("KeyPressed : " << msg->data);
   if (msg->data == "1")
    {
        jaguar->lightON();
        
        ROS_INFO_STREAM("ON ");
    }
    else if (msg->data == "0")
    {
       jaguar->lightOff();
        ROS_INFO_STREAM("off");
    }else if (msg->data == "z")
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
    x =  int(((msg->linear.x)/2)* 1000);
    z =  int(((msg->angular.z)/2)* 1000);
    cmdValue1 =  -(x - z);
    cmdValue2 =   (x + z);
    
    jaguar->moveWheels(cmdValue1,cmdValue2);
    ros::Rate loop_rate(50);
    // sleep for 200ms
    loop_rate.sleep();
}
void Nodes::subscriberCallback3(
    const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Third subscriber callback "
                    << sub3_callback_count_ << ", in thread:"
                    << boost::this_thread::get_id());
    sub3_callback_count_++;
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
  imuInfo.yaw = imuData.estYaw;	//radian
  imuInfo.pitch = 0; //not used now
  imuInfo.roll = 0; //not used now
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
  gpsInfo.latitude = gpsData.gpsLat;  //
  gpsInfo.longitude = gpsData.gpsLong;
  gpsInfo.vog = gpsData.gpsVog;
  gpsInfo.cog = gpsData.gpsCog;
  gpsInfo_pub_.publish(gpsInfo);
}
void Nodes::processRobotData() {
    int count = 0;
    // char revData[512];
    // watchDogCnt = 0;
    // do {
        // count = m_tcpRobot->readLine(revData,512);
        // dealWithPackage(QString::fromUtf8(revData),int(count));
        dealWithPackage(jaguar->read(),int(count));
    // } while ( m_tcpRobot->canReadLine() );
}

void Nodes:: dealWithPackage(std::string revData, int len)
{
    int index;
    std::string prefix_1 = "#"; // IMU
    std::string prefix_2 = "$GPRMC"; //GPS
    // std::string prefix_3 = "#";
    // std::string prefix_4 = "#";
    std::vector<std::string> data;

    // if (revData.startsWith("#")){
    // if (revData.substr(0, prefix_1.size()) == prefix_1){
     if (startWith(revData, prefix_1) ){
        //IMU sensor data package
          data = split(revData,prefix_1,",");
        if (data.size() > 15){
            imuData.seqNo = std::stoi(data.at(0));
            imuData.estYaw = std::stod(data.at(2));
            imuData.gyroRaw[0] = std::stoi(data.at(4));
            imuData.gyroRaw[1] =std::stoi(data.at(5));
            imuData.gyroRaw[2] = std::stoi(data.at(6));
            imuData.accelRaw[0] = std::stoi(data.at(8));
            imuData.accelRaw[1] = std::stoi(data.at(9));
            imuData.accelRaw[2] = std::stoi(data.at(10));
            imuData.compassRaw[0] = std::stoi(data.at(12));
            imuData.compassRaw[1] = std::stoi(data.at(13));
            imuData.compassRaw[2] = std::stoi(data.at(14));

           publisherIMUData(imuData);
        }
    }
    else if(startWith(revData,prefix_2)){
        //GPS sensor data package
          data = split(revData,prefix_2,",");
        if (data.size() > 9){
            gpsData.gpsTimeStamp = std::stod(data.at(1));
//            ui->gpsTimeStampLineEdit->setText(data[1]);
            if (data[2] == "A"){
                gpsData.gpsState = 1;
            }
            else if(data[2] == "V"){
                gpsData.gpsState = 0;
            }
            if (gpsData.gpsState >0){
//                ui->gpsStateLineEdit->setText("Valid");
            }
            else{
//                ui->gpsStateLineEdit->setText("InValid");
            }
            gpsData.gpsLat = std::stod(data.at(3));
            if (data[4] == "S"){
                gpsData.gpsLat = -gpsData.gpsLat;
            }
//            ui->gpsLatLineEdit->setText(data[4] + ":" + data[3]);
            gpsData.gpsLong = std::stod(data.at(5));
            if (data[6] == "W"){
                gpsData.gpsLong = -gpsData.gpsLong;
            }
//            ui->gpsLongLineEdit->setText(data[6] + ":" + data[5]);
            if (!data.at(7).empty()){
                gpsData.gpsVog = std::stod(data.at(7)) * KNNOT2MS;
//                ui->gpsVogLineEdit->setText(QString::number(gpsData.gpsVog,'f',2));
            }
            if (!data[8].empty()){
                gpsData.gpsCog = std::stod(data.at(8));
//                ui->gpsCogLineEdit->setText(data[8]);
            }

           publisherGPSInfo(gpsData);

        }
    }
//     else if(revData.startsWith("MM")){
//         // motor and driver board data package
//         if (revData.startsWith("MM0")){
//             index = 0;
//         }
//         else if (revData.startsWith("MM1")){
//             index = 1;
//         }
//         else if (revData.startsWith("MM2")){
//             index = 2;
//         }
//         else if (revData.startsWith("MM3")){
//             index = 3;
//         }
//                         //driver 1 and front motors
//         revData = revData.remove(0,4);

//         if (revData.startsWith("A=")){
//             //current data
//             revData = revData.remove(0,2);
//             QStringList data = revData.split(":");
//             motorData[index * 2+ 0].motorAmp = data[0].toDouble()/10;
//             motorData[index * 2+ 1].motorAmp = data[1].toDouble()/10;
//         }
//         else if(revData.startsWith("AI=")){
//             // A/D data, here 3,4 will be motor temperature sensor
//             revData = revData.remove(0,3);
//             QStringList data = revData.split(":");
//             motorData[index * 2+ 0].motorTemp = ad2Temperature(data[2].toInt());
//             motorData[index * 2+ 1].motorTemp = ad2Temperature(data[3].toInt());
//         }
//         else if(revData.startsWith("C=")){
//             // encoder position data
//             revData = revData.remove(0,2);
//             QStringList data = revData.split(":");
//             motorData[index * 2+ 0].encoderPos = data[0].toInt();
//             motorData[index * 2+ 1].encoderPos = data[1].toInt();
//             if (index == 2){
//                 flipArmMotor[0].encoderPos = motorData[4].encoderPos;
//                 flipArmMotor[1].encoderPos = motorData[5].encoderPos;
//                 getFrontFlipAngle();
//             }
//             else if(index == 3){
//                 flipArmMotor[2].encoderPos = motorData[6].encoderPos;
//                 flipArmMotor[3].encoderPos = motorData[7].encoderPos;
//                 getRearFlipAngle();
//             }
//         }
//         else if(revData.startsWith("P=")){
//             // output PWM value, 0 ~ 1000
//             revData = revData.remove(0,2);
//             QStringList data = revData.split(":");
//             motorData[index * 2+ 0].motorPower = data[0].toInt();
//             motorData[index * 2+ 1].motorPower = data[1].toInt();

//         }
//         else if(revData.startsWith("S=")){
//             // encoder velocity data RPM
//             revData = revData.remove(0,2);
//              QStringList data = revData.split(":");
//             motorData[index * 2+ 0].encoderSpeed = data[0].toInt();
//             motorData[index * 2+ 1].encoderSpeed = data[1].toInt();

//         }
//         else if(revData.startsWith("T=")){
//             // motor driver board temperature
//             revData = revData.remove(0,2);
//              QStringList data = revData.split(":");
//             motorBoardData[index].ch1Temp = data[0].toDouble();
//             motorBoardData[index].ch2Temp = data[1].toDouble();
//         }
//         else if(revData.startsWith("V=")){
//             // voltage data
//             revData = revData.remove(0,2);
//             QStringList data = revData.split(":");
//             motorBoardData[index].drvVoltage = data[0].toDouble()/10;
//             motorBoardData[index].motVoltage = data[1].toDouble()/10;
//             motorBoardData[index].reg5Voltage = data[2].toDouble() /1000;

//         }
//         else if(revData.startsWith("CR=")){
//             // here is the encoder relative difference reading,
//             // very useful to estimate the encoder/motor traveling distance
//         }
//         else if(revData.startsWith("FF=")){
//             // driver board state
//             revData = revData.remove(0,3);
//             motorBoardData[index].driverState = revData.toInt();
//         }

// //        if (index == 0)
// //        {
// //            ui->lfmPosLineEdit->setText(QString::number(motorData[0].encoderPos));
// //            ui->lfmCurrentLineEdit->setText(QString::number(motorData[0].motorAmp,'f',2));
// //            ui->lfmPWMLineEdit->setText(QString::number(motorData[0].motorPower));
// //            ui->lfmTempLineEdit->setText(QString::number(motorData[0].motorTemp,'f',2));
// //            ui->lfmVelLineEdit->setText(QString::number(motorData[0].encoderSpeed));
// //
// //            ui->motorVolLineEdit->setText(QString::number(motorBoardData[0].motVoltage,'f',2));
// //            ui->rfmPosLineEdit->setText(QString::number(motorData[1].encoderPos));
// //            ui->rfmCurrentLineEdit->setText(QString::number(motorData[1].motorAmp,'f',2));
// //            ui->rfmPWMLineEdit->setText(QString::number(motorData[1].motorPower));
// //            ui->rfmTempLineEdit->setText(QString::number(motorData[1].motorTemp,'f',2));
// //            ui->rfmVelLineEdit->setText(QString::number(motorData[1].encoderSpeed));
// //
// //        }
// //        else if(index == 1)
// //        {
// //            ui->lrmPosLineEdit->setText(QString::number(motorData[2].encoderPos));
// //            ui->lrmCurrentLineEdit->setText(QString::number(motorData[2].motorAmp,'f',2));
// //            ui->lrmPWMLineEdit->setText(QString::number(motorData[2].motorPower));
// //            ui->lrmTempLineEdit->setText(QString::number(motorData[2].motorTemp,'f',2));
// //            ui->lrmVelLineEdit->setText(QString::number(motorData[2].encoderSpeed));
// //
// //            ui->rrmPosLineEdit->setText(QString::number(motorData[3].encoderPos));
// //            ui->rrmCurrentLineEdit->setText(QString::number(motorData[3].motorAmp,'f',2));
// //            ui->rrmPWMLineEdit->setText(QString::number(motorData[3].motorPower));
// //            ui->rrmTempLineEdit->setText(QString::number(motorData[3].motorTemp,'f',2));
// //            ui->rrmVelLineEdit->setText(QString::number(motorData[3].encoderSpeed));
// //        }
// //        else if(index == 2)
// //        {
// //            ui->lffmPosLineEdit->setText(QString::number(motorData[4].encoderPos));
// //            ui->lffmCurrentLineEdit->setText(QString::number(motorData[4].motorAmp,'f',2));
// //            ui->lffmPWMLineEdit->setText(QString::number(motorData[4].motorPower));
// //            ui->lffmTempLineEdit->setText(QString::number(motorData[4].motorTemp,'f',2));
// //            ui->lffmVelLineEdit->setText(QString::number(motorData[4].encoderSpeed));
// //
// //            ui->rffmPosLineEdit->setText(QString::number(motorData[5].encoderPos));
// //            ui->rffmCurrentLineEdit->setText(QString::number(motorData[5].motorAmp,'f',2));
// //            ui->rffmPWMLineEdit->setText(QString::number(motorData[5].motorPower));
// //            ui->rffmTempLineEdit->setText(QString::number(motorData[5].motorTemp,'f',2));
// //            ui->rffmVelLineEdit->setText(QString::number(motorData[5].encoderSpeed));
// //        }
// //        else if(index == 3)
// //        {
// //            ui->lrfmPosLineEdit->setText(QString::number(motorData[6].encoderPos));
// //            ui->lrfmCurrentLineEdit->setText(QString::number(motorData[6].motorAmp,'f',2));
// //            ui->lrfmPWMLineEdit->setText(QString::number(motorData[6].motorPower));
// //            ui->lrfmTempLineEdit->setText(QString::number(motorData[6].motorTemp,'f',2));
// //            ui->lrfmVelLineEdit->setText(QString::number(motorData[6].encoderSpeed));
// //
// //            ui->rrfmPosLineEdit->setText(QString::number(motorData[7].encoderPos));
// //            ui->rrfmCurrentLineEdit->setText(QString::number(motorData[7].motorAmp,'f',2));
// //            ui->rrfmPWMLineEdit->setText(QString::number(motorData[7].motorPower));
// //            ui->rrfmTempLineEdit->setText(QString::number(motorData[7].motorTemp,'f',2));
// //            ui->rrfmVelLineEdit->setText(QString::number(motorData[7].encoderSpeed));
// //        }



//         QString strError ="";
//         for (int i = 0; i < 4; i++){
//             strError.clear();
//             if ((motorBoardData[i].driverState & 0x1) != 0){
//                 strError = "OH";
//             }

//             if ((motorBoardData[i].driverState & 0x2) != 0){
//                 strError += "OV";
//             }
//             if ((motorBoardData[i].driverState & 0x4) != 0){
//                 strError += "UV";
//             }
//             if ((motorBoardData[i].driverState & 0x8) != 0){
//                 strError += "SHT";
//             }
//             if ((motorBoardData[i].driverState & 0x10) != 0){
//                 strError += "ESTOP";
//             }
//             if ((motorBoardData[i].driverState & 0x20) != 0){
//                 strError += "SEPF";
//             }
//             if ((motorBoardData[i].driverState & 0x40) != 0){
//                 strError += "PromF";
//             }
//             if ((motorBoardData[i].driverState & 0x80) != 0){
//                 strError += "ConfF";
//             }
//             if (strError.length()< 1){
//                 strError = "OK";
//             }
// //            if (i == 0){
// //                ui->driver1StateLineEdit->setText(strError);
// //            }
// //            else if (i == 1){
// //                ui->driver2StateLineEdit->setText(strError);
// //            }
// //            else if (i == 2){
// //                ui->driver3StateLineEdit->setText(strError);
// //            }
// //            else if (i == 3){
// //                ui->driver4StateLineEdit->setText(strError);
// //            }


//         }
//         //publish sensor here
// //  qnode.publisherMotorData(motorData,8);
// //        qnode.publisherMotorBoardInfoArray(motorBoardData,4);
//     }
} // namespace jaguar
}