
#include <sstream>
#include "../include/Jaguar.hpp"
int countt = 0;
double resTable[25] = {114660,84510,62927,47077,35563,27119,20860,16204,12683,10000,7942,6327,5074,4103,3336,2724,2237,1846,1530,1275,1068,899.3,760.7,645.2,549.4};
double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
double FULLAD = 4095;


Jaguar::Jaguar(const std::string &ip, int port){
	ipTCP = new tcpSocket(ip, port);
	File::getInstance();
 for (int i = 0; i < 4; i++){
    flipArmMotor[i].angle = File::getInstance()->getFlipers(i);
    flipArmMotor[i].preEncoder = 0;
    flipArmMotor[i].encoderPos = 0;
    flipArmMotor[i].iniFlag = false;
  }

	MAX_WHEELS_ACCEL = 30;
	current_left_Wheel_Speed = 0;
    current_right_Wheel_Speed = 0;

    print("connecting to Jaguar...");
    if (ipTCP->connect())
    {
        print("Jaguar is connected");
    }
    else
    {
        print("Failed to connect to Jaguar");
    }
}

double Jaguar::getFrontRightFliperAngle(){
    return flipArmMotor[1].angle;
}

double Jaguar::getFrontLeftFliperAngle(){
    return flipArmMotor[0].angle;
}

double Jaguar::getBackRightFliperAngle(){
    return flipArmMotor[3].angle;
}

double Jaguar::getBackLeftFliperAngle(){
    return flipArmMotor[2].angle;
}

Jaguar::~Jaguar(){
	delete ipTCP;
}

bool Jaguar::connect(){
	return ipTCP->connect();
}

bool Jaguar::is_acceleration_reach_the_limit(int currentSpeed, int aimedSpeed){
	int speedDiff = currentSpeed - aimedSpeed;
	if ( (speedDiff < MAX_WHEELS_ACCEL ) && ( speedDiff >  (- MAX_WHEELS_ACCEL))) {
        return false;
    }else{
       return true; 
    }
}

int Jaguar::adjust_left_Speed( int aimedSpeed){

	if(aimedSpeed > current_left_Wheel_Speed ){
		return ( current_left_Wheel_Speed + MAX_WHEELS_ACCEL ) ;
	}else{
		return ( current_left_Wheel_Speed - MAX_WHEELS_ACCEL );
	}
}

int Jaguar::adjust_right_Speed( int aimedSpeed){
	if(aimedSpeed > current_right_Wheel_Speed ){
		return ( current_right_Wheel_Speed + MAX_WHEELS_ACCEL ) ;
	}else{
		return ( current_right_Wheel_Speed - MAX_WHEELS_ACCEL );
	}
}
void Jaguar::moveWheels(int left,int right){



    int i = 0;
    int l_left  = left;
	int l_right = right;
        while((current_left_Wheel_Speed != left|| current_right_Wheel_Speed != right) ){


            if(is_acceleration_reach_the_limit(current_right_Wheel_Speed , right)) { //
                l_right = adjust_right_Speed(right);
                current_right_Wheel_Speed = l_right;
            }else{
                l_right = right;
                current_right_Wheel_Speed = right;
            }
            
            if(is_acceleration_reach_the_limit(current_left_Wheel_Speed , left)) { //
                l_left = adjust_left_Speed(left);
                current_left_Wheel_Speed = l_left;
            }else{
                l_left = left;
                current_left_Wheel_Speed = left;
            }
            usleep(5000);
            print(i++);
            moveWheels_1(l_left,l_right);
        }
}


void Jaguar::moveWheels_1(int left, int right)
{

	if ((left == 0) && (right == 0))
		stopWheels();
	if ((left < -1000) || (right < -1000))
	{
		ipTCP->send("MMW !EX\r\n");
	}
	else if ((left > 1000) || (right > 1000))
	{
		ipTCP->send("MMW !MG\r\n");
	}
	else
	{
		ipTCP->send(("MMW !M " + toString(left) + " " + toString(right) + "\r\n"));
	}
}

void Jaguar::stopWheels()
{
	ipTCP->send("MMW !M 0 0\r\n");
}

void Jaguar::secureWheels()
{
	ipTCP->send("MMW !EX\r\n");
}

void Jaguar::sendPing()
{
	ipTCP->send("PING\r\n");
}
// void Jaguar::moveFlipers(double _1, double _2, double _3, double _5)
// {
// 	std::stringstream _1_;
// 	std::stringstream _2_;
// 	std::stringstream _3_;
// 	std::stringstream _5_;
// 	_1_ << _1;
// 	_2_ << _2;
// 	_3_ << _3;
// 	_5_ << _5;

// 	std::string _1_s = _1_.str();
// 	std::string _2_s = _2_.str();
// 	std::string _3_s = _3_.str();
// 	std::string _5_s = _5_.str();

//   std::string strCmd;
//   strCmd = "MM2 !PR 1 " + _2_s + "\r\n";
//   if (ipTCP != nullptr){
//     // if (ipTCP->isWritable())
//     // {
//         ipTCP->send(strCmd);
//     // }
//   }

//   strCmd = "MM2 !PR 2 " + _1_s + "\r\n";
//   if (ipTCP != nullptr){
//     // if (ipTCP->isWritable())
//     // {
//         ipTCP->send(strCmd);
//     // }
//   }

//   strCmd = "MM3 !PR 1 " + _3_s + "\r\n";
//   if (ipTCP != nullptr){
//     // if (ipTCP->isWritable())
//     // {
//         ipTCP->send(strCmd);
//     // }
//   }

//   strCmd = "MM3 !PR 2 " + _5_s+ "\r\n";
//   if (ipTCP != nullptr){
//     // if (ipTCP->isWritable())
//     // {
//         ipTCP->send(strCmd);
//     // }
//   }

// }

void Jaguar::releaseWheels()
{
	ipTCP->send("MMW !MG\r\n");
}

void Jaguar::releaseFrontFlipers()
{
	ipTCP->send("MM2 !MG\r\n");
}

void Jaguar::releaseRearFlipers()
{
	ipTCP->send("MM3 !MG\r\n");
}

void Jaguar::turnLeft()
{
	ipTCP->send( "MMW !M 200 200\r\n");
}

void Jaguar::turnRight()
{
	ipTCP->send("MMW !M -200 -200\r\n");
}

void Jaguar::forward()
{
	ipTCP->send("MMW !M -200 200\r\n");
}

void Jaguar::backward()
{
	ipTCP->send("MMW !M 200 -200\r\n");
}

void Jaguar::lightON()
{
	ipTCP->send("SYS MMC 255\r\n");
}

void Jaguar::lightOff()
{
	ipTCP->send("SYS MMC 127\r\n");
}

std::string Jaguar::read()
{
	return ipTCP->read();
}



void Jaguar::move_right_Front_Fliper_Degree(double radians_Angle)
{
 if ( ( radians_Angle + flipArmMotor[1].angle )  < (MAX_FLIPERS_ANGLE) && 
  ( radians_Angle + flipArmMotor[1].angle )  > (-MAX_FLIPERS_ANGLE) ){
    
    std::string strCmd;
    int targetPos = 0;

    targetPos = int( - radians_Angle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
	
    strCmd = "MM2 !PR 2 " +  toString(targetPos) + "\r\n";
    if (ipTCP != nullptr){
        // if (m_tcpRobot->isWritable()){
            ipTCP->send(strCmd);
            flipArmMotor[1].angle += radians_Angle;
            File::getInstance()->setFlipers(flipArmMotor[1].angle, 1);
        // }
        
    }
 }
} 

void Jaguar::move_left_Front_Fliper_Degree(double radians_Angle)
{
    if ( ( radians_Angle + flipArmMotor[0].angle )  < (MAX_FLIPERS_ANGLE) && 
     ( radians_Angle + flipArmMotor[0].angle )  > (-MAX_FLIPERS_ANGLE) ){

        std::string strCmd;
        int targetPos = 0;
        targetPos = int(radians_Angle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
        strCmd = "MM2 !PR 1 " +  toString(targetPos) + "\r\n";
        if (ipTCP != nullptr){
            // if (m_tcpRobot->isWritable()){
                ipTCP->send(strCmd);
                flipArmMotor[0].angle += radians_Angle;
                File::getInstance()->setFlipers(flipArmMotor[0].angle, 0);
            // }
        }
    }
} 

void Jaguar::move_right_Back_Fliper_Degree(double radians_Angle)
{
    if ( ( radians_Angle + flipArmMotor[3].angle )  < (MAX_FLIPERS_ANGLE) && 
     ( radians_Angle + flipArmMotor[3].angle )  > (-MAX_FLIPERS_ANGLE) ){

        std::string strCmd;
        int targetPos = 0;
        targetPos = int(radians_Angle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
       
        strCmd = "MM3 !PR 2 " +  toString(targetPos) + "\r\n";
        if (ipTCP != nullptr){
            // if (m_tcpRobot->isWritable()){
                ipTCP->send(strCmd);
                flipArmMotor[3].angle += radians_Angle;
                File::getInstance()->setFlipers(flipArmMotor[3].angle, 3);
            // }
        }
    }

} 

void Jaguar::move_left_back_Fliper_Degree(double radians_Angle)
{
    if ( ( radians_Angle + flipArmMotor[2].angle ) < (MAX_FLIPERS_ANGLE) &&
     ( radians_Angle + flipArmMotor[2].angle ) > (-MAX_FLIPERS_ANGLE) ){
        std::string strCmd;
        int targetPos = 0;

        targetPos = int(-radians_Angle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);

        strCmd = "MM3 !PR 1 " +  toString(targetPos) + "\r\n";
        if (ipTCP != nullptr){
            // if (ipTCP->isWritable()){
                ipTCP->send(strCmd);
                flipArmMotor[2].angle += radians_Angle;
                File::getInstance()->setFlipers(flipArmMotor[2].angle, 2);
            // }
        }
    }
} 

void Jaguar::go_to_right_Front_Fliper_Degree(double radians)
{
    double diff_angle = radians - flipArmMotor[1].angle   ;
    move_right_Front_Fliper_Degree(diff_angle);
} 
void Jaguar::go_to_left_Front_Fliper_Degree(double radians)
{
    double diff_angle = radians - flipArmMotor[0].angle   ;
    move_left_Front_Fliper_Degree(diff_angle);
} 
void Jaguar::go_to_right_Back_Fliper_Degree(double radians)
{
    double diff_angle = radians - flipArmMotor[3].angle   ;
    move_right_Back_Fliper_Degree(diff_angle);
} 
void Jaguar::go_to_left_back_Fliper_Degree(double radians)
{
    double diff_angle = radians - flipArmMotor[2].angle   ;
    move_left_back_Fliper_Degree(diff_angle);
} 
void Jaguar::moveFlipers_degree(double rightFront , double leftFront, double rightBack, double leftBack)
{ 
    move_right_Front_Fliper_Degree( degreesToRadians(rightFront) );
    move_left_back_Fliper_Degree( degreesToRadians(leftBack));
    move_right_Back_Fliper_Degree( degreesToRadians(rightBack));
    move_left_Front_Fliper_Degree( degreesToRadians(leftFront));
}

void Jaguar::go_To_Flipers_degree(double rightFront , double leftFront, double rightBack, double leftBack)
{
    go_to_right_Front_Fliper_Degree( degreesToRadians(rightFront));
    go_to_left_back_Fliper_Degree( degreesToRadians(leftBack));
    go_to_right_Back_Fliper_Degree( degreesToRadians(rightBack));
    go_to_left_Front_Fliper_Degree( degreesToRadians(leftFront));
}

double Jaguar::ad2Temperature(int adValue)
{
    //for new temperature sensor
               double tempM = 0;
               double k = (adValue / FULLAD);
               double resValue = 0;
               if (k != 1.0)
               {
                   resValue = 10000 * k / (1 - k);      //AD value to resistor
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

void Jaguar::getFrontFlipAngle()
{
    int deltaEncoder = 0;
    if (flipArmMotor[0].iniFlag){       //leftFront
        deltaEncoder = flipArmMotor[0].encoderPos - flipArmMotor[0].preEncoder;
        flipArmMotor[0].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[0].angle;
        if (flipArmMotor[0].angle > M_PI) flipArmMotor[0].angle = -(2 * M_PI - flipArmMotor[0].angle);
        if (flipArmMotor[0].angle < -M_PI) flipArmMotor[0].angle = (2 * M_PI + flipArmMotor[0].angle);
        flipArmMotor[0].preEncoder = flipArmMotor[0].encoderPos;
    }

    if (flipArmMotor[1].iniFlag){   //rightFront
        deltaEncoder = flipArmMotor[1].encoderPos - flipArmMotor[1].preEncoder;
        flipArmMotor[1].angle = (double)(-deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[1].angle;
        if (flipArmMotor[1].angle > M_PI) flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[1].angle);
        if (flipArmMotor[1].angle < -M_PI) flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[1].angle);
        flipArmMotor[1].preEncoder = flipArmMotor[1].encoderPos;
    }
}

void Jaguar::getRearFlipAngle()
{
     int deltaEncoder = 0;
    if (flipArmMotor[2].iniFlag){       //leftRear
        deltaEncoder = flipArmMotor[2].encoderPos - flipArmMotor[2].preEncoder;
        flipArmMotor[2].angle = (double)(-deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[2].angle;
        if (flipArmMotor[2].angle > M_PI) flipArmMotor[2].angle = -(2 * M_PI - flipArmMotor[2].angle);
        if (flipArmMotor[2].angle < -M_PI) flipArmMotor[2].angle = (2 * M_PI + flipArmMotor[2].angle);
        flipArmMotor[2].preEncoder = flipArmMotor[2].encoderPos;
    }

    if (flipArmMotor[3].iniFlag){   //rightRear
        deltaEncoder = flipArmMotor[3].encoderPos - flipArmMotor[3].preEncoder;
        flipArmMotor[3].angle = (double)(deltaEncoder % FLIPARM_CIRCLE_CNT)/FLIPARM_CIRCLE_CNT * M_PI * 2 + flipArmMotor[3].angle;
        if (flipArmMotor[3].angle > M_PI) flipArmMotor[1].angle = -(2 * M_PI - flipArmMotor[3].angle);
        if (flipArmMotor[3].angle < -M_PI) flipArmMotor[1].angle = (2 * M_PI + flipArmMotor[3].angle);
        flipArmMotor[3].preEncoder = flipArmMotor[3].encoderPos;
    }

}

// void Jaguar:: dealWithPackage(std::string revData, int len)
// {
//     int index;
//     std::string prefix_1 = "#"; // IMU
//     std::string prefix_2 = "$GPRMC"; //GPS
//     std::string prefix_3 = "MM";
//     // std::string prefix_4 = "#";
//     std::vector<std::string> data;

//      if (startWith(revData, prefix_1) ){
//         //IMU sensor data package
//           data = split(revData,prefix_1,",");
//         if (data.size() > 15){
//             imuData.seqNo = std::stoi(data.at(0));
//             imuData.estYaw = std::stod(data.at(2));
//             imuData.gyroRaw[0] = std::stoi(data.at(4));
//             imuData.gyroRaw[1] =std::stoi(data.at(5));
//             imuData.gyroRaw[2] = std::stoi(data.at(6));
//             imuData.accelRaw[0] = std::stoi(data.at(8));
//             imuData.accelRaw[1] = std::stoi(data.at(9));
//             imuData.accelRaw[2] = std::stoi(data.at(10));
//             imuData.compassRaw[0] = std::stoi(data.at(12));
//             imuData.compassRaw[1] = std::stoi(data.at(13));
//             imuData.compassRaw[2] = std::stoi(data.at(14));
// //       publisherIMUData(imuData);
//         }
//     }
//     else if(startWith(revData,prefix_2)){
//         //GPS sensor data package
//           data = split(revData,prefix_2,",");
//         if (data.size() > 9){
//             gpsData.gpsTimeStamp = std::stod(data.at(1));
// //            ui->gpsTimeStampLineEdit->setText(data[1]);
//             if (data[2] == "A"){
//                 gpsData.gpsState = 1;
//             }
//             else if(data[2] == "V"){
//                 gpsData.gpsState = 0;
//             }
//             if (gpsData.gpsState >0){
// //                ui->gpsStateLineEdit->setText("Valid");
//             }
//             else{
// //                ui->gpsStateLineEdit->setText("InValid");
//             }
//             gpsData.gpsLat = std::stod(data.at(3));
//             if (data[4] == "S"){
//                 gpsData.gpsLat = -gpsData.gpsLat;
//             }
// //            ui->gpsLatLineEdit->setText(data[4] + ":" + data[3]);
//             gpsData.gpsLong = std::stod(data.at(5));
//             if (data[6] == "W"){
//                 gpsData.gpsLong = -gpsData.gpsLong;
//             }
// //            ui->gpsLongLineEdit->setText(data[6] + ":" + data[5]);
//             if (!data.at(7).empty()){
//                 gpsData.gpsVog = std::stod(data.at(7)) * KNNOT2MS;
// //                ui->gpsVogLineEdit->setText(QString::number(gpsData.gpsVog,'f',2));
//             }
//             if (!data[8].empty()){
//                 gpsData.gpsCog = std::stod(data.at(8));
// //                ui->gpsCogLineEdit->setText(data[8]);
//             }

//         //    publisherGPSInfo(gpsData);

//         }
//     }
//     else if(startWith(revData,"MM")){
//         // motor and driver board data package
//         if (startWith(revData,"MM0")){
//             index = 0;
//         }
//         else if (startWith(revData,"MM1")){
//             index = 1;
//         }
//         else if (startWith(revData,"MM2")){
//             index = 2;
//         }
//         else if (startWith(revData,"MM3")){
//             index = 3;
//         }
//                         //driver 1 and front motors
//         revData = revData.erase(0,4);

//         if (startWith(revData,"A=")){
//             //current data
// 			revData.erase(0,2);
//             data = split(revData,":");
//             motorData[index * 2+ 0].motorAmp = std::stod(data[0])/10;
//             motorData[index * 2+ 1].motorAmp = std::stod(data[1])/10;
//         }
//         else if(startWith(revData,"AI=")){
//             // A/D data, here 3,4 will be motor temperature sensor
//             revData.erase(0,3);
//             data = split(revData,":");
//             motorData[index * 2+ 0].motorTemp = ad2Temperature( std::stoi(data[2]));
//             motorData[index * 2+ 1].motorTemp = ad2Temperature( std::stod(data[3]));
//         }
//         else if(startWith(revData,"C=")){
//             // encoder position data
//            revData.erase(0,2);
//            data = split(revData,":");
//             motorData[index * 2+ 0].encoderPos = std::stoi(data[0]);
//             motorData[index * 2+ 1].encoderPos = std::stoi(data[1]);
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
//         else if(startWith(revData,"P=")){
//             // output PWM value, 0 ~ 1000
//             revData.erase(0,2);
//             data = split(revData,":");
//             motorData[index * 2+ 0].motorPower = std::stoi(data[0]);
//             motorData[index * 2+ 1].motorPower = std::stoi(data[1]);

//         }
//         else if(startWith(revData,"S=")){
//             // encoder velocity data RPM
//            revData.erase(0,2);
//             data = split(revData,":");
//             motorData[index * 2+ 0].encoderSpeed = std::stoi(data[0]);
//             motorData[index * 2+ 1].encoderSpeed = std::stoi(data[1]);

//         }
//         else if(startWith(revData,"T=")){
//             // motor driver board temperature
//             revData = revData.erase(0,2);
//             data = split(revData,":");
//             motorBoardData[index].ch1Temp = std::stoi(data[0]);
//             motorBoardData[index].ch2Temp = std::stoi(data[1]);
//         }
//         else if(startWith(revData,"V=")){
//             // voltage data
//             revData = revData.erase(0,2);
//        		data = split(revData,":");
//             motorBoardData[index].drvVoltage = std::stod(data[0])/10;
//             motorBoardData[index].motVoltage = std::stod(data[1])/10;
//             motorBoardData[index].reg5Voltage = std::stod(data[2])/1000;

//         }
//         else if(startWith(revData,"CR=")){
//             // here is the encoder relative difference reading,
//             // very useful to estimate the encoder/motor traveling distance
//         }
//         else if(startWith(revData,"FF=")){
//             // driver board state
//             revData = revData.erase(0,3);
//             motorBoardData[index].driverState = std::stoi(revData);
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



//         std::string strError ="";
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

// }


// void Jaguar::driveRearFlipDegree(double targetAngle)
// {
//     double deltaAngle;
//     std::string strCmd;
//     int targetPos = 0;
//     targetAngle = targetAngle * M_PI/180;
//     deltaAngle = targetAngle - flipArmMotor[2].angle;
//     targetPos = int(-deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);

//     std::stringstream targetPos_;
// 	targetPos_ << targetPos;
// 	std::string targetPos__s = targetPos_.str();

//     strCmd = "MM3 !PR 1 " + targetPos__s + "\r\n";
//     if (ipTCP != nullptr){
//         // if (ipTCP->isWritable()){
//             ipTCP->send(strCmd);
//         // }
//     }
//     deltaAngle = targetAngle - flipArmMotor[3].angle;
//     targetPos = int(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);

// 	targetPos_ << targetPos;
// 	targetPos__s = targetPos_.str();
//     strCmd = "MM3 !PR 2 " + targetPos__s + "\r\n";
//     if (ipTCP != nullptr){
//         // if (m_tcpRobot->isWritable()){
//             ipTCP->send(strCmd);
//         // }
//     }

// }

// void Jaguar::driveFrontFlipDegree(double targetAngle)
// {
//     double deltaAngle;
//     std::string  strCmd;
//     int targetPos = 0;
//     targetAngle = targetAngle * M_PI/180;
//     deltaAngle = targetAngle - flipArmMotor[0].angle;
//     targetPos = int(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    
	
// 	std::stringstream targetPos_ ;
// 	targetPos_ << targetPos;
// 	std::string targetPos__s = targetPos_.str();
	
// 	strCmd = "MM2 !PR 1 " + targetPos__s + "\r\n";
//     if (ipTCP != nullptr){
//         // if (ipTCP->isWritable()){
//             ipTCP->send(strCmd);
//         // }
//     }
//     deltaAngle = targetAngle - flipArmMotor[1].angle;
//     targetPos = int(-deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
//     strCmd = "MM2 !PR 2 " + targetPos__s + "\r\n";
//     if (ipTCP != nullptr){
//         // if (m_tcpRobot->isWritable()){
//             ipTCP->send(strCmd);
//         // }
//     }

// }
