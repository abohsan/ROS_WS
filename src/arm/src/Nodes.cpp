
#include <string>
#include "../include/Nodes.hpp"

namespace arm_ns
{


Nodes::Nodes(const ros::NodeHandle &node_handle,
             const ros::NodeHandle &private_node_handle): nh_(node_handle), pnh_(private_node_handle)
{
   arm = std::make_shared<Arm>();
//    arm->armSetIniCmd();
    this->init();
}


void Nodes::init()
{
    arm_motorInfo_pub_ = pnh_.advertise<arm::ArmMotorInfoArray>("/jaguar_arm_ctrl_sensor", 1);
    joint_pub = pnh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

    keyboard_sub = pnh_.subscribe("/keyboard_command", 1, &Nodes::keyboard_Callback, this);
    // joint_sub = pnh_.subscribe("/joints_radians", 1, &Nodes::join_sub_Callback, this);
    joint_0_sub = pnh_.subscribe("/joint_0", 1, &Nodes::join_0_sub_Callback, this);
    joint_1_sub = pnh_.subscribe("/joint_1", 1, &Nodes::join_1_sub_Callback, this);
    joint_2_sub = pnh_.subscribe("/joint_2", 1, &Nodes::join_2_sub_Callback, this);
    joint_3_sub = pnh_.subscribe("/joint_3", 1, &Nodes::join_3_sub_Callback, this);
    joint_4_sub = pnh_.subscribe("/joint_4", 1, &Nodes::join_4_sub_Callback, this);

    ping_timer = pnh_.createTimer(ros::Duration(0.3), &Nodes::ping_timer_callback, this);
    pub_timer = pnh_.createTimer(ros::Duration(0.5), &Nodes::pub_timer_callback, this);

}

void Nodes::ping_timer_callback(const ros::TimerEvent &event)
{
    arm->sendPings();
}

void Nodes::pub_timer_callback(const ros::TimerEvent &event)
{
    // print("arm_processRobotData1");
    // arm_processRobotData1();
    //   print("processRobotData2");
    // arm->processRobotData2();
    //   print("processRobotData3");
    // arm->processRobotData3();
    // publisherFliperJointStatus();
}

// void Nodes::join_sub_Callback(const arm::JointAngle::ConstPtr &msg){
//     std::cout << "joint_0 : " <<  msg->joint_0 << std::endl;
//     std::cout << "joint_1 : " <<  msg->joint_1 << std::endl;
//     std::cout << "joint_2 : " <<  msg->joint_2 << std::endl;
//     std::cout << "joint_3 : " <<  msg->joint_3 << std::endl;
//     std::cout << "joint_4 : " <<  msg->joint_4 << std::endl;
//     std::cout << "joint_5 : " <<  msg->joint_5 << std::endl;
//     std::cout << "" << std::endl;


//     arm->move_arm_radians( msg->joint_0, msg->joint_1, msg->joint_2,
//                             msg->joint_3, msg->joint_4, msg->joint_5);
// }

void Nodes::join_0_sub_Callback(const arm::singleJoint::ConstPtr &msg)
{
    arm->j0Go_to_radians(msg->joint);
}

void Nodes::join_1_sub_Callback(const arm::singleJoint::ConstPtr &msg)
{
    arm->j1Go_to_radians(msg->joint);
}

void Nodes::join_2_sub_Callback(const arm::singleJoint::ConstPtr &msg)
{
    arm->j2Go_to_radians(msg->joint);
}

void Nodes::join_3_sub_Callback(const arm::singleJoint::ConstPtr &msg)
{
    arm->j3Go_to_radians(msg->joint);
}

void Nodes::join_4_sub_Callback(const arm::singleJoint::ConstPtr &msg)
{
    arm->j4Go_to_radians(msg->joint);
}

void Nodes::keyboard_Callback(const std_msgs::String::ConstPtr &msg)
{
    // ROS_INFO_STREAM("KeyPressed : " << msg->data);
    if (msg->data == "9")
    {

    }
    else if (msg->data == "i")
    {
        arm->armSetIniCmd();
        print(" armSetIniCmd ");

    }
    else if (msg->data == "z")
    {
        arm->motorReleaseAll();
        print("arm motor released");
    }
    else if (msg->data == "f")
    { 
        
    }
   
    else if (msg->data == "b")
    {
        // arm->armReleaseInitial();
    }
    else if (msg->data == "r")
    {
        // arm->armSetIniCmd();
        // print("armSetIniCmd");
    }
    else if (msg->data == "l")
    {

    }
    else if (msg->data == "s")
    {
            // arm->motorStopAll();
            //  print("motorStopAll");
    }
    else if (msg->data == "m0u")
    {
        arm->j0Go_to_degree(5);
    }
    else if (msg->data == "m0d")
    {
        arm->j0Go_to_degree(-5);
    }
    else if (msg->data == "m1u")
    {
        arm->j1Go_to_degree(100);
    }
    else if (msg->data == "m1d")
    {
        arm->j1Go_to_degree(-300);
    }
    else if (msg->data == "m2u")
    {
        arm->j2Go_to_degree(43);
    }
    else if (msg->data == "m2d")
    {
        arm->j2Go_to_degree(1);
    }
    else if (msg->data == "m3u")
    {
        arm->j3Go_to_degree(10);
    }
    else if (msg->data == "m3d")
    {
        arm->j3Go_to_degree(-10);
    }
    else if (msg->data == "m4u")
    {
            arm->j4Go_to_degree(10);
    }
    else if (msg->data == "m4d")
    {
            arm->j4Go_to_degree(0);
    }
    else if (msg->data == "m5u")
    {
        arm->moveMotor_5(500);
    }
    else if (msg->data == "m5d")
    {
        arm->moveMotor_5(-100);
        
    }
    else if (msg->data == "m6u")
    {
      arm->moveMotor_5(0);
    }
    else if (msg->data == "m6d")
    {
           arm->moveMotor_5(250);
    }
    else if (msg->data == "m7u")
    {
        // arm->armPosGoCmd();
    }
    else if (msg->data == "m7d")
    {

    }
    else
    {
        ROS_INFO_STREAM("KeyPressed : " << msg->data);
    }

    ros::Rate loop_rate(50);
    // sleep for 200ms
    loop_rate.sleep();
}


// void Nodes::arm_sub_Callback(const jaguar::FlipMotor::ConstPtr &msg)
// {
//     // jaguar->moveFlipers_degree(msg->rightFront, msg->leftFront, msg->rightRear, msg->leftRear);
//     // jaguar->go_To_Flipers_degree(msg->go_to_rightFront, msg->go_to_leftFront, msg->go_to_rightRear, msg->go_to_leftRear);

//     ros::Rate loop_rate(5);
//     // sleep for 200ms
//     loop_rate.sleep();
// }

void Nodes::publisherArmJointStatus()
{
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(15);
    joint_state.position.resize(15);
    joint_state.name[0] = "base_link__base_arm";
    joint_state.name[1] = "base_link__wheel_front_left";
    joint_state.name[2] = "base_link__wheel_front_right";
    joint_state.name[3] = "base_link__wheel_back_right";
    joint_state.name[4] = "base_link__wheel_back_left";
    joint_state.name[5] = "base_link__fliper_front_left";
    joint_state.name[6] = "base_link__fliper_back_left";
    joint_state.name[7] = "base_link__fliper_front_right";
    joint_state.name[8] = "base_link__fliper_back_right";
    joint_state.name[9] = "base_arm__arm_1";
    joint_state.name[10] = "arm_1__arm_2";
    joint_state.name[11] = "arm_2__arm_3";
    joint_state.name[12] = "arm_3__arm_4";
    joint_state.name[13] = "arm_4__griper_1";
    joint_state.name[14] = "arm_4__griper_2";

    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    // joint_state.position[5] = jaguar->getFrontLeftFliperAngle();
    // joint_state.position[6] = PI - jaguar->getBackLeftFliperAngle();
    // joint_state.position[7] = jaguar->getFrontRightFliperAngle();
    // joint_state.position[8] = PI - jaguar->getBackRightFliperAngle();
    joint_state.position[5] =  0;
    joint_state.position[6] =  0;
    joint_state.position[7] =  0;
    joint_state.position[8] =  0;
    joint_state.position[9] =  0;
    joint_state.position[10] = 0;
    joint_state.position[11] = 0;
    joint_state.position[12] = 0;
    joint_state.position[13] = 0;
    joint_state.position[14] = 0;

    joint_pub.publish(joint_state);
}



void Nodes::arm_publisher(int pos[],int vel[],int pwm[], double temp[],double angle[],int len)
{
    arm::ArmMotorInfoArray motorInfoArray;
	motorInfoArray.motorInfos.resize(len);
	for (uint32_t i = 0 ; i < len; ++i)
	{
	  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
	  motorInfoArray.motorInfos[i].header.frame_id = "/arm_motor";
	  motorInfoArray.motorInfos[i].encoder_pos = pos[i];
	  motorInfoArray.motorInfos[i].encoder_vel = vel[i];
	  motorInfoArray.motorInfos[i].motor_pwm = pwm[i];
	  motorInfoArray.motorInfos[i].motor_temperature = temp[i];
	  motorInfoArray.motorInfos[i].motorID = i;
	  motorInfoArray.motorInfos[i].joint_angle = angle[i];
	}

	//ROS_INFO("publish motor info array");
	arm_motorInfo_pub_.publish(motorInfoArray);
}

void Nodes::arm_dealWithPackage1(std::string received)
{
    print("arm_dealWithPackage1 inside");
    std::string temp;
    std::vector<std::string> rev = splitByString(received, "\r");

    for (int i = 0; i < rev.size(); i++)
    {
        received = rev.at(i);

        if (startWith(received,"A="))
        {
            received.erase(0,2);
            std::vector<std::string>  strData = split(received, ':');
            try
            {
                if (strData.size()> 1)
                {
                    arm->motorData1.motAmp1 = std::stod(strData.at(0)) / 10;
                    arm->motorData1.motAmp2 = std::stod(strData.at(1)) / 10;
                    arm->jointMotorData[0].currentAmp = arm->motorData1.motAmp1;
                    arm->jointMotorData[1].currentAmp = arm->motorData1.motAmp2;

                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received, "AI="))
        {
            received.erase(0,3);
            std::vector<std::string> strData = split(received, ':');
            try
            {
                if (strData.size()> 3)
                {
                    arm->motorData1.ai3 = std::stod(strData.at(2));
                    arm->motorData1.motTemp1 = arm->ad2Temperature(arm->motorData1.ai3);
                    arm->motorData1.ai4 = std::stod(strData.at(3));
                    arm->motorData1.motTemp2 = arm->ad2Temperature(arm->motorData1.ai4);
                    arm->jointMotorData[0].motorTemperature = arm->motorData1.motTemp1;
                    arm->jointMotorData[1].motorTemperature = arm->motorData1.motAmp2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received, "C="))
        {
            received.erase(0,2);
            std::vector<std::string> strData = split(received, ':');
            try
            {
                if (strData.size()> 1)
                {
                    arm->motorData1.motEncP1 = std::stoi(strData.at(0));
                    arm->motorData1.motEncP2 = std::stoi(strData.at(1));
                    arm->jointMotorData[0].encoderPos = arm->motorData1.motEncP1;
                    arm->jointMotorData[1].encoderPos = arm->motorData1.motEncP2;
                    arm->jointMotorData[0].angle = arm->trans2Angle(0);
                    arm->jointMotorData[1].angle = (arm->trans2Angle(1));
           
                    //get arm tip end position
                    arm->getPositionXY();
						//publish all the joint information here
						//publish sensor data here
							 int motorPos[arm->get_MOTOR_NUM()];
							 int motorVel[arm->get_MOTOR_NUM()];
							 int motorPWM[arm->get_MOTOR_NUM()];
							 double motorTemperature[arm->get_MOTOR_NUM()];
							 double jointAngle[arm->get_MOTOR_NUM()];
							for(int i = 0; i < arm->get_MOTOR_NUM(); i++)
							{
 								motorPos[i] = arm->jointMotorData[i].encoderPos;
   							motorVel[i] = arm->jointMotorData[i].encodeSpeed;
								motorPWM[i] = arm->jointMotorData[i].pwmOutput;
    							motorTemperature[i] = arm->jointMotorData[i].motorTemperature;
								jointAngle[i] = arm->jointMotorData[i].angle;
							}
							arm_publisher(motorPos,motorVel,motorPWM,motorTemperature,jointAngle,arm->get_MOTOR_NUM()) ;

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
                        arm->motorData1.motPower1 = std::stoi(strData.at(0));
                        arm->motorData1.motPower2 = std::stoi(strData.at(1));
                        arm->jointMotorData[0].pwmOutput = arm->motorData1.motPower1;
                        arm->jointMotorData[1].pwmOutput = arm->motorData1.motPower2;
                }
            }
            catch(...)
            {

            }
        }
        else if (startWith(received,"S="))
        {
            received.erase(0,2);
            std::vector<std::string>  strData = split(received,':');
            try
            {
                if (strData.size()> 1)
                {
                        arm->motorData1.motEncS1 = std::stoi(strData.at(0));
                        arm->motorData1.motEncS2 = std::stoi(strData.at(1));
                        arm->jointMotorData[0].encodeSpeed = arm->motorData1.motEncS1;
                        arm->jointMotorData[1].encodeSpeed = arm->motorData1.motEncS2;
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
                        arm->motorData1.ch1Temp = std::stoi(strData.at(0));
                        arm->motorData1.ch2Temp =  std::stoi(strData.at(1));
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
                        arm->motorData1.drvVoltage = std::stod(strData.at(0))/10;
                        arm->motorData1.batVoltage = std::stod(strData.at(1))/10;
                        arm->motorData1.reg5VVoltage = std::stod(strData.at(2))/1000;

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
                        arm->motorData1.mode1 =  std::stoi(strData.at(0));
                        arm->motorData1.mode2 = std::stoi(strData.at(1));
                    }
                }
                catch(...)
                {

                }
            }
        else if (startWith(received,"FF="))
            {
                received.erase(0,3);
                // received.remove(QChar('\r'),Qt::CaseInsensitive);

                try
                {
                    arm->motorData1.statusFlag = std::stoi(received);
                    if((arm->motorData1.statusFlag & 0x01) != 0)
                    {
                        arm->lineEditChannel1State =("OverHeat+");
                    }
                    if((arm->motorData1.statusFlag & 0x02) != 0)
                    {
                        arm->lineEditChannel1State =("OverVoltage+");
                    }
                    if((arm->motorData1.statusFlag & 0x04) != 0)
                    {
                        arm->lineEditChannel1State =("UnderVol+");
                    }
                    if((arm->motorData1.statusFlag & 0x08) != 0)
                    {
                       arm->lineEditChannel1State =("Short+");
                    }
                    if((arm->motorData1.statusFlag & 0x10) != 0)
                    {
                       arm->lineEditChannel1State =("ESTOP+");
                    }
                    if (arm->motorData1.statusFlag == 0)
                    {
                        arm->lineEditChannel1State =("OK");
                    }
                }
                catch(...)
                {

                }
            }
    }



}

// void Nodes::arm_dealWithPackage1(std::string received)
// {
//     std::string temp;
//     std::vector<std::string> rev = splitByString(received, "\r");

//     for (int i = 0; i < rev.size(); i++)
//     {
//         received = rev.at(i);

//         if (startWith(received, "A="))
//         {
//             received.erase(0, 2);
//             std::vector<std::string> strData = split(received, ':');
//             try
//             {
//                 if (strData.size() > 1)
//                 {
//                     arm->motorData1.motAmp1 = std::stod(strData.at(0)) / 10;
//                     arm->motorData1.motAmp2 = std::stod(strData.at(1)) / 10;
//                     arm->jointMotorData[0].currentAmp = arm->motorData1.motAmp1;
//                     arm->jointMotorData[1].currentAmp = arm->motorData1.motAmp2;
//                 }
//             }
//             catch (...)
//             {
//             }
//         }
//         else if (startWith(received, "AI="))
//         {
//             received.erase(0, 3);
//             // received.remove(QChar('\r'),Qt::CaseInsensitive);
//             std::vector<std::string> strData = split(received, ':');
//             try
//             {
//                 if (strData.size() > 3)
//                 {
//                     arm->motorData1.ai3 = std::stod(strData.at(2));
//                     arm->motorData1.motTemp1 = arm->ad2Temperature(arm->motorData1.ai3);
//                     arm->motorData1.ai4 = std::stod(strData.at(3));

//                     arm->motorData1.motTemp2 = arm->ad2Temperature(arm->motorData1.ai4);
//                     arm->jointMotorData[0].motorTemperature = arm->motorData1.motTemp1;
//                     arm->jointMotorData[1].motorTemperature = arm->motorData1.motAmp2;
//                 }
//             }
//             catch (...)
//             {
//             }
//         }
//         else if (startWith(received, "C="))
//         {print(received);
//             received.erase(0, 2);
           
//             std::vector<std::string> strData = split(received, ':');
//             try
//             { 
//                 if (strData.size() > 1)
//                 {
//                     arm->motorData1.motEncP1 = std::stoi(strData.at(0));
//                     arm->motorData1.motEncP2 = std::stoi(strData.at(1));
//                     // std::cout << std::stoi(strData.at(0)) << " : " << std::stoi(strData.at(1))<< std::endl;
//                     // temp.setNum(motorData1.motEncP1);
//                     //                     ui->lineEditM1Pos->setText(temp);
//                     // temp.setNum(motorData1.motEncP2);
//                     //                     ui->lineEditM2Pos->setText(temp);
//                     arm->jointMotorData[0].encoderPos = arm->motorData1.motEncP1;
//                     arm->jointMotorData[1].encoderPos = arm->motorData1.motEncP2;
//                     arm->jointMotorData[0].angle = arm->trans2Angle(0);
//                     // arm->jointMotorData[1].angle = -(arm->trans2Angle(1) - arm->get_JOINT_INI_ANGLE(i)) + arm->jointMotorData[0].angle;
//                     arm->jointMotorData[1].angle = (arm->trans2Angle(1));
           
//                     //get arm tip end position
//                     arm->getPositionXY();
    
//                                     // publish all the joint information here
//                                     // publish sensor data here
//                                     int motorPos[arm->get_MOTOR_NUM()];
//                                     int motorVel[arm->get_MOTOR_NUM()];
//                                     int motorPWM[arm->get_MOTOR_NUM()];
//                                     double motorTemperature[arm->get_MOTOR_NUM()];
//                                     double jointAngle[arm->get_MOTOR_NUM()];
//                                     for(int i = 0; i < arm->get_MOTOR_NUM(); i++)
//                                     {
//                                         motorPos[i] = arm->jointMotorData[i].encoderPos;
//                                         motorVel[i] = arm->jointMotorData[i].encodeSpeed;
//                                         motorPWM[i] = arm->jointMotorData[i].pwmOutput;
//                                         motorTemperature[i] = arm->jointMotorData[i].motorTemperature;
//                                         jointAngle[i] = arm->jointMotorData[i].angle;
//                                     }
//                     arm_publisher(motorPos,motorVel,motorPWM,motorTemperature,jointAngle,arm->get_MOTOR_NUM()) ;
//                 }
//             }
//             catch (...)
//             {
//             }
//         }
//             else if (startWith(received,"P="))
//             {
//                 received.erase(0,2);
//                 std::vector<std::string> strData = split(received,':');
//                 try
//                 {
//                     if (strData.size()> 1)
//                     {
//                         arm->motorData1.motPower1 = std::stoi(strData.at(0));
//                         arm->motorData1.motPower2 = std::stoi(strData.at(1));
//                         arm->jointMotorData[0].pwmOutput = arm->motorData1.motPower1;
//                         arm->jointMotorData[1].pwmOutput = arm->motorData1.motPower2;
//                     }
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//             else if (startWith(received,"S="))
//             {
//                 received.erase(0,2);
//                 std::vector<std::string>  strData = split(received,':');
//                 try
//                 {
//                     if (strData.size() > 1)
//                     {
//                         arm->motorData1.motEncS1 = std::stoi(strData.at(0));
//                         arm->motorData1.motEncS2 = std::stoi(strData.at(1));
//                         arm->jointMotorData[0].encodeSpeed = arm->motorData1.motEncS1;
//                         arm->jointMotorData[1].encodeSpeed = arm->motorData1.motEncS2;
//                     }
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//             else if (startWith(received,"T="))
//             {
//                 received.erase(0,2);
//                   std::vector<std::string> strData = split(received,':');
//                 try
//                 {
//                     if (strData.size()> 1)
//                     {
//                         arm->motorData1.ch1Temp = std::stoi(strData.at(0));
//                         arm->motorData1.ch2Temp =  std::stoi(strData.at(1));
//                     }
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//             else if (startWith(received,"V="))
//             {
//                 received.erase(0,2);
//                 std::vector<std::string> strData = split(received,':');
//                 try
//                 {
//                     if (strData.size()> 2)
//                     {
//                         arm->motorData1.drvVoltage = std::stod(strData.at(0))/10;
//                         arm->motorData1.batVoltage = std::stod(strData.at(1))/10;
//                         arm->motorData1.reg5VVoltage = std::stod(strData.at(2))/1000;
//                     }
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//             else if (startWith(received,"MMOD="))
//             {
//                 received.erase(0,5);
//                 std::vector<std::string>  strData = split(received,':');
//                 try
//                 {
//                     if (strData.size()> 1)
//                     {
//                         arm->motorData1.mode1 =  std::stoi(strData.at(0));
//                         arm->motorData1.mode2 = std::stoi(strData.at(1));
//                     }
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//             else if (startWith(received,"FF="))
//             {
//                 received.erase(0,3);
//                 // received.remove(QChar('\r'),Qt::CaseInsensitive);

//                 try
//                 {
//                     arm->motorData1.statusFlag = std::stoi(received);
//                     temp="";
//                     if((arm->motorData1.statusFlag & 0x01) != 0)
//                     {
//                         temp.append("OverHeat+");
//                     }
//                     if((arm->motorData1.statusFlag & 0x02) != 0)
//                     {
//                         temp.append("OverVoltage+");
//                     }
//                     if((arm->motorData1.statusFlag & 0x04) != 0)
//                     {
//                         temp.append("UnderVol+");
//                     }
//                     if((arm->motorData1.statusFlag & 0x08) != 0)
//                     {
//                         temp.append("Short+");
//                     }
//                     if((arm->motorData1.statusFlag & 0x10) != 0)
//                     {
//                         temp.append("ESTOP+");
//                     }
//                     if (arm->motorData1.statusFlag == 0)
//                     {
//                         temp.append("OK");
//                     }

//                     //                 ui->lineEditChannel1State->setText(temp);
//                 }
//                 catch(...)
//                 {

//                 }
//             }
//     }
// }

void Nodes::arm_processRobotData1() {
    // std::string received = "";
    // std::string m_receivedData1;
    // // print(arm->read_ipTCP_1());
    //     received = arm->read_ipTCP_1();
    //     m_receivedData1.append(received);
    // if ( endWith(m_receivedData1,"\r") ) {
        // print("Inside");
        arm_dealWithPackage1(arm->read_ipTCP_1());
        // m_receivedData1 = "";
    // }
    //  m_receivedData1 = "";
}

// void Nodes::arm_processRobotData2()
// {
//     std::string received = "";
//      std::string m_receivedData2;


//         received = arm->read_ipTCP_2
//         m_receivedData2.append(received);
//     if (endWith(m_receivedData2,"\r"))
//     {
//         arm_dealWithPackage2(m_receivedData2);
//         m_receivedData2 = "";
//     }
// }

// void Nodes::arm_processRobotData3()
// {
    // std::string received = "";
    // int count = 0;

    // m_watchDogCnt2 = 0;
    // // while( (count = ipTCP_3->bytesAvailable()) > 0)
    // // {

    //     received = ipTCP_3->read();
    //     m_receivedData3.append(received);
    // // }
    // if (endWith(m_receivedData3 , "\r"))
    // {
    //     dealWithPackage3(m_receivedData3);
    //     m_receivedData3 = "";
    // }
// }
} // namespace jaguar_ns