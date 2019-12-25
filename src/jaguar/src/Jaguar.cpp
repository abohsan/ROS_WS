
#include <sstream>
#include "../include/Jaguar.hpp"

Jaguar::Jaguar(const std::string &ip, int port){
	ipTCP = new tcpSocket(ip, port);
	ipTCP->connect();
}

void Jaguar::moveWheels(int left, int right)
{
	std::stringstream left_;
	left_ << left;
	std::string left_s = left_.str();

	std::stringstream right_;
	right_ << right;
	std::string right_s = right_.str();

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
		ipTCP->send(("MMW !M " + left_s + " " + right_s + "\r\n"));
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

void Jaguar::driveRearFlipDegree(double targetAngle)
{
    double deltaAngle;
    std::string strCmd;
    int targetPos = 0;

	std::stringstream targetAngle_ ;
	targetAngle_ << targetAngle;
	std::string targetAngle_s = targetAngle_.str();


    targetAngle = targetAngle * M_PI/180;
    deltaAngle = targetAngle - flipArmMotor[2].angle;
    targetPos = int(-deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);


    strCmd = "MM3 !PR 1 " + targetAngle_s + "\r\n";
    if (ipTCP != nullptr){
        // if (ipTCP->isWritable()){
            ipTCP->send(strCmd);
        // }
    }
    deltaAngle = targetAngle - flipArmMotor[3].angle;
    targetPos = int(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM3 !PR 2 " + targetAngle_s + "\r\n";
    if (ipTCP != nullptr){
        // if (m_tcpRobot->isWritable()){
            ipTCP->send(strCmd);
        // }
    }

}

void Jaguar::driveFrontFlipDegree(double targetAngle)
{
    double deltaAngle;
    std::string  strCmd;
    int targetPos = 0;

	std::stringstream targetAngle_ ;
	targetAngle_ << targetAngle;
	std::string targetAngle_s = targetAngle_.str();

    targetAngle = targetAngle * M_PI/180;
    deltaAngle = targetAngle - flipArmMotor[0].angle;
    targetPos = int(deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM2 !PR 1 " + targetAngle_s + "\r\n";
    if (ipTCP != nullptr){
        // if (ipTCP->isWritable()){
            ipTCP->send(strCmd);
        // }
    }
    deltaAngle = targetAngle - flipArmMotor[1].angle;
    targetPos = int(-deltaAngle/(M_PI * 2) * FLIPARM_CIRCLE_CNT);
    strCmd = "MM2 !PR 2 " + targetAngle_s + "\r\n";
    if (ipTCP != nullptr){
        // if (m_tcpRobot->isWritable()){
            ipTCP->send(strCmd);
        // }
    }

}

