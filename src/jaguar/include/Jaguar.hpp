#ifndef __JAGUAR_HPP__
#define __JAGUAR_HPP__

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

#include "tcpSocket.hpp"
#include "variables.hpp"
#include "shareMethods.hpp"
class Jaguar
{
public:
	Jaguar(const std::string &ip, int port);
	~Jaguar();

	void moveWheels(int left, int right); // left and right wheels
	void releaseWheels();
	void releaseFrontFlipers();
	void releaseRearFlipers();
	void stopWheels();
	void secureWheels();


	// start of Auxliliary method
	void forward();
	void backward();
	void turnRight();
	void turnLeft();
	// End of Auxliliary method
	void sendPing();

	void lightON();
	void lightOff();

	std::string read();

	void driveRearFlipDegree(double);
	void driveFrontFlipDegree(double);

	void processRobotData();
	void dealWithPackage(std::string , int);

private:
	tcpSocket *ipTCP;

	MotorData flipArmMotor[4];
};

#endif