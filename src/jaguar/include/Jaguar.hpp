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

	void moveWheels(int left, int right); // left and right wheels of Jaguar
	void moveFlipers_degree(double frontLeft, double frontRight, double backLeft, double backRight); // First, Second, Third and Fourth Flipers
	void go_To_Flipers_degree(double frontLeft, double frontRight, double backLeft, double backRight); // First, Second, Third and Fourth Flipers
	// void driveFlipDegree(double frontLeft, double frontRight, double backLeft, double backRight); 
	void releaseWheels();
	void releaseFrontFlipers();
	void releaseRearFlipers();
	void stopWheels();
	void secureWheels();
	bool connect();


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

	// void driveRearFlipDegree(double);
	// void driveFrontFlipDegree(double);

 	void getFrontFlipAngle();
	void getRearFlipAngle();

	void processRobotData();
	// void dealWithPackage(std::string , int);
 	double ad2Temperature(int value); 
private:
	tcpSocket *ipTCP;
	MotorData flipArmMotor[4];
	// future work
	bool is_acceleration_reach_the_limit(int currentSpeed, int aimedSpeed);
	int adjust_Speed(int aimedSpeed);
	// bool checkFlipers();
	IMUData imuData;
	GPSData gpsData;
	MotorData motorData[8];
	MotorBoardData motorBoardData[4];


	void move_right_Front_Fliper_Degree(double degree);
	void move_left_Front_Fliper_Degree(double degree);
	void move_right_Back_Fliper_Degree(double degree);
	void move_left_back_Fliper_Degree(double degree);
	
	void go_to_right_Front_Fliper_Degree(double degree);
	void go_to_left_Front_Fliper_Degree(double degree);
	void go_to_right_Back_Fliper_Degree(double degree);
	void go_to_left_back_Fliper_Degree(double degree);

	int current_left_Wheel_Speed;
	int current_right_Wheel_Speed;
	int MAX_WHEELS_ACCEL;
	
};

#endif