// JointMotor.h

#ifndef _JOINTMOTOR_h
#define _JOINTMOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ams_as5048b.h"
//#include <Servor.h>

class JointMotor {
private:
	int pinDirectionA, pinDirectionB, pinPWM;
	AMS_AS5048B encoder;

	//PID
	double desiredAngle;
	double kP, kI, kD;
	double kP2, kI2, kD2;
	double sumError, lastError;

	double last_calibrated_angle; //angle of joint
	double angle_offset; // offset of angle in calibration position

public:
	bool debug;

	JointMotor();
	JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset);
	JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset);

	void    setSpeed(int speed);
	void    changeDirection(int speed);
	void    setAngle(double angle);
	void    switchPID();
	void    updateSpeed();
	double  getAngleDegrees();
	// double getKP();
	// void setKP(double kpValue)

};

#endif

