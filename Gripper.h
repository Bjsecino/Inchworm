//*****************************************************************************
//
// Gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Servo.h>
#include <Arduino.h>


typedef enum { // Used globaly as states for gripper functions
	engage,   //0
	disengage,//1
	idle,     //2
}gripperState;

class Gripper {
private:
	int pin, zeroPosition, threshold;
	int maxSpeedCCW, maxSpeedCW;
	int maxPulse, minPulse, medianPulse;
	bool directionCW;
	bool isEngaged;

	int lastUpdate = 0; //increment for motor move interval
	int startTime = 0;
	bool resetTime = true;
	bool gripperFinished;

	Servo grip;

public:

	Gripper(); //default constructor
	Gripper(int pin, bool directionCW, int zeroPosition = 0, int threshold = 5); //contructor with values

	void setEngaged(bool e);
	bool getEngaged(void);
	// Writes the pulsewidth specified to the motorcontroller and maps the values.
	void write(int power);
	// Sets gripper state to engage or disengage. Time is in milliseconds (1000 milliseconds = 1 sec)
	bool setGripper(gripperState gState, int time);
	bool setGripper(int gState, int time);

};

#endif
