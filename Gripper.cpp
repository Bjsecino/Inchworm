/*******************************************************************
* Gripper library that allows to enagage and disengage the grippers
* of the robot. This code is designed for the Vex 29 motor controllers.
*
* Nov 2019 - Josue Contreras and Trevor Rizzo, Swarm Construction MQP
******************************************************************/

#include "Gripper.h"

/*
* Gripper Constructors
*/

//Empty contructor to build object
Gripper::Gripper() {
}

//Constructor to added object properties. Two arguments are optional, zeroposition:
//sets the middle value for the range of values that are mapped. Threshold is not used
//but could serve to implement the write to motor function.
Gripper::Gripper(int pin, bool directionCW, int zeroPosition = 0, int threshold = 5) {

	pin = pin;
	directionCW = directionCW;
	zeroPosition = zeroPosition;
	threshold = threshold;
	medianPulse = 1500; //motor stops at this pulse width
	isEngaged = false;

	if (directionCW) {
		maxSpeedCCW = -255;
		maxSpeedCW = 255;
		maxPulse = 2000; //unique value to VEX 29 motorcontrollers
		minPulse = 1000; //unique value to VEX 29 motorcontrollers
	}
	else if (!directionCW) {
		maxSpeedCCW = 255;
		maxSpeedCW = -255;
		maxPulse = 1000; //unique value to VEX 29 motorcontrollers
		minPulse = 2000; //unique value to VEX 29 motorcontrollers
	}

	grip.attach(pin);
}

/*
* Set gripper flag to engaged or disengaged
*/

void Gripper::setEngaged(bool e) {
	isEngaged = e;
}

bool Gripper::getEngaged(void) {
	return isEngaged;
}

/*
* Motor spins CW for pulse widths less than 1500 uS and CCW
* for pulse widths greater than 1500. map() scales the power to a
* pulse width.
*/
void Gripper::write(int power) {

	int pulseWidth;

	pulseWidth = map(power, maxSpeedCCW, maxSpeedCW, maxPulse, minPulse);
	grip.writeMicroseconds(pulseWidth);
}

/*
* For gripper engagment and disengament
* TODO: when current sensing added to gripper, add that sensng instead of the time delay.
*/
bool Gripper::setGripper(gripperState gState, int time) {

	if (resetTime) {
		startTime = millis();
		resetTime = false;
	}

	switch (gState) {
	case engage: //engage gripper
		if ((int)(millis() - startTime) < time) {
			write(maxSpeedCCW);
			gripperFinished = false;
		}
		else {
			write(0);
			resetTime = true;
			gripperFinished = true;
			setEngaged(true);
		}
		break;
	case disengage: //disengage gripper
		if ((int)(millis() - startTime) < time) {
			write(maxSpeedCW);
			gripperFinished = false;
		}
		else {
			write(0);
			resetTime = true;
			gripperFinished = true;
			setEngaged(false);
		}
		break;
	case idle: //nothing happens
		write(0);
		gripperFinished = true;
		break;
	default: //if none are selected
		write(0);
		gripperFinished = true;
		break;
	}

	return gripperFinished;

}

bool Gripper::setGripper(int gState, int time) {

	if (resetTime) {
		startTime = millis();
		resetTime = false;
	}

	switch (gState) {
	case engage: //engage gripper
		if ((int)(millis() - startTime) < time) {
			write(maxSpeedCCW);
			gripperFinished = false;
		}
		else {
			write(0);
			resetTime = true;
			gripperFinished = true;
			setEngaged(true);
		}
		break;
	case disengage: //disengage gripper
		if ((int)(millis() - startTime) < time) {
			write(maxSpeedCW);
			gripperFinished = false;
		}
		else {
			write(0);
			resetTime = true;
			gripperFinished = true;
			setEngaged(false);
		}
		break;
	case idle: //nothing happens
		write(0);
		gripperFinished = true;
		break;
	default: //if none are selected
		write(0);
		gripperFinished = true;
		break;
	}

	return gripperFinished;
}
