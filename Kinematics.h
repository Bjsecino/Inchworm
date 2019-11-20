// Kinematics.h

#ifndef _KINEMATICS_h
#define _KINEMATICS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Matrix.h>

struct task_space_vals
{
	double x;
	double y;
	double z;
	double th;
	double phi;
};

task_space_vals forward_kinematics(double q1, double q2, double q3, double q4, double q5);
Matrix<double> DH_to_Trans_Mat(double d, double theta, double a, double alpha);

#endif


