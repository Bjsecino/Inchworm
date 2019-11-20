// Trajectory.h

#ifndef _TRAJECTORY_h
#define _TRAJECTORY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <Matrix.h>

struct trajectory_vals
{
	double x_t;
	double dx_t;
	double ddx_t;
};

trajectory_vals trajectory(double t, Matrix<double> coeffs);
Matrix<double> quinticTrajectory(double t0, double tf, double v0, double vf, double a0, double af, double q0, double qf);

#endif
