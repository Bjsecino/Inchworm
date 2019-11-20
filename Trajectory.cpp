// 
// 
// 

#include "Trajectory.h"

trajectory_vals trajectory(double t, Matrix<double> coeffs) {
	double x_t = coeffs._entity[0][0] + coeffs._entity[1][0] * t + coeffs._entity[2][0] * pow(t, 2) + coeffs._entity[3][0] * pow(t, 3) + coeffs._entity[4][0] * pow(t, 4) + coeffs._entity[5][0] * pow(t, 5);
	double dx_t = coeffs._entity[1][0] + 2 * coeffs._entity[2][0] * t + 3 * coeffs._entity[3][0] * pow(t, 2) + 4 * coeffs._entity[4][0] * pow(t, 3) + 5 * coeffs._entity[5][0] * pow(t, 4);
	double ddx_t = 2 * coeffs._entity[2][0] + 6 * coeffs._entity[3][0] * t + 12 * coeffs._entity[4][0] * pow(t, 2) + 20 * coeffs._entity[5][0] * pow(t, 3);

	trajectory_vals des;
	des.x_t = x_t;
	des.dx_t = dx_t;
	des.ddx_t = ddx_t;

	return des;
}

Matrix<double> quinticTrajectory(double t0, double tf, double v0, double vf, double a0, double af, double q0, double qf) {
	double quinticArray[6][6] = { {1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)},
								  {0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4)},
								  {0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3)},
								  {1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)},
								  {0, 1, 2 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4)},
								  {0, 0, 2, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3)} };
	Matrix<double> quinticMat(6,6, (double*)quinticArray);

	double initEndArray[6][1] = { {q0}, {v0}, {a0}, {qf}, {vf}, {af} };
	Matrix<double> initEndMat(6,1, (double*)initEndArray);

	Matrix<double> coeffMat = Matrix<double>::inv(quinticMat) * initEndMat;

	return coeffMat;
}