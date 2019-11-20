#include "Kinematics.h"

task_space_vals forward_kinematics(double q1, double q2, double q3, double q4, double q5) {
	double l1 = 4.125;
	double l2 = 6.43;

	Matrix<double> T01 = DH_to_Trans_Mat(l1, q1, 0, -PI / 2);
	Matrix<double> T02 = T01 * DH_to_Trans_Mat(0, q2 - (PI / 2), l2, 0);
	Matrix<double> T03 = T02 * DH_to_Trans_Mat(0, q3, l2, 0);
	Matrix<double> T04 = T03 * DH_to_Trans_Mat(0, q4, l1, PI / 2);
	Matrix<double> T05 = T04 * DH_to_Trans_Mat(0, q5, 0, 0);

	double x = T05._entity[0][3];
	double y = T05._entity[1][3];
	double z = T05._entity[2][3];

	double z0_dot_z5 = T05._entity[2][2];
	double mag_z0 = 1; // always equal to 1 because of rotation matrix definition
	double mag_z5 = 1;

	double theta = acos(z0_dot_z5 / (mag_z0 * mag_z5));

	double phi = q5;

	task_space_vals position;
	position.x = x;
	position.y = y;
	position.z = z;
	position.th = theta;
	position.phi = phi;

	return position;
}

Matrix<double> DH_to_Trans_Mat(double d, double theta, double a, double alpha) {
	double T11 = cos(theta);
	double T12 = -sin(theta) * cos(alpha);
	double T13 = sin(theta) * sin(alpha);
	double T14 = a * cos(theta);

	double T21 = sin(theta);
	double T22 = cos(theta) * cos(alpha);
	double T23 = -cos(theta) * sin(alpha);
	double T24 = a * sin(theta);

	double T31 = 0;
	double T32 = sin(alpha);
	double T33 = cos(alpha);
	double T34 = d;

	double T41, T42, T43 = 0;
	double T44 = 1;

	double Tarray[4][4] = { {T11,T12,T13,T14},{T21,T22,T23,T24},{T31,T32,T33,T34},{T41,T42,T43,T44} };
	Matrix<double> T(4, 4, (double*)Tarray);

	return T;
}
