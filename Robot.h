// Robot.h

#ifndef _ROBOT_h
#define _ROBOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include <Encoder.h>
#include <Matrix.h>
#include <Servo.h>
#include "Trajectory.h"
#include "Kinematics.h"

struct controller_vals
{
	double tau1;
	double tau2;
	double tau3;
	double tau4;
	double tau5;
};

struct joint_p_v_a {
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;

	double dq1;
	double dq2;
	double dq3;
	double dq4;
	double dq5;

	double ddq1;
	double ddq2;
	double ddq3;
	double ddq4;
	double ddq5;
};

class Robot
{
public:
	Robot();
	void move_robot(double xd, double yd, double zd, double thd, double phid);

private:

	controller_vals controller(trajectory_vals xd, trajectory_vals yd, trajectory_vals zd, trajectory_vals thd, trajectory_vals phid, double* last_e_q1, double* last_e_q2, double* last_e_q3, double* last_e_q4, double* last_e_q5, double* last_t);
	void move_motors(controller_vals tau);
	joint_p_v_a curr_joint_vals();
	joint_p_v_a ikin(trajectory_vals xtraj, trajectory_vals ytraj, trajectory_vals ztraj, trajectory_vals thtraj, trajectory_vals phitraj);
	task_space_vals fwkin();
	double joint_angle_rad(int joint);
	int map_servo_input(double input);
};

#endif


