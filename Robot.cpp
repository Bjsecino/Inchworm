#include "Robot.h"
//initialize encoders and motors



Robot::Robot() {}

	void Robot::init() {
		jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 100, 0.1, 50, 10, 0.1, 5, 27.81, 1);
		jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 120, 0.1, 60, 124.38, 1);
		jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, 0.1, 5, 100, 0.1, 60, 27.81, 0);
		gripper[0] = Gripper(GRIPPER_MOTOR_1, true);
		gripper[1] = Gripper(GRIPPER_MOTOR_2, false);
	}

	void Robot::move_robot(double xd, double yd, double zd, double thd, double phid) {
		Serial.println("in move_robot");

		//while (1) {
		//	double ang2 = joint_angle_rad(2);
		//	double ang3 = joint_angle_rad(3);
		//	double ang4 = joint_angle_rad(4);

		//	Serial.print(String(ang2));
		//	Serial.print("  ");
		//	Serial.print(String(ang3));
		//	Serial.print("  ");
		//	Serial.println(String(ang4));
		//}

		double start_t = millis()/1000.0; // movement start time, in seconds
		double travel_time = 3; // travel time, in seconds
		double end_t = start_t + travel_time; // movement end time, in seconds

		double xi, yi, zi, thi, phii; // initial task space positions tip position x y z theta phi in inches and radians

		//while (1) {
			task_space_vals start_task_space = fwkin();
			xi = start_task_space.x;
			yi = start_task_space.y;
			zi = start_task_space.z;
			thi = start_task_space.th;
			phii = start_task_space.phi;
			//Serial.println(String(xi) + "  " + String(yi) + "  " + String(zi) + "  " + String(thi) + "  " + String(phii));
		//}

		// quintic trajectory matricies
			//Matrix<double> x_coeffmat = quinticTrajectory(start_t, end_t, 0, 0, 0, 0, xi, xd);

			double x_array[6][1] = { {6}, {0},{0}, {0}, {0}, {0} };
			Matrix<double> x_coeffmat(6, 1, (double*)x_array);

			Serial.println("after x");


			//Matrix<double> z_coeffmat = quinticTrajectory(start_t, end_t, 0, 0, 0, 0, zi, zd);


			double z_array[6][1] = { {0}, {0},{0}, {1.11111}, {-0.5555556}, {0.0741} };
			Matrix<double> z_coeffmat(6, 1, (double*)z_array);

			Serial.println("after z");


			//Matrix<double> th_coeffmat = quinticTrajectory(start_t, end_t, 0, 0, 0, 0, thi, thd);

			double th_array[6][1] = { {PI}, {0},{0}, {0}, {0}, {0} };
			Matrix<double> th_coeffmat(6, 1, (double*)th_array);

			Serial.println("after th");


			double y_array[6][1] = { {0}, {0},{0}, {0}, {0}, {0} };
			Matrix<double> y_coeffmat(6, 1, (double*)y_array);

			Serial.println("after y");


			double phi_array[6][1] = { {0}, {0},{0}, {0}, {0}, {0} };
			Matrix<double> phi_coeffmat(6, 1, (double*)phi_array);
			Serial.println("after phi");


			//Matrix<double> y_coeffmat = quinticTrajectory(start_t, end_t, 0, 0, 0, 0, yi, yd);
			//Matrix<double> phi_coeffmat = quinticTrajectory(start_t, end_t, 0, 0, 0, 0, phii, phid);

		trajectory_vals xt;
		trajectory_vals yt;
		trajectory_vals zt;
		trajectory_vals tht;
		trajectory_vals phit;

		double t = 0; // time in travel, in seconds

		double last_e_q1 = 0; // last error
		double last_e_q2 = 0;
		double last_e_q3 = 0;
		double last_e_q4 = 0;
		double last_e_q5 = 0;
		double last_t = 0; // last time through controller


		Serial.println("ready to loop");

		while (millis()/1000.0 <= end_t) {

			double ang2 = joint_angle_rad(2);
			double ang3 = joint_angle_rad(3);
			double ang4 = joint_angle_rad(4);


			t = (millis()/1000.0 - start_t);

			xt = trajectory(t, x_coeffmat);
			yt = trajectory(t, y_coeffmat);
			zt = trajectory(t, z_coeffmat);
			tht = trajectory(t, th_coeffmat);
			phit = trajectory(t, phi_coeffmat);			

			controller_vals controller_return = controller(xt, yt, zt, tht, phit, &last_e_q1, &last_e_q2, &last_e_q3, &last_e_q4, &last_e_q5, &last_t);

			//move_motors(controller_return);
		}
		return;
	}

	//todo add Gravity
	controller_vals Robot::controller(trajectory_vals xd, trajectory_vals yd, trajectory_vals zd, trajectory_vals thd, trajectory_vals phid, double *last_e_q1, double *last_e_q2, double* last_e_q3, double* last_e_q4, double* last_e_q5, double *last_t) {
		joint_p_v_a qd = ikin(xd, yd, zd, thd, phid);
		//joint_p_v_a q = curr_joint_vals();

		Serial.println(String(qd.q2) + "  " + String(qd.q3) + "  " + String(qd.q4));

		jointMotor[0].setAngle(qd.q2 * (180.0 / PI));
		jointMotor[1].setAngle(qd.q3 * (180.0 / PI));
		jointMotor[2].setAngle(qd.q4 * (180.0 / PI));

		jointMotor[0].updateSpeed();
		jointMotor[1].updateSpeed();
		jointMotor[2].updateSpeed();

		/*double e_q1 = qd.q1 - q.q1;
		double e_q2 = qd.q2 - q.q2;
		double e_q3 = qd.q3 - q.q3;
		double e_q4 = qd.q4 - q.q4;
		double e_q5 = qd.q5 - q.q5;

		double de_1 = (e_q1 - *last_e_q1) / ((millis() / 1000.0) - *last_t);
		double de_2 = (e_q2 - *last_e_q2) / ((millis() / 1000.0) - *last_t);
		double de_3 = (e_q3 - *last_e_q3) / ((millis() / 1000.0) - *last_t);
		double de_4 = (e_q4 - *last_e_q4) / ((millis() / 1000.0) - *last_t);
		double de_5 = (e_q5 - *last_e_q5) / ((millis() / 1000.0) - *last_t);

		double kp1 = 10;
		double kd1 = 1;

		double kp2 = 10;
		double kd2 = 1;

		double kp3 = 10;
		double kd3 = 1;

		double kp4 = 10;
		double kd4 = 1;

		double kp5 = 10;
		double kd5 = 1;

		double tau1 = kp1 * e_q1 + kd1 * de_1;
		double tau2 = kp2 * e_q2 + kd2 * de_2;
		double tau3 = kp3 * e_q3 + kd3 * de_3;
		double tau4 = kp4 * e_q4 + kd4 * de_4;
		double tau5 = kp5 * e_q5 + kd5 * de_5;

		*last_e_q1 = e_q1;
		*last_e_q2 = e_q2;
		*last_e_q3 = e_q3;
		*last_e_q4 = e_q4;
		*last_e_q5 = e_q5;

		*last_t = millis() / 1000.0;
		*/

		controller_vals return_val;
		/*return_val.tau1 = tau1;
		return_val.tau2 = tau2;
		return_val.tau3 = tau3;
		return_val.tau4 = tau4;
		return_val.tau5 = tau5;
		*/

		return return_val;
	}

	void Robot::move_motors(controller_vals tau) {

		jointMotor[0].setSpeed(tau.tau2);
		jointMotor[1].setSpeed(tau.tau3);
		jointMotor[2].setSpeed(tau.tau4);

		return;
	}

	int Robot::map_servo_input(double input) {
		int output;
		if (input > 180) {
			output = 180;
		}
		else if (input < 0) {
			output = 0;
		}
		else {
			output = int(input);
		}

		return output;
	}

	//todo add dq and ddq
	joint_p_v_a Robot::curr_joint_vals() {
		double q1 = joint_angle_rad(1);
		double q2 = joint_angle_rad(2);
		double q3 = joint_angle_rad(3);
		double q4 = joint_angle_rad(4);
		double q5 = joint_angle_rad(5);

		double dq1 = 0;
		double dq2 = 0;
		double dq3 = 0;
		double dq4 = 0;
		double dq5 = 0;

		double ddq1 = 0;
		double ddq2 = 0;
		double ddq3 = 0;
		double ddq4 = 0;
		double ddq5 = 0;

		joint_p_v_a joint_vals;
		joint_vals.q1 = q1;
		joint_vals.q2 = q2;
		joint_vals.q3 = q3;
		joint_vals.q4 = q4;
		joint_vals.q5 = q5;
		joint_vals.dq1 = dq1;
		joint_vals.dq2 = dq2;
		joint_vals.dq3 = dq3;
		joint_vals.dq4 = dq4;
		joint_vals.dq5 = dq5;
		joint_vals.ddq1 = ddq1;
		joint_vals.ddq2 = ddq2;
		joint_vals.ddq3 = ddq3;
		joint_vals.ddq4 = ddq4;
		joint_vals.ddq5 = ddq5;

		return joint_vals;
	}

	//todo add vkin and akin
	joint_p_v_a Robot::ikin(trajectory_vals xtraj, trajectory_vals ytraj, trajectory_vals ztraj, trajectory_vals thtraj, trajectory_vals phitraj) {
		// inverse kinematics
		double l1 = 4.125;
		double l2 = 6.43;

		double P_prime_1 = xtraj.x_t - cos(atan2(ytraj.x_t, xtraj.x_t)) * l1 * cos(thtraj.x_t - PI / 2);
		double P_prime_2 = ytraj.x_t - sin(atan2(ytraj.x_t, xtraj.x_t)) * l1 * cos(thtraj.x_t - PI / 2);
		double P_prime_3 = ztraj.x_t - l1 * cos(thtraj.x_t);
		double a = pow(pow(P_prime_1, 2) + pow(P_prime_2, 2) + pow(P_prime_3 - l1, 2), 0.5);
		double A = acos((pow(a, 2) - 2 * pow(l2, 2)) / (-2 * pow(l2, 2)));
		double B = acos(a / (2 * l2));
		double C = PI - A - B;
		double alpha = asin((P_prime_3 - l1) / a);

		// desired joint positions
		double q1 = atan2(ytraj.x_t, xtraj.x_t);
		double q2 = (PI / 2) - C - alpha;
		double q3 = PI - A;
		double q4 = thtraj.x_t - q2 - q3;
		double q5 = phitraj.x_t;

		// inverse velocity kinematics

		double dx = xtraj.dx_t;
		double dy = ytraj.dx_t;
		double dz = ztraj.dx_t;
		double dtheta = thtraj.dx_t;
		double dphi = phitraj.dx_t;

		double dq1 = (dy * cos(q1) - dx * sin(q1)) / (l2 * sin(q2 + q3) + l2 * sin(q2) + l1 * sin(q2 + q3 + q4));
		double dq2 = (dx * sin(q1 + 2.0 * q2 + 2.0 * q3 + q4) - dy * cos(q1 + 2.0 * q2 + 2.0 * q3 + q4) + dy * cos(q1 + q4) + dy * cos(2.0 * q2 - q1 + 2.0 * q3 + q4) - dx * sin(q1 + q4) + dx * sin(2.0 * q2 - q1 + 2.0 * q3 + q4) + 2.0 * dz * cos(q4) + 2.0 * dz * cos(2.0 * q2 + 2.0 * q3 + q4) - dy * cos(q1 - q4) + dx * sin(q1 - q4) - 2.0 * pow(2.0, 0.5) * dtheta * l1 * sin(q4) * pow((cos(2.0 * q2 + 2.0 * q3 + 2.0 * q4) + 1.0),0.5)) / (2.0 * l2 * (sin(q2 + 2.0 * q3 + q4) - sin(q2 + q4)));
		double dq3 = -(dy * cos(2.0 * q2 - q1 + q3 + q4) - dy * cos(q1 + 2.0 * q2 + 2.0 * q3 + q4) + dx * sin(2.0 * q2 - q1 + q3 + q4) + dx * sin(q1 + 2.0 * q2 + 2.0 * q3 + q4) - dy * cos(q3 - q1 + q4) + 2.0 * dz * cos(2.0 * q2 + q3 + q4) - dx * sin(q3 - q1 + q4) + dy * cos(q1 + q4) + 2.0 * dz * cos(q3 + q4) + dy * cos(2.0 * q2 - q1 + 2.0 * q3 + q4) - dx * sin(q1 + q4) + dx * sin(2.0 * q2 - q1 + 2.0 * q3 + q4) + 2.0 * dz * cos(q4) + 2.0 * dz * cos(2.0 * q2 + 2.0 * q3 + q4) - dy * cos(q1 + 2.0 * q2 + q3 + q4) + dx * sin(q1 + 2.0 * q2 + q3 + q4) - dy * cos(q1 - q4) + dx * sin(q1 - q4) + dy * cos(q1 + q3 + q4) - dx * sin(q1 + q3 + q4) - 2.0 * pow(2,0.5) * dtheta * l1 * sin(q3 + q4) * pow((cos(2 * q2 + 2 * q3 + 2 * q4) + 1),0.5) - 2.0 * pow(2.0,0.5) * dtheta * l1 * sin(q4) * pow((cos(2.0 * q2 + 2.0 * q3 + 2.0 * q4) + 1.0),0.5)) / (2.0 * l2 * (sin(q2 + 2.0 * q3 + q4) - sin(q2 + q4)));
		double dq4 = (dy * cos(2.0 * q2 - q1 + q3 + q4) + dx * sin(2.0 * q2 - q1 + q3 + q4) - dy * cos(q3 - q1 + q4) + 2.0 * dz * cos(2.0 * q2 + q3 + q4) - dx * sin(q3 - q1 + q4) + 2.0 * dz * cos(q3 + q4) - dy * cos(q1 + 2.0 * q2 + q3 + q4) + dx * sin(q1 + 2.0 * q2 + q3 + q4) + dy * cos(q1 + q3 + q4) - dx * sin(q1 + q3 + q4) - 2.0 * pow(2,0.5) * dtheta * l1 * sin(q3 + q4) * pow((cos(2.0 * q2 + 2 * q3 + 2.0 * q4) + 1.0),0.5) - 2.0 * pow(2,0.5) * dtheta * l2 * sin(q3) * pow((cos(2.0 * q2 + 2.0 * q3 + 2.0 * q4) + 1.0),0.5)) / (2.0 * l2 * (sin(q2 + 2.0 * q3 + q4) - sin(q2 + q4)));
		double dq5 = dphi;


		// desired joint accelerations
		double ddq1 = 0;
		double ddq2 = 0;
		double ddq3 = 0;
		double ddq4 = 0;
		double ddq5 = 0;
		/*
		double ddx = xtraj.ddx_t;
		double ddy = ytraj.ddx_t;
		double ddz = ztraj.ddx_t;
		double ddtheta = thtraj.ddx_t;
		double ddphi = phitraj.ddx_t;

		double ddq1 = (ddx * l1 * cos(q2 - q1 + q3 + q4) - ddy * l1 * sin(q2 - q1 + q3 + q4) + ddx * l2 * cos(q1 - q2) + ddy * l2 * sin(q1 - q2) - ddx * l2 * cos(q1 + q2 + q3) - ddy * l2 * sin(q1 + q2 + q3) + ddx * l2 * cos(q2 - q1 + q3) - ddy * l2 * sin(q2 - q1 + q3) - ddx * l1 * cos(q1 + q2 + q3 + q4) - ddy * l1 * sin(q1 + q2 + q3 + q4) - ddx * l2 * cos(q1 + q2) - ddy * l2 * sin(q1 + q2) + dq1 * dy * l2 * cos(q2 - q1 + q3) + dq2 * dy * l2 * cos(q2 - q1 + q3) + dq3 * dy * l2 * cos(q2 - q1 + q3) + dq1 * dx * l2 * sin(q2 - q1 + q3) + dq2 * dx * l2 * sin(q2 - q1 + q3) + dq3 * dx * l2 * sin(q2 - q1 + q3) - dq1 * dy * l1 * cos(q1 + q2 + q3 + q4) + dq2 * dy * l1 * cos(q1 + q2 + q3 + q4) + dq3 * dy * l1 * cos(q1 + q2 + q3 + q4) + dq4 * dy * l1 * cos(q1 + q2 + q3 + q4) + dq1 * dx * l1 * sin(q1 + q2 + q3 + q4) - dq2 * dx * l1 * sin(q1 + q2 + q3 + q4) - dq3 * dx * l1 * sin(q1 + q2 + q3 + q4) - dq4 * dx * l1 * sin(q1 + q2 + q3 + q4) - dq1 * dy * l2 * cos(q1 + q2) + dq2 * dy * l2 * cos(q1 + q2) + dq1 * dx * l2 * sin(q1 + q2) - dq2 * dx * l2 * sin(q1 + q2) + dq1 * dy * l1 * cos(q2 - q1 + q3 + q4) + dq2 * dy * l1 * cos(q2 - q1 + q3 + q4) + dq3 * dy * l1 * cos(q2 - q1 + q3 + q4) + dq4 * dy * l1 * cos(q2 - q1 + q3 + q4) + dq1 * dx * l1 * sin(q2 - q1 + q3 + q4) + dq2 * dx * l1 * sin(q2 - q1 + q3 + q4) + dq3 * dx * l1 * sin(q2 - q1 + q3 + q4) + dq4 * dx * l1 * sin(q2 - q1 + q3 + q4) + dq1 * dy * l2 * cos(q1 - q2) + dq2 * dy * l2 * cos(q1 - q2) - dq1 * dx * l2 * sin(q1 - q2) - dq2 * dx * l2 * sin(q1 - q2) - dq1 * dy * l2 * cos(q1 + q2 + q3) + dq2 * dy * l2 * cos(q1 + q2 + q3) + dq3 * dy * l2 * cos(q1 + q2 + q3) + dq1 * dx * l2 * sin(q1 + q2 + q3) - dq2 * dx * l2 * sin(q1 + q2 + q3) - dq3 * dx * l2 * sin(q1 + q2 + q3)) / (2 * l2 ^ 2 * cos(2 * q2 + q3) + l2 ^ 2 * cos(2 * q2) + l1 ^ 2 * cos(2 * q2 + 2 * q3 + 2 * q4) + l2 ^ 2 * cos(2 * q2 + 2 * q3) - l1 ^ 2 - 2 * l2 ^ 2 - 2 * l2 ^ 2 * cos(q3) - 2 * l1 * l2 * cos(q4) + 2 * l1 * l2 * cos(2 * q2 + 2 * q3 + q4) + 2 * l1 * l2 * cos(2 * q2 + q3 + q4) - 2 * l1 * l2 * cos(q3 + q4));
		double ddq2 = (3 * ddy * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * ddz * sin(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq3 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq3 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq3 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(4 * q2 - q1 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq3 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 4 * dq3 * dz * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 4 * dq3 * dz * cos(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + 2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq3 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 12 * dq3 * dz * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 - q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq3 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 12 * dq3 * dz * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq3 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(2 * q2 - q1 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(2 * q2 - q1 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq3 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 4 * q2 + 5 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(q3 - q4) + 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(q3 - q4) + 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(q3 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 3 * q3 + 3 * q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 3 * q3 + 3 * q4) + 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 3 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 3 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 3 * q3 + 5 * q4) + 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 5 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 3 * q3 + 5 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 5 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 5 * q3 + 5 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 5 * q3 + 5 * q4) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + q3 + q4) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + q3 + q4) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 6 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(q3 + q4) + 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(q3 + q4) - 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + q3 + 3 * q4) + 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 3 * q3 + q4) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + q3 + 3 * q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 3 * q3 + q4) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 3 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1)) / (2 * l2 * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) * (cos(3 * q2 + 5 * q3 + 3 * q4) - 2 * cos(3 * q2 + 3 * q3 + 3 * q4) - 6 * cos(q2 + q3 + q4) + 3 * cos(q2 - q3 + q4) + 3 * cos(q2 + 3 * q3 + q4) + cos(3 * q2 + q3 + 3 * q4)));
		double ddq3 = -(3 * ddy * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * ddz * sin(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * ddz * sin(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(q1 + 2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(2 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(4 * q2 - q1 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dx * sin(q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(2 * q2 - q1 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dx * sin(q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(q1 + 2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(q1 + 4 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 2 * q3 + 3 * q4) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 2 * q3 + 3 * q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 3 * q3 + 3 * q4) + 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 3 * q3 + 3 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 5 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 5 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 5 * q3 + 5 * q4) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 3 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + q3 + q4) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(q3 + q4) - 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 6 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(q4) - 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(q4) + 4 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 2 * q3 + q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 2 * q3 + q4) + 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1)) / (2 * l2 * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) * (cos(3 * q2 + 2 * q3 + 3 * q4) - 6 * cos(q2 + q3 + q4) - 2 * cos(3 * q2 + 3 * q3 + 3 * q4) + cos(3 * q2 + 4 * q3 + 3 * q4) + 3 * cos(q2 + 2 * q3 + q4) + 3 * cos(q2 + q4)));
		double ddq4 = (3 * ddy * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddy * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * ddx * cos(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * ddz * sin(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddx * cos(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddx * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddy * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * ddz * sin(2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * ddz * sin(2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + ddy * sin(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - ddy * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * ddz * sin(4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * ddz * sin(4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * ddx * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dx * sin(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq2 * dz * cos(2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 4 * 2 ^ (1 / 2) * ddtheta * l2 * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 3 * dq1 * dx * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(q1 - q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(2 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(2 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(4 * q2 - q1 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(4 * q2 - q1 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq2 * dz * cos(4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 2 * dq2 * dz * cos(4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 2 * dq3 * dz * cos(4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(q1 + 2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(q1 + 2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dx * sin(q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dy * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dy * cos(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dy * cos(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dy * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dy * cos(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dy * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dy * cos(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dx * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dx * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dx * sin(q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq1 * dx * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dx * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dx * sin(2 * q2 - q1 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq1 * dx * sin(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(q1 + 2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq2 * dx * sin(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq1 * dx * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq3 * dx * sin(q1 + 2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(q1 + 4 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - dq2 * dx * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + dq3 * dx * sin(q1 + 4 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq1 * dy * cos(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq2 * dy * cos(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq2 * dy * cos(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 3 * dq3 * dy * cos(2 * q2 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 3 * dq3 * dy * cos(2 * q3 - q1 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 6 * dq2 * dz * cos(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) + 6 * dq3 * dz * cos(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + q4) + 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q3 + q4) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 8 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 2 * q3 + 3 * q4) - 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 2 * q3 + 3 * q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 4 * q3 + 3 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 2 * q3 + 3 * q4) + 2 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 3 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 3 * q4) - 2 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 5 * q4) - 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 4 * q3 + 5 * q4) + 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(4 * q2 + 6 * q3 + 5 * q4) - 4 * 2 ^ (1 / 2) * ddtheta * l2 * cos(2 * q3) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 2 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 4 * 2 ^ (1 / 2) * ddtheta * l2 * cos(2 * q2 + 2 * q3 + 2 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 4 * q3 + 3 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 2 * 2 ^ (1 / 2) * ddtheta * l2 * cos(2 * q2 + 4 * q3 + 2 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 2 * 2 ^ (1 / 2) * ddtheta * l2 * cos(2 * q2 + 2 * q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) - 12 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(q4) - 6 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(q4) + 8 * 2 ^ (1 / 2) * dq3 * dtheta * l1 * sin(2 * q2 + 2 * q3 + q4) + 4 * 2 ^ (1 / 2) * dq4 * dtheta * l1 * sin(2 * q2 + 2 * q3 + q4) + 4 * 2 ^ (1 / 2) * ddtheta * l1 * cos(q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) + 2 * 2 ^ (1 / 2) * ddtheta * l1 * cos(2 * q2 + 2 * q3 + q4) * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1)) / (2 * l2 * (cos(2 * q2 + 2 * q3 + 2 * q4) + 1) ^ (1 / 2) * (cos(3 * q2 + 5 * q3 + 3 * q4) - 2 * cos(3 * q2 + 3 * q3 + 3 * q4) - 6 * cos(q2 + q3 + q4) + 3 * cos(q2 - q3 + q4) + 3 * cos(q2 + 3 * q3 + q4) + cos(3 * q2 + q3 + 3 * q4)));
		double ddq5 = ddphi
		*/

		joint_p_v_a desired;
		desired.q1 = q1;
		desired.q2 = q2;
		desired.q3 = q3;
		desired.q4 = q4;
		desired.q5 = q5;
		desired.dq1 = dq1;
		desired.dq2 = dq2;
		desired.dq3 = dq3;
		desired.dq4 = dq4;
		desired.dq5 = dq5;
		desired.ddq1 = ddq1;
		desired.ddq2 = ddq2;
		desired.ddq3 = ddq3;
		desired.ddq4 = ddq4;
		desired.ddq5 = ddq5;

		return desired;
	}

	//reads joint values and returns task space x, y, z, theta, phi values of end effector
	task_space_vals Robot::fwkin() {

		double q1 = joint_angle_rad(1);
		double q2 = joint_angle_rad(2);
		double q3 = joint_angle_rad(3);
		double q4 = joint_angle_rad(4);
		double q5 = joint_angle_rad(5);

		return forward_kinematics(q1, q2, q3, q4, q5);
	}

	//returns joint angle, in radians, of specified joint. 
	double Robot::joint_angle_rad(int joint) {
		double rad1 = 0;

		double deg2 = jointMotor[0].getAngleDegrees();
		double rad2 = deg2 *(PI / 180.0);
		
		double deg3 = jointMotor[1].getAngleDegrees();
		double rad3 = deg3 *(PI / 180.0);

		double deg4 = jointMotor[2].getAngleDegrees();
		double rad4 = deg4 *(PI / 180.0);

		double rad5 = 0;

		double q;

		switch (joint)
		{
		case 1:
			q = rad1;
			break;
		case 2:
			q = rad2;
			break;
		case 3:
			q = rad3;
			break;
		case 4:
			q = rad4;
			break;
		case 5:
			q = rad5;
			break;
		}
		
		return q;
	}
