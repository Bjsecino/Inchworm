/*
    Name:       Inchworm.ino
    Created:	11/15/2019 10:42:45 AM
    Author:     DESKTOP-NTHC4K9\Benjamin Secino
*/


#include "Robot.h"

Robot robot;

double home_x = 5;
double home_y = 0;
double home_z = 0;
double up_z = 5;
double out_x = 10;

void setup()
{
}

void loop()
{
	robot.move_robot(home_x, home_y, home_z, PI, 0); // home
	robot.move_robot(home_x, home_y, up_z, PI, 0); // up
	robot.move_robot(out_x, home_y, up_z, PI, 0); // out
	robot.move_robot(out_x, home_y, home_z, PI, 0); // down
}
