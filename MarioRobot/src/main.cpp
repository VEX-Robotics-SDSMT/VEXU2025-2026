#include "api.h"


void on_center_button() 
{
}


void initialize() 
{
}

void disabled() {}


void competition_initialize() {}

void autonomous() {}


void opcontrol() 
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor motor(1);

	motor.move(127);


	while (true) 
	{
		pros::delay(20);
	}
}