#include "main.h"
#include "PID.h"


void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}


void opcontrol() 
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor motor(1,pros::v5::MotorGears::blue,pros::v5::MotorEncoderUnits::degrees);
	
	Mines::PIDTuning tuning;
	tuning.kP = 0.5;
	tuning.kI = 0.001;
	tuning.kD = 0.5;
	Mines::PID pid(tuning, .5, 10, 100000);

	pid.setTarget(-2000.0);

	while (true) 
	{
		while(pid)
		{
			motor.move(pid.update(motor.get_position()));
			pros::delay(20);
		}
		motor.move(0);

		pros::delay(20);
	}
}