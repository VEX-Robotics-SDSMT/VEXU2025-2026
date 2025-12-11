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
	tuning.kI = 0.0;
	tuning.kD = 0.0;
	Mines::PID pid(tuning, .005, 10, 100000);

	pid.setTarget(2000.0);

	while (true) 
	{
		motor.move(pid.update(motor.get_position()));

		pros::delay(20);
	}
}