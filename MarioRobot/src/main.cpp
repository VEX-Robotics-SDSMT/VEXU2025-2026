#include "main.h"
#include "PID.h"


void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}


void opcontrol() 
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor motor(1, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);

	Mines::PIDTuning tuning;
	tuning.kP = 1.0;
	tuning.kI = 0.5;
	tuning.kD = 0;

	tuning.integralClamp = 100.0;
	tuning.maxOutput = 127.0;
	tuning.minOutput = -127.0;

	Mines::PID pid(tuning);

	

	while (true) 
	{
		pros::delay(20);
	}
}