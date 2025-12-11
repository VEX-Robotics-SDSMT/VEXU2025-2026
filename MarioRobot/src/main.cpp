#include "main.h"
#include "PID.h"


void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}


void opcontrol() 
{
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::MotorGroup leftMotors({-11,-5,-20,-16},pros::v5::MotorGears::green,pros::v5::MotorEncoderUnits::degrees);
	pros::MotorGroup rightMotors({3,18,1,10},pros::v5::MotorGears::green,pros::v5::MotorEncoderUnits::degrees);

	Mines::PIDTuning tuning;
	tuning.kP = 1.0;
	tuning.kI = 0.01;
	tuning.kD = 0.5;

	tuning.maxIntegral = 100.0;
	
	Mines::PID pid(tuning, 1, 100, 10000);

	double target = 360.0;
	pid.setTarget(target);

	while (true) 
	{
		while(pid)
		{
			double result = pid.update(leftMotors.get_position());
			leftMotors.move(result);
			rightMotors.move(result);
			pros::delay(20);
		}
		leftMotors.move(0);
		rightMotors.move(0);

		pros::delay(20);
	}
}