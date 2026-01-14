#include "main.h"
#include "PID.h"
#include "test.h"

pros::MotorGroup left_motors({-3,-2,-1,4}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::MotorGroup right_motors({5,7,8,-6}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

pros::MotorGroup topIntakeMotors({-19,20}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor sorterMotor(12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor botomIntakeMotor(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor middleIntakeMotor(13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor basketOuttakeMotor(18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}


void opcontrol() 
{
	testFunction();
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	topIntakeMotors.move(-127);
	//botomIntakeMotor.move(127);
	middleIntakeMotor.move(-127);
	basketOuttakeMotor.move(127);
	//sorterMotor.move(127);

	while (true) 
	{
		left_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		right_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		pros::delay(20);
	}
}