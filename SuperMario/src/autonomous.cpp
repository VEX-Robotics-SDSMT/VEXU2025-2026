#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

static constexpr uint8_t RIGHT = -1;
static constexpr uint8_t LEFT = 1;

static void _grabMiddleBallLeftAutoRoute(Mines::Drivetrain& drive)
{
    intakeMotors.move(127);
	drive.driveDistance(7.673, 5000, 160);
    pros::delay(300);
    intakeMotors.move(30);

    drive.driveDistance(-7.673, 5000, 160);
    drive.turnTo(90 * LEFT, .5 , 180);
}

void leftAutonRoute(void){
	Mines::Drivetrain drive(&leftMotors, &rightMotors, &enc, &imu);
	drive.capVoltage(100);

    //===================================
    //PID SetUp
    //===================================
	drive.setDrivePID(.0009, 1, 0.0, 3.5);
	drive.setTurnPID(1.8, 0.6, 0.0, 1.0);


    //===================================
    //Auto
    //===================================
	drive.driveDistance(15.7, 5000, 160);
	drive.turnTo(49.027 * RIGHT, .5 , 180);

    intakeMotors.move(127);
    drive.driveDistance(34.94, 5000, 160);
    pros::delay(300);
    intakeMotors.move(30);

    drive.driveDistance(-7.371, 5000, 160);
    drive.turnTo(49.027 * LEFT, .5 , 180);

    //Run the same code for each ball
    _grabMiddleBallLeftAutoRoute(drive);
    
    drive.driveDistance(7.2, 5000, 160);
    
    drive.turnTo(90 * RIGHT, .5 , 180);
    _grabMiddleBallLeftAutoRoute(drive);

    drive.driveDistance(7.342, 5000, 160);
    drive.turnTo(90 * RIGHT, .5 , 180);
    _grabMiddleBallLeftAutoRoute(drive);

    drive.turnTo(5.774 * RIGHT, .5 , 180);
    drive.driveDistance(9.528, 5000, 160);

    intakeMotors.move(0);

    drive.turnTo(39.226 * RIGHT, .5 , 180);
    //prepare to score in goal
    hood.extend();
    drive.driveDistance(7.5, 5000, 160);

    //Outake all balls in the intak
    intakeMotors.move(127);

	rightMotors.move(0);
	leftMotors.move(0);
    intakeMotors.move(0);
}


void autonomous() {
    leftAutonRoute();
}