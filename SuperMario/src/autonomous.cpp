#include "main.h"

void leftAutonRoute(void)
{

}

void rightAutonRoute(void)
{

}

void leftRouteSkills(void)
{

}

void rightRouteSkills(void)
{

}


void autonomous() 
{
    unLoader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    drive.capVoltage(60);

    drive.setDrivePID(.001, 0.8 , .04, 10.0);

    drive.setTurnPID(1.0, 1, 0, 12.5);
    
    drive.driveDistance(-49.38, 5000, 200);
    drive.turnFor(47.7, 2, 200);
    drive.driveDistance(-(10.5 - 7.25), 5000, 200);

    leftMotors.brake();
    rightMotors.brake();

    hood1.toggle();
    hood2a.toggle();
    hood2b.toggle();

    intakeMotors.move(127);
    pros::delay(1500);

    intakeMotors.brake();
    //hood1.toggle();
    //hood2a.toggle();
    //hood2b.toggle();

    drive.driveDistance((10.5 - 7.5), 5000, 200);

    drive.turnFor(75.5, 3, 200);
    intakeMotors.move(127);
    drive.driveDistance(17.4, 5000, 200);
    pros::delay(1500);

    intakeMotors.brake();

    drive.driveDistance(-17.4, 5000, 200);
    drive.turnFor(-75.5, 3, 200);

    drive.driveDistance(-(10.5 - 7.5), 5000, 200);

    hood1.toggle();
    hood2a.toggle();
    hood2b.toggle();

    intakeMotors.move(127);
    pros::delay(1500);

    intakeMotors.brake();
    
    rightMotors.brake();
    leftMotors.brake();
    
}