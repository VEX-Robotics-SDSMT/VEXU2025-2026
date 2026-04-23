#include "../include/botFunctions.h"

bool intakeToggle = 0;
bool flywheelToggle = 0;
bool compressToggle = 0;

void toggleIntake()
{
    if (intakeToggle == 0)
        intakeToggle = 1;
    else
        intakeToggle = 0;
}

void toggleFlywheels()
{
    if (flywheelToggle == 0)
        flywheelToggle = 1;
    else
        flywheelToggle = 0;
}

void toggleCompress()
{
    if (compressToggle == 0)
        compressToggle = 1;
    else
        compressToggle = 0;
}

void intakeDrop(pros::ADIDigitalOut left, pros::ADIDigitalOut right, bool value)
{
    left.set_value(value);
    right.set_value(value);
}

void intakeScoreTop()
{
    IntakeFront.move(127);
    IntakeTop.move(127);
}

void intakeOut()
{
    IntakeFront.move(-127);
    IntakeTop.move(-127);
}

void intakeBrake()
{
    IntakeFront.brake();
    IntakeTop.brake();
}

void driveLoop(Mines::MinesMotorGroup leftMotorGroup, Mines::MinesMotorGroup rightMotorGroup, double leftVelocity, double rightVelocity)
{
    leftMotorGroup.move(leftVelocity);
    rightMotorGroup.move(rightVelocity);
}
