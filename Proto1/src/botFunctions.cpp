#include "../include/botFunctions.h"

bool intakeToggle = 0;
bool flywheelToggle = 0;
bool compressToggle = 0;

void toggleIntake()
{
    if(intakeToggle == 0)
        intakeToggle = 1;
    else
        intakeToggle = 0;
}

void toggleFlywheels()
{
    if(flywheelToggle == 0)
        flywheelToggle = 1;
    else
        flywheelToggle = 0;
}

void toggleCompress()
{
    if(compressToggle == 0)
        compressToggle = 1;
    else
        compressToggle = 0;
}

void intakeDrop(pros::ADIDigitalOut left, pros::ADIDigitalOut right, bool value) {
    left.set_value(value);
    right.set_value(value);
}

void intakeSlow(pros::Motor front, pros::Motor mid, pros::Motor top, pros::Motor rear) {
    front.move(127);
    mid.move(-40);
    top.move(-40);
    rear.brake();
}

void scoreTop(pros::Motor front, pros::Motor mid, pros::Motor top, pros::Motor rear) {
    front.move(127);
    mid.move(-127);
    top.move(-127);
    rear.move(127);
}

void scoreMid(pros::Motor front, pros::Motor mid, pros::Motor top, pros::Motor rear) {
    front.move(127);
    mid.move(-127);
    top.move(-127);
    rear.move(-127);
}

void outTake(pros::Motor front, pros::Motor mid, pros::Motor top, pros::Motor rear) {
    front.move(-127);
    mid.move(127);
    top.move(127);
    rear.move(127);
}

void intakeBrake(pros::Motor front, pros::Motor mid, pros::Motor top, pros::Motor rear) {
    front.brake();
    mid.brake();
    top.brake();
    rear.brake();
}

void driveLoop(Mines::MinesMotorGroup leftMotorGroup, Mines::MinesMotorGroup rightMotorGroup, double leftVelocity, double rightVelocity)
{
    leftMotorGroup.move(leftVelocity);
    rightMotorGroup.move(rightVelocity);
}

