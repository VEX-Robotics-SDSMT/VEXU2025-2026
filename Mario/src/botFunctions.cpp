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

void intakeScoreTop(bool slower)
{
    if (slower)
    {
        IntakeFront.move(-80);
        IntakeMid.move(-80);
        IntakeBot.move(80);
        IntakeTop1.move(80);
        IntakeTop2.move(80);
        IntakeBack.move(80);
    }
    else
    {
        IntakeFront.move(-127);
        IntakeMid.move(-127);
        IntakeBot.move(127);
        IntakeTop1.move(127);
        IntakeTop2.move(127);
        IntakeBack.move(127);
    }
}
void intakeBasket()
{
    double color = colorSensor.get_hue();
    if (color < COLOR_MAX && color > COLOR_MIN)
    {
        IntakeBot.move(127);
    }
    else
    {
        IntakeBot.move(-127);
    }
    IntakeFront.move(-127);
    IntakeMid.move(-127);
    IntakeTop1.move(127);
    IntakeTop2.move(-127);
    IntakeBack.move(-127);
}
void intakeOut()
{
    double color = colorSensor.get_hue();
    if (color < COLOR_MAX && color > COLOR_MIN)
    {
        IntakeBot.move(127);
        IntakeFront.move(-127);
    }
    else
    {
        IntakeBot.move(-20);
        IntakeFront.move(127);
    }
    IntakeMid.move(127);
    IntakeTop1.move(127);
    IntakeTop2.move(127);
    IntakeBack.move(127);
}
void intakeScoreMid()
{
    IntakeFront.move(-127);
    IntakeMid.move(127);
    IntakeBot.move(127);
    IntakeTop1.move(127);
    IntakeTop2.move(127);
    IntakeBack.move(127);
}
void intakeBrake()
{
    IntakeFront.brake();
    IntakeMid.brake();
    ;
    IntakeBot.brake();
    IntakeTop1.brake();
    IntakeTop2.brake();
    IntakeBack.brake();
}

void driveLoop(Mines::MinesMotorGroup leftMotorGroup, Mines::MinesMotorGroup rightMotorGroup, double leftVelocity, double rightVelocity)
{
    leftMotorGroup.move(leftVelocity);
    rightMotorGroup.move(rightVelocity);
}
