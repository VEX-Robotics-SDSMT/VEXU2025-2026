#include "./../include/MinesMotorGroup.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include <vector>

using namespace Mines;

//contructor for passing an array of motors
/*EX call:
    * rightDriveArray[2] = {rightFront, rightRear};
    * Mines::MinesMotorGroup rightDriveMotors(rightDriveArray, 2);
*/
MinesMotorGroup::MinesMotorGroup(pros::Motor motors[], int numMotors)
{
    if (!initialized)
    {
        for(int i = 0; i < numMotors; i++)
        {
            motorVector.push_back(motors[i]);
        }

        initialized = true;
        size = motorVector.size();
    }
}

//constructor for passing a vector of motors
/*EX call:
    * std::vector<pros::Motor> rightDriveVector = {rightFront, rightRear};
    * Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
*/
MinesMotorGroup::MinesMotorGroup(std::vector<pros::Motor> &motors)
{
    if (!initialized)
    {
        for(int i = 0; i < motors.size() ; i++)
        {
            motorVector.push_back(motors[i]);
        }

        initialized = true;
        size = motorVector.size();
    }
}

//tells motors in motor group to spin, higher voltage = higher speed
//voltage range -127 to +127
/*EX call:
    * rightDriveMotors.move(127) //spin forward
    * rightDriveMotors.move(-127) //spin backward
*/
int MinesMotorGroup::move(const int voltage)
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].move(voltage);
    }
    return 0;
}

//tells motors in group to spin at velocity
//motor velocity from Red: +-100 : , Green: +-200, or Blue: +-600 depending on the motor’s color gearset
/*EX call:
    * rightDriveMotors.move_velocity(100) //spin forward
    * rightDriveMotors.move_velocity(-100) //spin backward
*/
int MinesMotorGroup::moveVelocity(const int velocity)
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].move_velocity(velocity);
    }
    return 0;
}

//tells motors in motor group to spin, higher voltage = higher speed
//voltage range -127 to +127
/*EX call:
    * rightDriveMotors.moveVoltage(127) //spin forward
    * rightDriveMotors.moveVoltage(-127) //spin backward
*/
int MinesMotorGroup::moveVoltage(const int voltage)
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].move_voltage(voltage);
    }
    return 0;
}

//tells motors in motor group to stop
/*EX call:
    * rightDriveMotors.brake()
*/
int MinesMotorGroup::brake()
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].brake();
    }
    return 0;
}

//sets the absolute zero position of the motors to its current position
/*EX call:
    * rightDriveMotors.tarePosition()
*/
int MinesMotorGroup::tarePosition()
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].tare_position();
    }
    return 0;
}

//sets the brake mode of the motors to a different mode
/*MODES:
    * pros::E_MOTOR_BRAKE_COAST //Motor coasts after calling brake()
    * pros::E_MOTOR_BRAKE_BRAKE //Motor short brakes ^^
    * pros::E_MOTOR_BRAKE_HOLD  //Motor will constant brake **WARNING** using this can over-exert motors
*/
int MinesMotorGroup::setBrakeMode(pros::motor_brake_mode_e mode)
{
    for(int i = 0; i < motorVector.size(); i++)
    {
        motorVector[i].set_brake_mode(mode);
    }
    return 0;
}

//returns the average current velocity of the motors
/*EX call:
    * double velocity = rightDriveMotors.getActualVelocity()
*/
double MinesMotorGroup::getActualVelocity()
{
    double average = 0;
    for(int i = 0; i < motorVector.size(); i++)
    {
        average += motorVector[i].get_actual_velocity();
    }
    return average / motorVector.size();
}

//returns the average current absolute position in its encoder ticks
/*EX call:
    * double ticks = rightDriveMotors.getPosition()
*/
double MinesMotorGroup::getPosition()
{
    double average = 0;
    for(int i = 0; i < motorVector.size(); i++)
    {
        average += motorVector[i].get_position();
    }
    return average / motorVector.size();
}

//returns the max velocity of the motors based on gearset
/*EX call:
    * double velocity = rightDriveMotors.getMaxVelocity()
*/
double MinesMotorGroup::getMaxVelocity()
{
    if(motorVector[0].get_gearing()== pros::E_MOTOR_GEARSET_36)
    {
        return 100;
    }
    if(motorVector[0].get_gearing() == pros::E_MOTOR_GEARSET_18)
    {
        return 200;
    }
    if(motorVector[0].get_gearing() == pros::E_MOTOR_GEARSET_06)
    {
        return 600;
    }
    return 0;
}
