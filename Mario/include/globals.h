#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "api.h"
#include "MinesMotorGroup.h"

#define INERTIAL_SENSOR 16
#define COLOR_SENSOR 14
#define VEX_GPS 21 //unused so far
#define ENCODER_TOP 5
#define ENCODER_BOTTOM 6

#define L_FRONT 3//
#define L_MID 2//
#define L_TREAR 4//
#define L_BREAR 1//
#define R_FRONT 7//
#define R_MID 8//
#define R_TREAR 6//
#define R_BREAR 5//

#define I_FRONT 11//
#define I_BOT 12//
#define I_MID 13//
#define I_TOP1 20//
#define I_TOP2 19//
#define I_BACK 18//

#define ARM_MOTOR 17

#define I_REAR 9

#define TESTPNEU 1//
#define TESTPNEU1 8//

#define INTAKE_MOTOR_GEARSET redGearbox
#define FLYWHEELS_MOTOR_GEARSET blueGearbox
#define ROLLER_MOTOR_GEARSET greenGearbox

extern pros::Controller MasterController;

extern pros::Imu intertialSensor;
extern pros::Optical colorSensor;
extern pros::ADIEncoder driveEncoder;
extern pros::GPS gps;

extern pros::Motor leftFront;
extern pros::Motor leftRearTop;
extern pros::Motor leftRearBot;
extern pros::Motor leftMid;
extern pros::Motor rightFront;
extern pros::Motor rightRearTop;
extern pros::Motor rightMid;
extern pros::Motor rightRearBot;
extern pros::Motor IntakeBack;
extern pros::Motor IntakeMid;
extern pros::Motor IntakeFront;
extern pros::Motor IntakeTop1;
extern pros::Motor IntakeTop2;
extern pros::Motor IntakeBot;
extern pros::Motor Arm;

extern std::vector<pros::Motor> leftDriveVector;
extern std::vector<pros::Motor> rightDriveVector;
extern Mines::MinesMotorGroup leftDriveMotors;
extern Mines::MinesMotorGroup rightDriveMotors;

extern pros::ADIDigitalOut intakeDropL;
extern pros::ADIDigitalOut intakeDropR;

enum Color { red, blue, purple };
extern pros::Motor string;

extern double axisPercentBlue;
extern double axisPercentGreen;
extern double axisPercentRed;
extern double RED_MAX;
extern double RED_MIN;
extern double BLUE_MAX;
extern double BLUE_MIN;
extern double COLOR_MAX;
extern double COLOR_MIN;
extern int blueGearbox;
extern int greenGearbox;
extern int redGearbox;

extern bool skills;
extern bool red_team;

extern uint8_t RED_GOAL_SIG_ID;
extern uint8_t BLUE_GOAL_SIG_ID;

extern int requiredColorLoops;
extern const double ROLLER_TIMEOUT;

#define buttonUp pros::E_CONTROLLER_DIGITAL_UP
#define buttonDown pros::E_CONTROLLER_DIGITAL_DOWN
#define buttonLeft pros::E_CONTROLLER_DIGITAL_LEFT
#define buttonRight pros::E_CONTROLLER_DIGITAL_RIGHT
#define buttonX pros::E_CONTROLLER_DIGITAL_X
#define buttonY pros::E_CONTROLLER_DIGITAL_Y
#define buttonA pros::E_CONTROLLER_DIGITAL_A
#define buttonB pros::E_CONTROLLER_DIGITAL_B
#define buttonL1 pros::E_CONTROLLER_DIGITAL_L1
#define buttonL2 pros::E_CONTROLLER_DIGITAL_L2
#define buttonR1 pros::E_CONTROLLER_DIGITAL_R1
#define buttonR2 pros::E_CONTROLLER_DIGITAL_R2
#define axisLeftY pros::E_CONTROLLER_ANALOG_LEFT_Y
#define axisLeftX pros::E_CONTROLLER_ANALOG_LEFT_X
#define axisRightY pros::E_CONTROLLER_ANALOG_RIGHT_Y
#define axisRightX pros::E_CONTROLLER_ANALOG_RIGHT_X

#endif