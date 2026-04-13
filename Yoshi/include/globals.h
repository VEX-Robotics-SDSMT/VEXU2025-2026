#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "api.h"
#include "MinesMotorGroup.h"

#define INERTIAL_SENSOR 17
#define VISION_SENSOR 22 //unused so far
#define VEX_GPS 21 //unused so far
#define ENCODER_TOP 1 //unused so far
#define ENCODER_BOTTOM 2 //unused so far

#define L_FRONT 20
#define L_FRONTM 3
#define L_REARM 2
#define L_REAR 1
#define R_FRONT 10
#define R_FRONTM 13
#define R_REARM 12
#define R_REAR 11

#define I_BOT 21
#define I_BACK 16
#define I_MID 19
#define I_TOP 18
#define I_SCORE 16
#define I_FRONTL 9
#define I_FRONTR 5

#define LIFT1 8
#define LIFT2 7
#define WING 6

#define INTAKE_MOTOR_GEARSET redGearbox
#define FLYWHEELS_MOTOR_GEARSET blueGearbox
#define ROLLER_MOTOR_GEARSET greenGearbox

extern pros::Controller MasterController;

extern pros::Imu intertialSensor;
extern pros::Vision vision;
extern pros::ADIEncoder driveEncoder;
extern pros::GPS gps;

extern pros::Motor leftFrontTop;
extern pros::Motor leftFrontBot;
extern pros::Motor leftRearTop;
extern pros::Motor leftRearBot;
extern pros::Motor rightFrontTop;
extern pros::Motor rightFrontBot;
extern pros::Motor rightRearTop;
extern pros::Motor rightRearBot;

extern pros::Motor IntakeRear;
extern pros::Motor IntakeBot;
extern pros::Motor IntakeMid;
extern pros::Motor IntakeTop;
extern pros::Motor IntakeScore;
extern pros::Motor IntakeFrontR;
extern pros::Motor IntakeLeftR;

extern std::vector<pros::Motor> leftDriveVector;
extern std::vector<pros::Motor> rightDriveVector;
extern std::vector<pros::Motor> intakeWheelVector;
extern Mines::MinesMotorGroup leftDriveMotors;
extern Mines::MinesMotorGroup rightDriveMotors;
extern Mines::MinesMotorGroup intakeWheels;

extern pros::ADIDigitalOut intakeLiftL;
extern pros::ADIDigitalOut intakeLiftR;
extern pros::ADIDigitalOut wing;

enum Color { red, blue, purple };
extern pros::Motor string;

extern double axisPercentBlue;
extern double axisPercentGreen;
extern double axisPercentRed;
extern int blueGearbox;
extern int greenGearbox;
extern int redGearbox;

extern bool skills;
extern bool lower;

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