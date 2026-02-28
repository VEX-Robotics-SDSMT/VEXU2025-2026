#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::Vision vision(VISION_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP, ENCODER_BOTTOM, false);
pros::GPS gps(VEX_GPS);

pros::Motor leftFrontTop(L_FRONT_T, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftFrontBot(L_FRONT_B, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftRearTop(L_REAR_T, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftRearBot(L_REAR_B, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFrontTop(R_FRONT_T, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFrontBot(R_FRONT_B, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightRearTop(R_REAR_T, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightRearBot(R_REAR_B, pros::E_MOTOR_GEARSET_06, false);

pros::Motor IntakeRear(I_BACK, pros::E_MOTOR_GEARSET_06, true);
pros::Motor IntakeMid(I_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeBot(I_BOT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeTop(I_TOP, pros::E_MOTOR_GEARSET_18, true);
pros::Motor IntakeFrontL(I_FRONTL, pros::E_MOTOR_GEARSET_18, true);
pros::Motor IntakeFrontR(I_FRONTR, pros::E_MOTOR_GEARSET_18, false);


pros::ADIDigitalOut intakeLiftR(LIFT1);
pros::ADIDigitalOut intakeLiftL(LIFT2);
pros::ADIDigitalOut wing(WING);

std::vector<pros::Motor> leftDriveVector = {leftFrontTop, leftFrontBot, leftRearTop, leftRearBot};
std::vector<pros::Motor> rightDriveVector = {rightFrontTop, rightFrontBot, rightRearTop, rightRearBot};
std::vector<pros::Motor> intakeWheelsVector = {IntakeFrontL, IntakeFrontR};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
Mines::MinesMotorGroup intakeWheels(intakeWheelsVector);

double axisPercentBlue = 600.0 / 127;
double axisPercentGreen = 200.0 / 127;
double axisPercentRed = 100.0 / 127;
int blueGearbox = 600;
int greenGearbox = 200;
int redGearbox = 100;


uint8_t RED_GOAL_SIG_ID = 1;
uint8_t BLUE_GOAL_SIG_ID = 2;

int requiredColorLoops = 3;
const double ROLLER_TIMEOUT = 3000;

bool skills = false;
bool lower = true;