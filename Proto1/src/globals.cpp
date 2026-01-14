#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::Vision vision(VISION_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP, ENCODER_BOTTOM, false);
pros::GPS gps(VEX_GPS);

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftMidFront(L_MFRONT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftMidRear(L_MREAR, pros::E_MOTOR_GEARSET_18, true);
pros::Motor leftRear(L_REAR, pros::E_MOTOR_GEARSET_18, true);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightMidFront(R_MFRONT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightMidRear(R_MREAR, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightRear(R_REAR, pros::E_MOTOR_GEARSET_18, false);

pros::Motor IntakeRear(I_REAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeMid(I_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor IntakeFront(I_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor IntakeTop(I_TOP, pros::E_MOTOR_GEARSET_06, false);

pros::ADIDigitalOut intakeDropR(TESTPNEU);
pros::ADIDigitalOut intakeDropL(TESTPNEU1);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftMidFront, leftMidRear, leftRear};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightMidFront, rightMidRear, rightRear};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);

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

bool skills = true;
bool lower = true;