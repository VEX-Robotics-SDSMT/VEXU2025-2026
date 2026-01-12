#include "main.h"
#include "PID.h"

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


pros::MotorGroup leftMotors({L_FRONT,L_MID,-L_TREAR,L_BREAR}, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::MotorGroup rightMotors({-R_FRONT,-R_MID,R_TREAR,-R_BREAR}, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);

pros::Motor IntakeBack(I_BACK, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::Motor IntakeBot(I_BOT, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::Motor IntakeMid(I_MID, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::Motor IntakeFront(I_FRONT, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::Motor IntakeTop1(I_TOP1, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);
pros::Motor IntakeTop2(I_TOP2, pros::v5::MotorGears::blue, pros::MotorEncoderUnits::deg);

pros::Controller master(pros::E_CONTROLLER_MASTER);


void intakeScoreTop(const pros::Motor& front, const pros::Motor& mid, const pros::Motor& top1, const pros::Motor& top2, const pros::Motor& back, const pros::Motor& bot) {
    front.move(-127);
    mid.move(-127);
    bot.move(127);
    top1.move(127);
    top2.move(127);
    back.move(127);
}
void intakeBasket(pros::Motor front, const pros::Motor& mid, const pros::Motor& bot, const pros::Motor& top1, const pros::Motor& top2, const pros::Motor& back) {
    front.move(-127);
    mid.move(-127);
    bot.move(-127);
    top1.move(127);
    top2.move(-127);
    back.move(-127);
}
void intakeOut(const pros::Motor& front, const pros::Motor& mid, const pros::Motor& bot, const pros::Motor& top1, const pros::Motor& top2, const pros::Motor& back) {
    front.move(127);
    mid.move(127);
    bot.move(-20);
    top1.move(127);
    top2.move(127);
    back.move(127);
}
void intakeScoreMid(const pros::Motor& front, const pros::Motor& mid, const pros::Motor& bot, const pros::Motor& top1, const pros::Motor& top2, const pros::Motor& back) {
    front.move(-127);
    mid.move(127);
    bot.move(127);
    top1.move(127);
    top2.move(127);
    back.move(127);
}
void intakeBrake(const pros::Motor& front, const pros::Motor& mid, const pros::Motor& bot, const pros::Motor& top1, const pros::Motor& top2, const pros::Motor& back) {
    front.brake();
    mid.brake();;
    bot.brake();
    top1.brake();
    top2.brake();
    back.brake();
}

void initialize() 
{

}

void disabled() 
{

}

void competition_initialize() 
{

}

void autonomous() 
{

}


void opcontrol() 
{


	

	while (true) 
	{
		leftMotors.move(master.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y));
		rightMotors.move(master.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y));

		if(master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1))
		{

		}
		pros::delay(20);
	}
}