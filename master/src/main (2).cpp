#include "main.h"
#include <cmath>
#include <vector>
#include <array>

#include "util.hpp"
#include "drive_manual.hpp"
#include "intake.hpp"
#include "tray.hpp"
#include "odometry.hpp"
#include "auto.hpp"
#include "menu.hpp"
#include "arm.hpp"
#include "selfcheck.hpp"
#include "sdcard.hpp"

#define OI_DRIVE_HOLD master.get_digital(DIGITAL_B)
#define AUTO_END_OF_STEP \
	autonStepsMain++;    \
	autonStepsSub = 0;   \
	driveFlag = 0; //increases mainstep, resets substep, resets driveFlag

//drive
extern int driveFlag = 0; //0:manual, 1:hold, 2:autonomous
extern int trayFlag = 0;
extern int armFlag = 0;
extern bool redSide = true;
extern double driveBaseWidthInch = 12;

//hardware
extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern okapi::Motor left_fwd_mtr(1, false, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_fwd_mtr(2, true, okapi::AbstractMotor::gearset::green);
extern okapi::Motor left_rear_mtr(3, false, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_rear_mtr(4, true, okapi::AbstractMotor::gearset::green);

extern okapi::Motor left_intake_mtr(8, true, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_intake_mtr(6, false, okapi::AbstractMotor::gearset::green);

extern okapi::Motor tray_mtr(5, false, okapi::AbstractMotor::gearset::green);

extern okapi::Motor arm_mtr(10, false, okapi::AbstractMotor::gearset::green);

extern okapi::MotorGroup intake_mtrgrp{left_intake_mtr, right_intake_mtr};
extern okapi::MotorGroup drive_mtrgrp{left_fwd_mtr, right_fwd_mtr, left_rear_mtr, right_rear_mtr};

extern okapi::IntegratedEncoder left_fwd_enc(left_fwd_mtr);
extern okapi::IntegratedEncoder right_fwd_enc(right_fwd_mtr);
extern okapi::IntegratedEncoder left_rear_enc(left_rear_mtr);
extern okapi::IntegratedEncoder right_rear_enc(right_rear_mtr);
extern okapi::IntegratedEncoder tray_enc(tray_mtr);
extern okapi::IntegratedEncoder arm_enc(arm_mtr);

extern okapi::ADIButton tray_btn('A', true);

extern okapi::ADIButton up_btn('F', false);
extern okapi::ADIButton down_btn('G', false);
extern okapi::ADIButton ok_btn('H', false);

extern okapi::AsyncPosIntegratedController tray_controller = okapi::AsyncControllerFactory::posIntegrated(tray_mtr);
extern okapi::AsyncPosIntegratedController arm_controller = okapi::AsyncControllerFactory::posIntegrated(arm_mtr);

extern okapi::ADIEncoder back_enc('C', 'D', false);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

std::vector<MyPoint> autonPathA;
extern double thirdStackX = 0;
extern double forthStackX = 26.7;
extern double firstStackX = 0;
extern double firstBlockX = -25;
extern bool debugging = false;
extern bool menuFlag = false;
// extern double startingY = 6 + 2.5 + 9.47 / 2;
extern double startingY = 7;
extern int autonSelection = 1;//0: 7 cube(redacted); 1: 5 cube; 2: upper; 3: programming skill

void initialize()
{
	pros::lcd::initialize();
	pros::delay(2);
	readConfigs();

	pros::lcd::print(3, "\t    %s", redSide ? "Red" : "Blue");
	pros::lcd::print(4, "\t    auton:%d", autonSelection);
	// master.print(1, 1, "A: %d, S: %s", autonSelection, redSide?"red_":"blue");

	switch (autonSelection)
	{
	case 0:
		resetEverything(thirdStackX, startingY, 0);
		break;
	case 1:
		resetEverything(forthStackX, startingY, 0);
		break;
	case 2:
		resetEverything(firstStackX, startingY, 0);
		break;
	case 3:
		resetEverything(thirdStackX, startingY, 0);
		break;
	}
	// startSelfcheck();
	startTrackPosition();
	initDriveCurveLookup();
	tray_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	startMenuTask();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	tray_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	startMenuTask();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

int autonStepsMain = 0;
int autonStepsSub = 0;
int autonStartTime = 0, autonDeltaTime = 0;

void autonomous()
{
	std::uint32_t now = pros::millis();
	autonStartTime = now;
	autonDeltaTime = now - autonStartTime;
	//pros::lcd::print(3, "It's %u", now);
	//pros::lcd::print(4, "delta t is %i", deltaT);

	tray_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);


	if (!debugging)
		endMenuTask();

	autonPathA.clear();
	autonPathA.push_back({thirdStackX, 60});
	autonPathA.push_back({thirdStackX, 45});
	autonPathA.push_back({thirdStackX + 5, 32});
	autonPathA.push_back({(thirdStackX + forthStackX) / 2, 22});
	autonPathA.push_back({forthStackX - 5, 18});
	autonPathA.push_back({forthStackX, 17});

	int autonStepsMain = 0;
	int autonStepsSub = 0;
	while (true)
	{
		if (!debugging)
			endMenuTask();
		std::uint32_t now = pros::millis();
		autonDeltaTime = now - autonStartTime;
		pros::lcd::print(7, "A: %d, B:%d", autonStepsMain, autonStepsSub); //debug
		switch (autonSelection)
		{
		case 0: //start of 7 cube auton without preload
			/*if (autonDeltaTime > 10000 && autonStepsMain < 5)
			{
				AUTO_END_OF_STEP
				autonStepsMain = 5;
			} */
			//if we run out of time, prioritize putting down the cubes in the scoring zone
			switch (autonStepsMain)
			{
			case 0: //grab the cubes infront of us
				switch (autonStepsSub)
				{
				case 0: //drives foward a bit to push the preload foward
					drive_line({thirdStackX, 15}, 1, 1, 0.5, 0.5, d2r(15), 500, false);
					autonStepsSub = 3;
					break;
				// case 1:
				// 	if (errD < 1 || driveFlag == 0)
				// 	{
				// 		driveFlag = 0;
				// 		autonStepsSub++;
				// 	}
				// 	break;
				// case 2: //drives back a bit
				// 	drive_line({thirdStackX, 18}, 0.8, 1, 0.5, 0.5, d2r(15), 500, true);
				// 	autonStepsSub++;
				// 	break;
				case 3: //extends itself
					if (errD < 4 || driveFlag == 0)
					{
						driveFlag = 0;
						drive_mtrgrp.moveVoltage(0);
						tray_controller.setMaxVelocity(200);
						moveTrayPosition(1100);
						arm_controller.setMaxVelocity(175);
						arm_controller.setTarget(550);
						arm_controller.waitUntilSettled();
						arm_controller.setTarget(50);
						moveTrayPosition(2300);
						autonStepsSub++;
					}
					break;
				case 4:
					if (tray_enc.get() > 2200 || (tray_mtr.getActualVelocity() < 10 && tray_enc.get() > 1700))
					{
						moveTrayPosition(0);
						autonStepsSub++;
					}
					break;
				case 5:
					if (tray_enc.get() < 850)
						autonStepsSub++;
					break;
				case 6: //move a bit slower through the cubes to pick them up
					arm_controller.flipDisable(true);
					drive_line({thirdStackX, 53}, 0.7, 1, 1, 0.5, d2r(3), 500, true);
					intake_mtrgrp.moveVelocity(-200);
					autonStepsSub++;
					break;
				case 7:
					if (errD < 2 || driveFlag == 0)
					{
						intake_mtrgrp.moveVelocity(0);
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 1:	//Backoff to the second stack
				switch (autonStepsSub)
				{
				case 0:
					follow_path(autonPathA, 1, 1, 1, 1, d2r(10), 800, 5, 2, 5, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 2:	//Move foward pretty fast for a bit after turning
				switch (autonStepsSub)
				{
				case 0:
					turn_angle(d2r(0), 0.5, d2r(15), 300, true);
					autonStepsSub++;
					break;
				case 1:
					if (fabs(errA) <= d2r(20) || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2:
					drive_line({forthStackX, 25}, 1, 1, 1, 0.4, d2r(5), 400, false);
					intake_mtrgrp.moveVelocity(-150);
					autonStepsSub++;
					break;
				case 3:
					if (errD < 5 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 3:	//Move foward slowly to take in cubes
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 53}, 0.5, 1, 1, 0.4, d2r(5), 700, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 4:	//Move backward a bit before turning and also lift the tray up a bit
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 38}, 0.8, 1, 1, 0.4, d2r(10), 750, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 5: //Turn to face the intake
				switch (autonStepsSub)
				{
				case 0:
					moveTrayPosition(300);
					turn_angle(d2r(120), 0.8, d2r(10), 1000, true);
					autonStepsSub++;
					break;
				case 1:
					if (fabs(errA) <= d2r(25) || driveFlag == 0)
					{
						intake_mtrgrp.moveVelocity(0);
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 6: //Drive up to the scoring zone
				switch (autonStepsSub)
				{
				case 0:
					intake_mtrgrp.moveVelocity(0);
					drive_line({36, 23}, 0.9, 0.9, 0.9, 0.5, d2r(6), 1500, false);
					moveTrayPosition(TRAY_SLOWDOWN_ENC / 2 - 500);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1.5 || driveFlag == 0)
					{
						//make sure the chassis stops moving
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 7: //Pushes the tray out after hard resetting
				if (autonStepsSub == 0)
				{
					drive_mtrgrp.moveVelocity(50);
					autonStepsSub++;
				}
				else if (autonStepsSub < 130)
				{
					autonStepsSub++;
				}

				if (autonStepsSub == 55)
				{
					drive_mtrgrp.moveVelocity(-50);
				}

				if (autonStepsSub == 90)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					intake_mtrgrp.moveVelocity(15);
					tray_controller.setMaxVelocity(75);
					moveTrayPosition(TRAY_STOP_ENC + 100);
				}
				if (autonStepsSub >= 100)
				{
					if (tray_enc.get() > TRAY_STOP_ENC - 50)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 8: //Moves back the robot after moving foward a bit
				autonStepsSub++;
				if (autonStepsSub == 30)
				{
					intake_mtrgrp.moveVelocity(0);
					drive_mtrgrp.moveVelocity(25);
				}
				if (autonStepsSub == 80)
				{
					tray_controller.setMaxVelocity(100);
					moveTrayPosition(TRAY_SLOWDOWN_ENC - 200);
					drive_mtrgrp.moveVelocity(-50);
					intake_mtrgrp.moveVelocity(30);
				}
				if (autonStepsSub >= 150)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					autonStepsMain++;
				}
				break;
			}
			break; //end of 7 cube auton without preload
		case 1:	//start of 5 cube auton with preload
			// if(autonDeltaTime>10000)
			switch (autonStepsMain)
			{
			case 0: //extending after pushing the preload foward
				switch (autonStepsSub)
				{
				case 0: //drives foward a bit to push the preload foward
					arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					drive_line({forthStackX, 21}, 1, 1, 0.5, 0.5, d2r(15), 400, true);
					tray_controller.setMaxVelocity(200);
					moveTrayPosition(1250);
					arm_controller.flipDisable(true);
					arm_controller.setMaxVelocity(140);
					pros::delay(300);
					arm_controller.setTarget(560);
					arm_controller.flipDisable(false);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //drives back a bit
					drive_line({forthStackX, 16}, 0.5, 1, 0.5, 0.5, d2r(15), 300, true);
					autonStepsSub++;
					break;
				case 3: //extends itself
					// if (errD < 1 || driveFlag == 0)
					// {
					// driveFlag = 0;
					// drive_mtrgrp.moveVoltage(0);
					pros::delay(300);
					autonStepsSub++;
				// }
					break;
				case 4:
					if (arm_enc.get() > 555)
					{
						arm_controller.flipDisable(true);
						arm_controller.setMaxVelocity(200);
						arm_controller.setTarget(50);
						arm_controller.flipDisable(false);
						tray_mtr.moveVelocity(200);
						// moveTrayPosition(2200);
						autonStepsSub++;
					}
					break;
				case 5:
					if (tray_enc.get() > 2050 || (tray_mtr.getActualVelocity() < 10 && tray_enc.get() > 1800))
					{
						tray_mtr.moveVelocity(-200);
						// moveTrayPosition(0);
						autonStepsSub++;
					}
					break;
				case 6:
					if (tray_enc.get() < 1400)
					{
						moveTrayPosition(0);
						arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
						arm_controller.flipDisable(true);
						autonStepsSub++;
						// pros::delay(50);
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 1:	//drive foward slowly to give arm enough time to deploy
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 24}, 0.45, 1, 1, 0.4, d2r(3), 200, false);
					intake_mtrgrp.moveVelocity(-200);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2.5 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 2:	//drive foward slowly to take in cube
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 54}, 0.525, 1, 1, 0.4, d2r(5), 300, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 3:	//move backward a bit
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 38}, 1, 1, 1, 0.4, d2r(10), 500, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 4: //turn to face the scoring zone
				switch (autonStepsSub)
				{
				case 0:
					moveTrayPosition(300);
					turn_angle(d2r(120), 0.9, d2r(10), 750, true);
					autonStepsSub++;
					break;
				case 1:
					if (fabs(errA) <= d2r(25) || driveFlag == 0)
					{
						intake_mtrgrp.moveVelocity(0);
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 5: //drive near the scoring zone
				switch (autonStepsSub)
				{
				case 0:
					intake_mtrgrp.moveVelocity(0);
					drive_line({36, 22}, 1, 1, 1, 0.5, d2r(6), 1500, false);
					moveTrayPosition(TRAY_SLOWDOWN_ENC / 2 - 500);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1.5 || driveFlag == 0)
					{
						//make sure the chassis stops moving
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 6: //pushes the tray out after hard resetting
				if (autonStepsSub == 0)
				{
					drive_mtrgrp.moveVelocity(70);
					autonStepsSub++;
				}
				else if (autonStepsSub < 70)
				{
					autonStepsSub++;
				}

				if (autonStepsSub == 35)
				{
					drive_mtrgrp.moveVelocity(-70);
				}

				if (autonStepsSub == 60)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					intake_mtrgrp.moveVelocity(20);
					tray_controller.setMaxVelocity(150);
					moveTrayPosition(TRAY_STOP_ENC + 70);
				}
				if (autonStepsSub >= 65)
				{
					if (tray_enc.get() > TRAY_STOP_ENC - 100)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 7: //pushes foward the tray then back off
				autonStepsSub++;
				if (autonStepsSub == 30)
				{
					intake_mtrgrp.moveVelocity(0);
					drive_mtrgrp.moveVelocity(35);
				}
				if (autonStepsSub == 60)
				{
					tray_controller.setMaxVelocity(100);
					moveTrayPosition(TRAY_SLOWDOWN_ENC - 200);
					drive_mtrgrp.moveVelocity(-75);
					intake_mtrgrp.moveVelocity(40);
				}
				if (autonStepsSub >= 90)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					autonStepsMain++;
				}
				break;
			}
			break; //end of 5 cube auton with preload
		case 2:	//start of upper auton
			switch (autonStepsMain)
			{
			case 0: //extending after pushing the preload foward
				switch (autonStepsSub)
				{
				case 0: //drives foward a bit to push the preload foward
					arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					drive_line({firstStackX, 20}, 0.7, 1, 0.5, 0.5, d2r(15), 800, true);
					tray_controller.setMaxVelocity(200);
					moveTrayPosition(1300);
					arm_controller.flipDisable(true);
					arm_controller.setMaxVelocity(130);
					arm_controller.setTarget(560);
					pros::delay(300);
					arm_controller.flipDisable(false);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //drives back a bit
					drive_line({firstStackX, 16}, 0.4, 1, 0.5, 0.5, d2r(15), 300, true);
					autonStepsSub++;
					break;
				case 3: //extends itself
					// if (errD < 1 || driveFlag == 0)
					// {
					// driveFlag = 0;
					// drive_mtrgrp.moveVoltage(0);
					// pros::delay(200);
					autonStepsSub++;
				// }
					break;
				case 4:
					if (arm_enc.get() > 500)
					{
						arm_controller.flipDisable(true);
						arm_controller.setMaxVelocity(200);
						arm_controller.setTarget(50);
						arm_controller.flipDisable(false);
						moveTrayPosition(2300);
						autonStepsSub++;
					}else{
						arm_controller.flipDisable(true);
						arm_mtr.move(200);
					}
					break;
				case 5:
					if (tray_enc.get() > 2050 || (tray_mtr.getActualVelocity() < 10 && tray_enc.get() > 1800))
					{
						moveTrayPosition(0);
						autonStepsSub++;
					}
					break;
				case 6:
					if (tray_enc.get() < 1450)
					{
						arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
						arm_controller.flipDisable(true);
						autonStepsSub++;
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 1:	//drive foward slowly to give arm enough time to deploy
				switch (autonStepsSub)
				{
				case 0:
					pros::delay(200);
					drive_line({firstStackX, 27}, 0.6, 1, 1, 0.4, d2r(3), 200, false);
					intake_mtrgrp.moveVelocity(-200);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2.5 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 2:	//drive foward slowly to take in cube, back off to make the stack fall then go foward again
				switch (autonStepsSub)
				{
				case 0:
					intake_mtrgrp.moveVelocity(-200);
					drive_line({firstStackX, 36}, 1, 1, 1, 0.4, d2r(5), 300, false);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2://go foward
					intake_mtrgrp.moveVelocity(-150);
					drive_line({firstStackX, 44}, 0.7, 1, 1, 0.4, d2r(5), 100, false);
					autonStepsSub++;
					break;
				case 3:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 4://go back to make the tower fall
					drive_line({firstStackX, 31}, 1, 1, 1, 0.4, d2r(5), 50, false);
					autonStepsSub++;
					break;
				case 5:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 6://go foward again to take in cubes
					intake_mtrgrp.moveVelocity(-200);
					drive_line({firstStackX, 53}, 0.9, 1, 1, 0.4, d2r(5), 100, true);
					autonStepsSub++;
					break;
				case 7:
					if (errD < 2 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 3:	//drive backwards a bit
				switch (autonStepsSub)
				{
				case 0:
					drive_line({firstStackX, 45}, 1, 1, 1, 0.4, d2r(10), 200, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 4: //turn to face the other cube
				switch (autonStepsSub)
				{
				case 0:
					// master.print(1, 1, "A:%d, B:%d", autonStepsMain,autonStepsSub);
					turn_angle(d2r(-120), 1, d2r(20), 600, true);
					autonStepsSub++;
					break;
				case 1:
					// master.print(1, 1, "A:%d, B:%d", autonStepsMain,autonStepsSub);
					if (fabs(errA) <= d2r(25) || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 5: //drive foward to pick up the cube
				switch (autonStepsSub)
				{
				case 0:
					drive_line({-15, 33.4}, 1, 1, 1, 0.4, d2r(5), 300, false);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2:
					drive_line({firstBlockX, 26}, 0.8, 1, 1, 0.4, d2r(5), 500, false);
					autonStepsSub++;
					break;
				case 3:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 6:	//continue driving towards the scoring zone
				switch (autonStepsSub)
				{
				case 0:
					moveTrayPosition(300);
					intake_mtrgrp.moveVelocity(0);
					drive_line({-27, 24.4}, 0.6, 0.9, 0.9, 0.5, d2r(6), 600, false);
					moveTrayPosition(TRAY_SLOWDOWN_ENC / 2 - 500);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 7: //pushes the tray out after hard resetting ish
				if (autonStepsSub == 0)
				{
					drive_mtrgrp.moveVelocity(35);
					autonStepsSub++;
				}
				else if (autonStepsSub < 70)
				{
					autonStepsSub++;
				}

				if (autonStepsSub == 40)
				{
					drive_mtrgrp.moveVelocity(-32);
				}

				if (autonStepsSub == 65)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					intake_mtrgrp.moveVelocity(30);
					tray_controller.setMaxVelocity(150);
					moveTrayPosition(TRAY_STOP_ENC + 50);
				}
				if (autonStepsSub >= 67)
				{
					if (tray_enc.get() > TRAY_STOP_ENC - 100)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 8: //pushes foward and then back off
				autonStepsSub++;
				if (autonStepsSub == 5)
				{
					intake_mtrgrp.moveVelocity(0);
					drive_mtrgrp.moveVelocity(30);
				}
				if (autonStepsSub == 35)
				{
					tray_controller.setMaxVelocity(150);
					moveTrayPosition(TRAY_SLOWDOWN_ENC - 200);
					drive_mtrgrp.moveVelocity(-150);
					intake_mtrgrp.moveVelocity(50);
				}
				if (autonStepsSub >= 70)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					autonStepsMain++;
				}
				break;
			}
			break; //end of upper auton
		case 3:	//start of programming skills
			switch (autonStepsMain)
			{
				case 0: //extending after pushing the preload foward
					switch (autonStepsSub)
					{
					case 0: //drives foward a bit to push the preload foward
						arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
						drive_line({thirdStackX, 21}, 1, 1, 0.5, 0.5, d2r(15), 400, true);
						tray_controller.setMaxVelocity(200);
						moveTrayPosition(1250);
						arm_controller.flipDisable(true);
						arm_controller.setMaxVelocity(140);
						pros::delay(300);
						arm_controller.setTarget(560);
						arm_controller.flipDisable(false);
						autonStepsSub++;
						break;
					case 1:
						if (errD < 2 || driveFlag == 0)
						{
							driveFlag = 0;
							autonStepsSub++;
						}
						break;
					case 2: //drives back a bit
						drive_line({thirdStackX, 14}, 0.4, 1, 0.5, 0.5, d2r(15), 300, true);
						autonStepsSub++;
						break;
					case 3: //extends itself
						// if (errD < 1 || driveFlag == 0)
						// {
						// driveFlag = 0;
						// drive_mtrgrp.moveVoltage(0);
						pros::delay(300);
						autonStepsSub++;
					// }
						break;
					case 4:
						if (arm_enc.get() > 555)
						{
							arm_controller.flipDisable(true);
							arm_controller.setMaxVelocity(200);
							arm_controller.setTarget(50);
							arm_controller.flipDisable(false);
							moveTrayPosition(2200);
							autonStepsSub++;
						}
						break;
					case 5:
						if (tray_enc.get() > 1975 || (tray_mtr.getActualVelocity() < 10 && tray_enc.get() > 1800))
						{
							moveTrayPosition(0);
							autonStepsSub++;
						}
						break;
					case 6:
						if (tray_enc.get() < 1300)
						{
							arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
							arm_controller.flipDisable(true);
							autonStepsSub++;
							// pros::delay(50);
							AUTO_END_OF_STEP
						}
						break;
					}
					break; //A
			case 1:	//grab the purple cube for mid tower
				autonStepsMain++;
				/*switch (autonStepsSub)
				{
				case 0: //drives foward
					drive_line({thirdStackX, 23.2}, 0.5, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //turns toward the mid tower
					turn_angle(d2r(-90), 0.7, d2r(5), 750, true);
					autonStepsSub++;
					break;
				case 3:
					if (fabs(errA) <= d2r(10) || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 4: //moves foward to pickup the purple cube
					intake_mtrgrp.moveVelocity(-200);
					drive_line({thirdStackX - 10, 23.2}, 0.5, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 5:
					if (errD < 1 || driveFlag == 0)
					{
						pros::delay(400);
						intake_mtrgrp.moveVelocity(0);
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 6: //moved back and reverse intake a bit at the same time
					intake_mtrgrp.moveRelative(180, 100);
					drive_line({thirdStackX - 1, 23.2}, 0.5, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 7:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 8: //move foward slowly while raising arm
					moveArmMidTower(1150);
					drive_line({thirdStackX - 5, 23.2}, 0.3, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 9:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 10: //moves intake to place cube in tower
					intake_mtrgrp.moveVelocity(100);
					pros::delay(500);
					AUTO_END_OF_STEP
					break;
				}*/
				break; //End of A 1
			case 2:	//Get back to the third stack and pick the third stack up
				switch (autonStepsSub)
				{
				case 0: //back off onto the third stack
					// moveArmDown();
					intake_mtrgrp.moveVelocity(-200);
					drive_line({thirdStackX, 23.2}, 0.4, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //drives slowly to pickup cubes
					drive_line({thirdStackX, 55}, 0.6, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 3:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //End of A 2
			case 3:	//Backoff to the second stack
				switch (autonStepsSub)
				{
				case 0:
					follow_path(autonPathA, 0.9, 1, 1, 1, d2r(5), 800, 5, 2, 5, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //End of A 3
			case 4:	//Sets up robot to pickup the forth stack
				switch (autonStepsSub)
				{
				case 0:
					turn_angle(d2r(0), 1, d2r(5), 300, true);
					autonStepsSub++;
					break;
				case 1:
					if (fabs(errA) <= d2r(10) || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2:
					drive_line({forthStackX, 26}, 1, 1, 1, 0.4, d2r(5), 400, false);
					intake_mtrgrp.moveVelocity(-200);
					autonStepsSub++;
					break;
				case 3:
					if (errD < 5 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 5:	//Move foward slowly to pickup the forth stack
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 50}, 0.4, 1, 1, 0.4, d2r(5), 700, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
					break;
				}
				break; //A
			case 6:	//move backward a bit
				switch (autonStepsSub)
				{
				case 0:
					drive_line({forthStackX, 38}, 0.8, 1, 1, 0.4, d2r(10), 750, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 7: //Turn to face the intake
				switch (autonStepsSub)
				{
				case 0:
					moveTrayPosition(300);
					turn_angle(d2r(120), 0.7, d2r(10), 1000, true);
					autonStepsSub++;
					break;
				case 1:
					if (fabs(errA) <= d2r(25) || driveFlag == 0)
					{
						intake_mtrgrp.moveVelocity(0);
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 8: //Drive up to the scoring zone
				switch (autonStepsSub)
				{
				case 0:
					intake_mtrgrp.moveVelocity(0);
					drive_line({36, 23}, 0.75, 0.9, 0.9, 0.5, d2r(6), 1500, false);
					moveTrayPosition(TRAY_SLOWDOWN_ENC / 2 - 500);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1.5 || driveFlag == 0)
					{
						//make sure the chassis stops moving
						AUTO_END_OF_STEP
					}
					break;
				}
				break;
			case 9: //Pushes the tray out after hard resetting
				if (autonStepsSub == 0)
				{
					drive_mtrgrp.moveVelocity(70);
					autonStepsSub++;
				}
				else if (autonStepsSub < 70)
				{
					autonStepsSub++;
				}

				if (autonStepsSub == 35)
				{
					drive_mtrgrp.moveVelocity(-75);
				}

				if (autonStepsSub == 60)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					intake_mtrgrp.moveVelocity(30);
					tray_controller.setMaxVelocity(150);
					moveTrayPosition(TRAY_STOP_ENC + 70);
				}
				if (autonStepsSub >= 65)
				{
					if (tray_enc.get() > TRAY_STOP_ENC - 100)
					{
						AUTO_END_OF_STEP
					}
				}
				break;
			case 10: //Moves back the robot after moving foward a bit
				autonStepsSub++;
				if (autonStepsSub == 10)
				{
					intake_mtrgrp.moveVelocity(0);
					drive_mtrgrp.moveVelocity(20);
				}
				if (autonStepsSub == 50)
				{
					tray_controller.setMaxVelocity(100);
					moveTrayPosition(TRAY_SLOWDOWN_ENC - 200);
					drive_mtrgrp.moveVelocity(-75);
					intake_mtrgrp.moveVelocity(40);
				}
				if (autonStepsSub >= 85)
				{
					drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					drive_mtrgrp.moveVelocity(0);
					intake_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					AUTO_END_OF_STEP
				}
				break;
			case 11: //orange cube for small tower
				switch (autonStepsSub)
				{
				case 0: //drives back
					drive_line({forthStackX-8.6, 40}, 0.8, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //turns toward the low tower
					turn_angle(d2r(0), 0.9, d2r(5), 750, true);
					autonStepsSub++;
					break;
				case 3:
					if (fabs(errA) <= d2r(5) || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 4: //moves foward to pickup the purple cube
					intake_mtrgrp.moveVelocity(-200);
					drive_line({forthStackX-8.6, 53}, 0.8, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 5:
					if (errD < 1 || driveFlag == 0)
					{
						pros::delay(400);
						intake_mtrgrp.moveVelocity(0);
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 6: //moved back and reverse intake a bit at the same time
					intake_mtrgrp.moveRelative(275, 100);
					drive_line({forthStackX-8.6, 45}, 0.5, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 7:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 8: //move foward slowly while raising arm
					moveArmLowTower(900);
					drive_line({forthStackX-8.6, 50}, 0.5, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 9:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 10: //moves intake to place cube in tower
					intake_mtrgrp.moveVelocity(100);
					pros::delay(500);
					AUTO_END_OF_STEP
					break;
				}
				break; //End of A 11
			case 12://backing off from tower and going to the other side
				switch(autonStepsSub){
				case 0: //move foward slowly while raising arm
					moveArmDown();
					drive_line({forthStackX-8.6, 45}, 1, 1, 1, 0.4, d2r(5), 400, true);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 1 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				}
			break;
			}
			break; //end of programming skills
		}		   //end of auton paths switch block
		pros::delay(20);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	// if (!debugging)
	// 	endMenuTask();
	intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	tray_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// if (!debugging)
	// 	endTrackPosition();
	autonStepsSub = 0;
	while (true)
	{
		// master.print(1,1,"test", 0);
		/*if(master.get_digital(DIGITAL_A)){
				switch (autonStepsSub)
				{
				case 0: //drives foward a bit to push the preload foward
					arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
					drive_line({forthStackX, 21}, 1, 1, 0.5, 0.5, d2r(15), 400, true);
					tray_controller.setMaxVelocity(200);
					moveTrayPosition(1250);
					arm_controller.flipDisable(true);
					arm_controller.setMaxVelocity(140);
					pros::delay(300);
					arm_controller.setTarget(560);
					arm_controller.flipDisable(false);
					autonStepsSub++;
					break;
				case 1:
					if (errD < 2 || driveFlag == 0)
					{
						driveFlag = 0;
						autonStepsSub++;
					}
					break;
				case 2: //drives back a bit
					drive_line({forthStackX, 16}, 0.5, 1, 0.5, 0.5, d2r(15), 300, true);
					autonStepsSub++;
					break;
				case 3: //extends itself
					// if (errD < 1 || driveFlag == 0)
					// {
					// driveFlag = 0;
					// drive_mtrgrp.moveVoltage(0);
					pros::delay(300);
					autonStepsSub++;
				// }
					break;
				case 4:
					if (arm_enc.get() > 555)
					{
						arm_controller.flipDisable(true);
						arm_controller.setMaxVelocity(200);
						arm_controller.setTarget(50);
						arm_controller.flipDisable(false);
						tray_mtr.moveVelocity(200);
						// moveTrayPosition(2200);
						autonStepsSub++;
					}
					break;
				case 5:
					if (tray_enc.get() > 2050 || (tray_mtr.getActualVelocity() < 10 && tray_enc.get() > 1800))
					{
						tray_mtr.moveVelocity(-200);
						// moveTrayPosition(0);
						autonStepsSub++;
					}
					break;
				case 6:
					if (tray_enc.get() < 1400)
					{
						moveTrayPosition(0);
						arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
						arm_controller.flipDisable(true);
						// pros::delay(50);
						AUTO_END_OF_STEP
						autonStepsSub = -100;
					}
					break;
				}
				continue;
		}*/

		/*
		To print the current time and deltaT

		std::uint32_t now = pros::millis();
			deltaT = now - lastTime;
			//pros::lcd::print(3, "It's %u", now);
			//pros::lcd::print(4, "delta t is %i", deltaT);
			lastTime = now;
		*/

		handleManualIntake();
		handleManualTray();
		handleManualArm();

		//driving stuffs
		if (debugging) //if debugging, enable odometry test buttons
		{
			switch (driveFlag)
			{
			case 0:
				if (OI_DRIVE_HOLD)
				{
					drive_hold();
				}
				//hold A,B,X,Y in sequence to test odometry accuracy
				else if (master.get_digital_new_press(DIGITAL_A))
				{
					drive_line({0, 48}, 1, 0.8, 1, 0.5, d2r(4), 600, true);
					break;
				}
				else if (master.get_digital_new_press(DIGITAL_X))
				{
					turn_angle(d2r(-120), 0.9, d2r(2), 1000, true);
					break;
				}
				//Pure pursuit controller test.
				else if (master.get_digital_new_press(DIGITAL_LEFT))
				{
					std::vector<MyPoint> squarePath;
					squarePath.push_back({0, 10});
					squarePath.push_back({0, 40});
					squarePath.push_back({-30, 40});
					squarePath.push_back({-30, 10});
					squarePath.push_back({-5, 10});
					follow_path(squarePath, 1, 1, 1, 0.5, d2r(15), 500, 5, 1, 4, true);
					break;
				}
				else
				{
					handleManualDrive();
				}
				break;
			case 1:
				if (!OI_DRIVE_HOLD)
					driveFlag = 0;
				break;
			case 2:
				if (!master.get_digital(DIGITAL_A) && !master.get_digital(DIGITAL_X) && !master.get_digital(DIGITAL_LEFT))
				driveFlag = 0;
				break;
			}
		}
		else //if in competition, just do normal driving
		{
			switch (driveFlag)
			{
			case 0:
				if (OI_DRIVE_HOLD)
				{
					drive_hold();
				}
				else
				{
					handleManualDrive();
				}
				break;
			case 1:
				if (!OI_DRIVE_HOLD)
					driveFlag = 0;
				break;
			case 2:
				driveFlag = 0;
				break;
			}
		}

		pros::delay(20);
	}
}
