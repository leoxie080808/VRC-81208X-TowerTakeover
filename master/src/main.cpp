#include "main.h"
#include <cmath>

#include "util.hpp"
#include "drive_manual.hpp"
#include "intake.hpp"
#include "tray.hpp"
#include "odometry.hpp"
#include "auto.hpp"

#define OI_DRIVE_HOLD master.get_digital(DIGITAL_B)
#define OI_DRIVE_AUTO master.get_digital(DIGITAL_A)
#define OI_DRIVE_AUTO_STRAIGHT master.get_digital(DIGITAL_Y)

//drive
extern int driveFlag = 0; //0:manual, 1:hold, 2:autonomous

//hardware
extern pros::Controller master(pros::E_CONTROLLER_MASTER);
extern okapi::Motor left_fwd_mtr(1, false, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_fwd_mtr(2, true, okapi::AbstractMotor::gearset::green);
extern okapi::Motor left_rear_mtr(3, false, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_rear_mtr(4, true, okapi::AbstractMotor::gearset::green);

extern okapi::Motor left_intake_mtr(6, true, okapi::AbstractMotor::gearset::green);
extern okapi::Motor right_intake_mtr(7, false, okapi::AbstractMotor::gearset::green);

extern okapi::Motor tray_mtr(8, false, okapi::AbstractMotor::gearset::green);

extern okapi::Motor arm_mtr(9, false, okapi::AbstractMotor::gearset::green);//TODO: check arm motor pin

extern okapi::MotorGroup intake_mtrgrp{left_intake_mtr, right_intake_mtr};

extern okapi::IntegratedEncoder left_fwd_enc(left_fwd_mtr);
extern okapi::IntegratedEncoder right_fwd_enc(right_fwd_mtr);
extern okapi::IntegratedEncoder left_rear_enc(left_rear_mtr);
extern okapi::IntegratedEncoder right_rear_enc(right_rear_mtr);
extern okapi::IntegratedEncoder tray_enc(tray_mtr);
// extern okapi::IntegratedEncoder arm_enc(arm_mtr);//TODO: check if this needs to be reversed

// auto arm_controller = okapi::AsyncControllerFactory::posIntegrated(arm_mtr);

extern okapi::ADIEncoder back_enc('C', 'D', false); //TODO: see if I need to invert this, code assumes right = positive, left = negative

extern double errD;
//AUTONOMOUSE DRIVING STUFFS
/*
x: <- negative, -> positive
y: v negative, A positive
a: clockwise: positive, counterclockwise: negative
*/

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 void initialize()
 {
	 left_fwd_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	 left_rear_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	 right_fwd_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	 right_rear_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	 tray_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

 	resetEverything(0,0,0);
	startTrackPosition();
 	initDriveCurveLookup();
 	pros::lcd::initialize();
 	// lastTime = 0;
 }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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

void autonomous() {
	int autonomousStepsA = 0;
	int autonomousStepsB = 0;
	int loopCounter = 0;
	while(true){
		switch(autonomousStepsA){
			case 0:
				drive_line(0, 30, 0.7, 0.8, 1, 0.5, 0.05, 600);
				switch(autonomousStepsB){
					case 0:
						if(errD < 25){
							intake_mtrgrp.moveVelocity(200);
							autonomousStepsB++;
						}
					break;
					case 1:
						if(errD < 2){
							intake_mtrgrp.moveVelocity(0);
							intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
							autonomousStepsB++;
						}
					break;
				}
				if(driveFlag==0){
					autonomousStepsA++;
					autonomousStepsB=0;
				}
			break;
			case 1:
				drive_line(5, 10, 0.7, 0.8, 0.7, 0.4, 0.05, 1000);
				if(driveFlag==0)autonomousStepsA++;
			break;
			case 2:
				if(tray_enc.get()>TRAY_STOP_ENC)trayOut(1);
				else {
					tray_mtr.move(0);
					autonomousStepsA++;
				}
				break;
			case 3:
				if(tray_enc.get()<0)trayIn(1);
				else {
					tray_mtr.move(0);
					autonomousStepsA++;
				}
			break;
			case 4:
				if(autonomousStepsB==0){
					left_fwd_mtr.moveVelocity(-20);
					right_fwd_mtr.moveVelocity(-20);
					left_rear_mtr.moveVelocity(-20);
					right_rear_mtr.moveVelocity(-20);
					intake_mtrgrp.moveVelocity(-10);
				}
				autonomousStepsB++;
				if(autonomousStepsB>=37){
					left_fwd_mtr.moveVelocity(0);
					right_fwd_mtr.moveVelocity(0);
					left_rear_mtr.moveVelocity(0);
					right_rear_mtr.moveVelocity(0);
					intake_mtrgrp.moveVelocity(0);
					intake_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
					autonomousStepsA++;
				}
			break;
		}
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
	// initialize();
	// okapi::ADIEncoder left_encoder (3, 4, false);
	// okapi::ADIEncoder right_encoder (5, 6, false);

	// auto driveController = okapi::AsyncControllerFactory::posPID(driveMotors, 0.0025, 0.0005, 0);

	//TODO: Tune PID
	// auto driveDistancePID = okapi::IterativeControllerFactory::posPID(0.0025, 0.0005, 0);
	// auto driveAnglePID = okapi::IterativeControllerFactory::posPID(0.0025, 0, 0);

	while (true)
	{
		/*
    To print the current time and deltaT

    std::uint32_t now = pros::millis();
		deltaT = now - lastTime;
		pros::lcd::print(3, "It's %u", now);
		pros::lcd::print(4, "delta t is %i", deltaT);
		lastTime = now;

    */

		// pros::lcd::print(4, "tray enc at %f", tray_enc.get());

		handleManualIntake();
		handleManualTray();



		//drive stuffs
		switch (driveFlag)
		{
		case 0:
			if (OI_DRIVE_HOLD)
			{
				drive_hold();
				break;
			}
			else if (master.get_digital(DIGITAL_A))
			{
				drive_line(0, 30, 0.7, 0.8, 1, 0.5, 0.06, 600);
				break;
			}
			else if (master.get_digital(DIGITAL_Y))
			{
				drive_line(12, 30, 0.4, 0.9, 1, 0.7, 0.08, 1000);
				break;
			}else{
				handleManualDrive();
			}

			break;
		case 1:
			if (!OI_DRIVE_HOLD)
				driveFlag = 0;
			break;
		case 2:
			if (!OI_DRIVE_AUTO && !master.get_digital(DIGITAL_Y))
				driveFlag = 0;
			break;
		}
		pros::delay(20);
	}
}
