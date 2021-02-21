#include "drive_manual.hpp"
#include "main.h"
#include <cmath>
int manualDrivePower, manualDriveTurn, manualDriveLeft, manualDriveRight;
int driveCurveLookUp[256];

void initDriveCurveLookup()
{
	int input;
	double a, b;
	for (int x = 0; x <= 254; x++)
	{
		input = x - 127;
		a = exp(MANUAL_DRIVE_CURVATURE * -0.1);
		b = (a + exp(0.1 * (fabs(input) - 127)) * (1 - a));
		driveCurveLookUp[x] = (int)round(b * input);
	}
}

int lookupDriveCurve(int input)
{
	return driveCurveLookUp[input + 127];
}

void handleManualDrive()
{
  manualDrivePower = lookupDriveCurve(OI_DRIVE_POWER);
  manualDriveTurn = lookupDriveCurve(OI_DRIVE_STEER);
  manualDriveLeft = manualDrivePower + manualDriveTurn;
  manualDriveRight = manualDrivePower - manualDriveTurn;
  left_fwd_mtr.move(manualDriveLeft);
  right_fwd_mtr.move(manualDriveRight);
  left_rear_mtr.move(manualDriveLeft);
  right_rear_mtr.move(manualDriveRight);
}


void hold_task_fn(void *param)
{
	left_fwd_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	left_rear_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	right_fwd_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	right_rear_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	auto left_fwd_hold_controller = okapi::AsyncControllerFactory::posIntegrated(left_fwd_mtr);
	auto left_rear_hold_controller = okapi::AsyncControllerFactory::posIntegrated(left_rear_mtr);
	auto right_fwd_hold_controller = okapi::AsyncControllerFactory::posIntegrated(right_fwd_mtr);
	auto right_rear_hold_controller = okapi::AsyncControllerFactory::posIntegrated(right_rear_mtr);
	double left_fwd_hold_target = left_fwd_enc.get();
	double left_rear_hold_target = left_rear_enc.get();
	double right_fwd_hold_target = right_fwd_enc.get();
	double right_rear_hold_target = right_rear_enc.get();

	while (driveFlag == 1)
	{
		if (fabs(left_fwd_enc.get() - left_fwd_hold_target) > HOLD_TOLERANCE)
		{
			left_fwd_hold_controller.setTarget(left_fwd_hold_target);
		}
		if (fabs(left_rear_enc.get() - left_rear_hold_target) > HOLD_TOLERANCE)
		{
			left_rear_hold_controller.setTarget(left_rear_hold_target);
		}
		if (fabs(right_fwd_enc.get() - right_fwd_hold_target) > HOLD_TOLERANCE)
		{
			right_fwd_hold_controller.setTarget(right_fwd_hold_target);
		}
		if(fabs(right_rear_enc.get() - right_rear_hold_target) > HOLD_TOLERANCE){
			right_rear_hold_controller.setTarget(right_rear_hold_target);
		}
		pros::delay(20);
	}
	drive_mtrgrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void drive_hold()
{
	if(driveFlag!=1)pros::Task hold_task(hold_task_fn, (void *)0, "hold task");
	driveFlag = 1;
}
