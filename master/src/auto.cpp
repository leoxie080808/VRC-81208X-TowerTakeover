#include "main.h"
#include "util.hpp"
#include "odometry.hpp"
#include "auto.hpp"

extern double errD = 1000000;

double auto_power_angle_function(double input, double powerAngle){
	if(-1*powerAngle<=input&&input<=0)return 1+(input/powerAngle);
	else if(0<input && input<=powerAngle)return 1-(input/powerAngle);
	else if(input <= -1*PI+powerAngle) return (input+PI)/powerAngle-1;
	else if(input >= PI-powerAngle ) return (-1*input+PI)/powerAngle-1;
	else return 0;
}
double targetX, targetY, overallSpeedFactor, powerFactor, steeringFactor, accelTimeMili, errA, tolD, tolA;
void auto_line_task_fn(void *param)//turns then drives to a point in a striaght line
{
	double deltaX, deltaY, deltaAA, deltaAB, power, lastpower, turning, left, right, angleToTarget;
	left = right = 0;
	int autoNow = pros::millis();
	int autoLastTime = autoNow;
	int autoDeltaT = 0;
	int autoLoopCounter = 0;
  auto powerDistanceController = okapi::IterativeControllerFactory::posPID(0.3, 0, 0.01);//TODO: Tune this
  auto turningController = okapi::IterativeControllerFactory::posPID(3, 0, 0.007);//TODO: Tune this
  powerDistanceController.setTarget(0);
  turningController.setTarget(0);
	deltaX = targetX - posX;
	deltaY = targetY - posY;
	errD = sqrt(deltaY * deltaY + deltaX * deltaX);
	// left_intake_mtr.move(127);
	// right_intake_mtr.move(127);
	lastpower = 0;
	while (driveFlag == 2 && errD > tolD)
	{
		autoLoopCounter++;
		autoNow = pros::millis();
		autoDeltaT = autoNow - autoLastTime;
		// pros::lcd::print(4, "delta T auto: %d, counter: %d", autoDeltaT, autoLoopCounter);
		autoLastTime = autoNow;

		deltaX = targetX - posX;
		deltaY = targetY - posY;

		//https://www.desmos.com/calculator/ryfqu5k1nw
		angleToTarget = minimizeAngle(atan2f(targetX - posX, targetY - posY));
    errA = minimizeAngle(angleToTarget - posA);
    turningController.step(abs(errA) > 0.5*PI ? signDouble(errA)*PI-errA : errA);

		errD = sqrt(deltaY * deltaY + deltaX * deltaX);//how far away am I to the target?
    powerDistanceController.step(errD);
		pros::lcd::print(7, "dA: %.2f, dD:.2f", errA, errD);//debug

    power	 = auto_power_angle_function(errA, tolA) * powerDistanceController.getOutput() * -1 * powerFactor; //from -1 to 1, go foward?
		turning = abs(errA) < 0.5*PI ? turningController.getOutput() * -1 : turningController.getOutput() * steeringFactor;
		master.print(2,1,"t: %f", turning);
		pros::lcd::print(5, "p: %.3f, t: %.3f, ", power, turning);
		power = lastpower + signDouble(power) * clampDouble(abs(power) - abs(lastpower), autoDeltaT * (1000/accelTimeMili) * 0.001);
		pros::lcd::print(6, "PA: %.4f, PS: %.4f", auto_power_angle_function(errA, tolA), power);
		lastpower = power;

		// pros::lcd::print(4, "#1: %f, #2: %f", (PI - abs(errA) )/ PI, clampDouble(errD * powerP, 1));

		left = power + turning;
		right = power - turning;
		pros::lcd::print(4, "left: %.2f, right: %.2f", left, right);

		left = clampDouble(left*MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
		right = clampDouble(right*MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);

		left_fwd_mtr.moveVoltage(left*overallSpeedFactor);
		left_rear_mtr.moveVoltage(left*overallSpeedFactor);
		right_fwd_mtr.moveVoltage(right*overallSpeedFactor);
		right_rear_mtr.moveVoltage(right*overallSpeedFactor);

		pros::delay(10);
	}
	driveFlag = 0;
	// left_intake_mtr.move(0);
	// right_intake_mtr.move(0);
  left_fwd_mtr.moveVoltage(0);
  left_rear_mtr.moveVoltage(0);
  right_fwd_mtr.moveVoltage(0);
  right_rear_mtr.moveVoltage(0);
}

void drive_line(double inTargetX, double inTargetY, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance,  double inaTolerance,double inaccelmilli)//a from -pi to pi
{
	targetX = inTargetX;
	targetY = inTargetY;
	overallSpeedFactor = inOverallSpeed;
	powerFactor = inPowerfactor;
	steeringFactor = inTurningfactor;
	tolD = indTolerance;
	tolA = inaTolerance;
	accelTimeMili = inaccelmilli;
	driveFlag = 2;
	if(driveFlag == 0)pros::Task auto_line_task(auto_line_task_fn, (void *)0, "auto line task");
}
