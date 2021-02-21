#include "main.h"
#include "util.hpp"
#include "odometry.hpp"
#include "auto.hpp"
#include <vector>

extern double errD = 1000000;
extern double errA = 1000000;
double lastErrD, lastErrA;

//for controlling the throttle based on wether the robot is facing the target or not
double auto_power_angle_function(double input, double angleTolToPower)
{
	if (-1 * angleTolToPower <= input && input <= 0)
		return 1 + (input / angleTolToPower);
	else if (0 < input && input <= angleTolToPower)
		return 1 - (input / angleTolToPower);
	else if (input <= -1 * PI + angleTolToPower)
		return (input + PI) / angleTolToPower - 1;
	else if (input >= PI - angleTolToPower)
		return (-1 * input + PI) / angleTolToPower - 1;
	else
		return 0;
}

bool getDriveStalling()
{
	if (left_fwd_mtr.getTorque() > 0.15 && left_fwd_mtr.getActualVelocity() < 10)
		return true;
	if (left_rear_mtr.getTorque() > 0.15 && left_rear_mtr.getActualVelocity() < 10)
		return true;
	if (right_fwd_mtr.getTorque() > 0.15 && right_fwd_mtr.getActualVelocity() < 10)
		return true;
	if (right_rear_mtr.getTorque() > 0.15 && right_rear_mtr.getActualVelocity() < 10)
		return true;
	return false;
}

double power, lastPower, turning, lastTurn;
bool getDriveSteadyState()
{
	//if not moving much and power isn't changing
	//errD 5 inch per second, angle 50 degrees per second, power 20% per second
	if (fabs(errD - lastErrD) < 5.0 / 100.0 && fabs(errA - lastErrA) < d2r(50) / 100.0 && fabs(power - lastPower) < 0.1 / 100.0)
		return true;
	else
		return false;
}

double targetX, targetY, overallSpeedFactor, powerFactor, steeringFactor, accelTimeMili, tolD, tolA;
bool stopAtEnd;
void auto_line_task_fn(void *param) //turns then drives to a point in a striaght line
{
	bool firstTime = true;
	double deltaX, deltaY, deltaAA, deltaAB, left, right, angleToTarget;
	left = right = 0;
	int autoNow = pros::millis();
	int autoLastTime = autoNow;
	int autoDeltaT = 0;
	int autoLoopCounter = 0;
	int stalling = 0;
	int steadyState = 0;
	auto powerDistanceController = okapi::IterativeControllerFactory::posPID(0.3, 0.003, 0.01);
	auto turningController = okapi::IterativeControllerFactory::posPID(4, 0.01, 0.02);
	powerDistanceController.setTarget(0);
	turningController.setTarget(0);
	deltaX = targetX - posX;
	deltaY = targetY - posY;
	lastErrD = errD = sqrt(deltaY * deltaY + deltaX * deltaX);
	angleToTarget = wrapAngle(atan2f(targetX - posX, targetY - posY));
	lastErrA = errA = wrapAngle(angleToTarget - posA);
	lastTurn = 0;
	lastPower = drive_mtrgrp.getActualVelocity()/200.0;
	while (driveFlag == 2 && errD > tolD)
	{
		if (turningController.isSettled())
			turningController.setTarget(0);
		if (powerDistanceController.isSettled())
			powerDistanceController.setTarget(0);
		autoLoopCounter++;
		autoNow = pros::millis();
		autoDeltaT = autoNow - autoLastTime;
		// pros::lcd::print(4, "delta T auto: %d, counter: %d", autoDeltaT, autoLoopCounter);
		autoLastTime = autoNow;

		deltaX = targetX - posX;
		deltaY = targetY - posY;
		// pros::lcd::print(3, "tx: %.2f, ty:%.2f", targetX, targetY); //debug

		//https://www.desmos.com/calculator/ryfqu5k1nw
		angleToTarget = wrapAngle(atan2f(targetX - posX, targetY - posY));
		errA = wrapAngle(angleToTarget - posA);
		turningController.step(fabs(errA) > 0.5 * PI ? signDouble(errA) * PI - errA : errA);

		// pros::lcd::print(7, "target:%.2f, errA:%.2f", angleToTarget, errA); //debug

		errD = sqrt(deltaY * deltaY + deltaX * deltaX); //how far away am I to the target?
		powerDistanceController.step(errD);
		// pros::lcd::print(4, "eA: %.3f, eD:%.3f", errA, errD); //debug

		power = auto_power_angle_function(errA, tolA) * powerDistanceController.getOutput() * -1 * powerFactor; //from -1 to 1, go foward?

		pros::lcd::print(4, "A:%.2f B:%.2f C:%.2f", auto_power_angle_function(errA, tolA), powerDistanceController.getOutput(), powerFactor); //debug

		turning = fabs(errA) < 0.5 * PI ? turningController.getOutput() * -1 : turningController.getOutput();

		turning = turning * steeringFactor;
		if (!redSide)
			turning = turning * -1;

		// turning = lastTurn + clampDouble(turning - lastTurn, autoDeltaT * (1.0/30.0));
		// pros::lcd::print(7, "clamped:%.3f", autoDeltaT); //debug

		if (fabs(power) + fabs(turning) > 1)
			power = signDouble(power) * (1 - fabs(turning)); //decrease power to prioritize turning
		if(firstTime){
			if(signDouble(power)!=signDouble(lastPower))lastPower=0;
			else if(signDouble(power)==signDouble(lastPower))lastPower = lastPower/3;
			firstTime = false;
		}

		power = lastPower + clampDouble(power - lastPower, autoDeltaT * (1.0 / accelTimeMili));

		// master.print(2, 1, "t: %f", turning);
		// pros::lcd::print(7, "p: %.3f, t: %.3f, ", power, turning);
		// pros::lcd::print(6, "PA: %.4f, PS: %.4f", auto_power_angle_function(errA, tolA), power);
		lastPower = power;
		lastTurn = turning;

		// pros::lcd::print(4, "#1: %f, #2: %f", (PI - fabs(errA) )/ PI, clampDouble(errD * powerP, 1));
		left = power + turning;
		right = power - turning;
		// pros::lcd::print(4, "left: %.2f, right: %.2f", left, right);

		//handling stalling
		if (getDriveStalling())
		{
			stalling++;
		}
		else
		{
			if (stalling > 0)
				stalling -= 5;
		}
		if (getDriveSteadyState())
		{
			steadyState++;
		}
		else
		{
			if (steadyState > 0)
				steadyState -= 5;
		}
		if (stalling+steadyState > 20)
		{
			power = power + (1 - power) * (((stalling+steadyState - 20.0) / 50.0));
			turning = turning + (1 - turning) * (((stalling+steadyState - 20.0) / 50.0));
		}
		if (stalling+steadyState > 80)
		{
			driveFlag = 0;
			break;
		}
		lastErrA = errA;
		lastErrD = errD;

		pros::lcd::print(6, "p:%.3f, t:%.3f", power, turning);
		pros::lcd::print(7, "stalling: %d, %s", stalling, getDriveStalling() ? "TRUE" : "FALSE");

		left = clampDouble(left * MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
		right = clampDouble(right * MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
		// pros::lcd::print(5, "left: %.2f, right: %.2f, o:%.2f", left, right,overallSpeedFactor);

		left_fwd_mtr.moveVoltage(left * overallSpeedFactor);
		left_rear_mtr.moveVoltage(left * overallSpeedFactor);
		right_fwd_mtr.moveVoltage(right * overallSpeedFactor);
		right_rear_mtr.moveVoltage(right * overallSpeedFactor);

		pros::delay(10);
	}
	driveFlag = 0;
	if (stopAtEnd)
	{
		left_fwd_mtr.moveVoltage(0);
		left_rear_mtr.moveVoltage(0);
		right_fwd_mtr.moveVoltage(0);
		right_rear_mtr.moveVoltage(0);
	}
}

void drive_line(MyPoint inTargetPoint, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance, double inaTolerance, double inaccelmilli, bool inStopAtEnd) //a from -pi to pi
{
	targetX = inTargetPoint.x;
	targetY = inTargetPoint.y;
	overallSpeedFactor = inOverallSpeed;
	powerFactor = inPowerfactor;
	steeringFactor = inTurningfactor;
	tolD = indTolerance;
	tolA = inaTolerance;
	accelTimeMili = inaccelmilli;
	stopAtEnd = inStopAtEnd;
	if (driveFlag == 0)
		pros::Task auto_line_task(auto_line_task_fn, (void *)0, "auto line task");
	driveFlag = 2;
}

double targetA, turnAccelTimeMili;
void auto_turn_task_fn(void *param) //turns then drives to a point in a striaght line
{
	int autoNow = pros::millis();
	int autoLastTime = autoNow;
	int autoDeltaT = 0;
	int autoLoopCounter = 0;
	int stalling = 0;
	auto turningController = okapi::IterativeControllerFactory::posPID(4, 0.01, 0.02);
	turningController.setTarget(0);
	lastTurn = 0;
	errA = wrapAngle(targetA - posA);
	while (driveFlag == 2 && fabs(errA) > fabs(tolA))
	{
		if (turningController.isSettled())
			turningController.setTarget(0);
		//counting stuffs/finding delta T
		autoLoopCounter++;
		autoNow = pros::millis();
		autoDeltaT = autoNow - autoLastTime;
		autoLastTime = autoNow;

		//just stepping a pid
		errA = wrapAngle(targetA - posA);
		pros::lcd::print(6, "errA %.2f", errA);

		turningController.step(errA);
		turning = turningController.getOutput() * -1;
		if (!redSide)
			turning = turning * -1;
		// if(fabs(turning)<0.2)turning = signDouble(turning) * 2;

		// turning = lastTurn + clampDouble(turning - lastTurn, autoDeltaT * (1 / turnAccelTimeMili));
		lastTurn = turning;
		turning = clampDouble(turning * MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
		//handling stalling
		if (getDriveStalling())
		{
			stalling++;
		}
		else
		{
			if (stalling > 0)
				stalling -= 5;
		}
		if (stalling > 30)
		{
			turning = turning + (1 - turning) * (((stalling - 30.0) / 80.0));
		}
		if (stalling > 200)
		{
			driveFlag = 0;
			break;
		}

		//pros::lcd::print(4, "turn:%d", turning);
		left_fwd_mtr.moveVoltage(turning * steeringFactor);
		left_rear_mtr.moveVoltage(turning * steeringFactor);
		right_fwd_mtr.moveVoltage(-1 * turning * steeringFactor);
		right_rear_mtr.moveVoltage(-1 * turning * steeringFactor);
		pros::delay(10);
	}
	driveFlag = 0;
	if (stopAtEnd)
	{
		left_fwd_mtr.moveVoltage(0);
		left_rear_mtr.moveVoltage(0);
		right_fwd_mtr.moveVoltage(0);
		right_rear_mtr.moveVoltage(0);
	}
}

//from -pi to pi, positive is clockwise
void turn_angle(double inTargetA, double inTurningfactor, double inaTolerance, double inturnAccelTimeMili, bool inStopAtEnd) //a from -pi to pi
{
	targetA = inTargetA;
	steeringFactor = inTurningfactor;
	tolA = inaTolerance;
	turnAccelTimeMili = inturnAccelTimeMili;
	stopAtEnd = inStopAtEnd;
	// accelTimeMili = inaccelmilli;
	if (driveFlag == 0)
		pros::Task auto_turn_task(auto_turn_task_fn, (void *)0, "auto turn task");
	driveFlag = 2;
}

std::vector<MyLine> pathToFollow;
double minLookAhead, maxLookAhead, lookAheadIncreaseDistance;
void pure_pursuit_task_fn(void *param) //turns then drives to a point in a striaght line
{
	int autoLoopCounter = 0;
	int autoLastTime = 0;
	int autoNow = pros::millis();
	int autoDeltaT;
	bool startedAutoLineTask = false;
	while (driveFlag == 2)
	{
		autoLoopCounter++;
		autoNow = pros::millis();
		autoDeltaT = autoNow - autoLastTime;
		// pros::lcd::print(2, "delta T auto: %d, counter: %d", autoDeltaT, autoLoopCounter);
		autoLastTime = autoNow;

		MyPoint robotPosition = {posX, posY};
		TargetPoint tempTarget = pathToFollow[0].getClosestPoint(robotPosition);
		TargetPoint closestTarget = tempTarget;
		int closestLineCount = 0;

		//finding the targetPoint with the least distance.....
		for (int i = 1; i < pathToFollow.size(); i++)
		{
			tempTarget = pathToFollow[i].getClosestPoint(robotPosition);
			if (tempTarget.d < closestTarget.d)
			{
				closestTarget = tempTarget;
				closestLineCount = i;
			}
		}

		double lookAheadDistance = closestTarget.d < lookAheadIncreaseDistance ? (minLookAhead - maxLookAhead) / (lookAheadIncreaseDistance) * (closestTarget.d) + maxLookAhead : minLookAhead;
		TargetPoint lookAheadTarget = pathToFollow[closestLineCount].getLookAheadPoint({closestTarget.x, closestTarget.y}, lookAheadDistance);

		//if the target is already too far and is off the line
		if (!lookAheadTarget.online)
		{
			//and if this line is not the last line
			if (closestLineCount < pathToFollow.size() - 1)
			{
				//just jump to the next line
				closestLineCount += 1;
				closestTarget = pathToFollow[closestLineCount].getClosestPoint(robotPosition);
			}
		}

		//remove all the lines before this one
		for (int i = 0; i < closestLineCount; i++)
		{
			pathToFollow.erase(pathToFollow.begin());
		}

		//now we know our target, just go after it
		targetX = lookAheadTarget.x;
		targetY = lookAheadTarget.y;
		if (!startedAutoLineTask)
		{
			startedAutoLineTask = true;
			pros::Task auto_line_task(auto_line_task_fn, (void *)0, "auto line task"); //auto line task will set drive flag to 0 when finished
		}
		pros::delay(20);
	}
}

//recommanded minlookahead: 1.5, maxLookAhead: 5, high aTolerance (10degrees ish?) idk
void follow_path(std::vector<MyPoint> &inPathToFollow, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance, double inaTolerance, double inaccelmilli, double inMaxLookAhead, double inMinLookAhead, double inlookAheadIncreaseDistance, bool inStopAtEnd) //a from -pi to pi
{
	pathToFollow.clear();
	for (int i = 0; i < inPathToFollow.size() - 1; i++)
	{
		pathToFollow.push_back({inPathToFollow[i], inPathToFollow[i + 1]});
	}

	overallSpeedFactor = inOverallSpeed;
	powerFactor = inPowerfactor;
	steeringFactor = inTurningfactor;
	tolD = indTolerance;
	tolA = inaTolerance;
	accelTimeMili = inaccelmilli;
	minLookAhead = inMinLookAhead;
	maxLookAhead = inMaxLookAhead;
	lookAheadIncreaseDistance = inlookAheadIncreaseDistance;
	stopAtEnd = inStopAtEnd;
	if (driveFlag == 0)
		pros::Task pure_pursuit_task(pure_pursuit_task_fn, (void *)0, "pure pursuit task");
	driveFlag = 2;
}
