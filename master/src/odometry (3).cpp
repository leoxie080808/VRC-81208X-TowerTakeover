#include "odometry.hpp"
#include "util.hpp"
#include "main.h"

/*
x: <- negative, -> positive
y: v negative, A positive
a: clockwise: positive, counterclockwise: negative
*/

extern double posX = 0;
extern double posY = 0;
extern double posA = 0;//clockwise = positive on red side
double actualA = 0;
double resetA, curL, curR, curB, lastL, lastR, lastB, deltaL, deltaR, deltaB, deltaX, deltaY, deltaA, newA, mediumA, theta, radius, newminA;
bool trackingPosition = true, resetted = false;
void resetEverything(double x,double y,double a){
	resetted = false;
	posX = x;
	posY = y;
	actualA = resetA = posA = a;
	left_fwd_enc.reset();
	left_rear_enc.reset();
	right_fwd_enc.reset();
	right_rear_enc.reset();
  back_enc.reset();
	resetted = true;
}

int delayCounter = 30;
void track_position_task_fn(void *param)
{
	lastL = avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet());
  lastR = avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet());
  lastB = back_enc.controllerGet();
	while (trackingPosition)
	{
		if(!resetted){
			pros::delay(5);
			continue;
		}else{
			if(delayCounter>=5){
				// pros::lcd::print(1, "delayCounter: %d", delayCounter);
				delayCounter=delayCounter-1;
				pros::delay(5);
				continue;
			}
		}



    curL = avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet());
    curR = avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet());
    curB = back_enc.controllerGet();

		deltaL = (curL - lastL) * WHEEL_ENCODER_TO_INCH;   //amount left side moved
		deltaR = (curR - lastR) * WHEEL_ENCODER_TO_INCH; //amount right side moved
		deltaB = (curB - lastB) * TRACKING_WHEEL_ENCODER_TO_INCH;	//amount back tracking wheel moved

		pros::lcd::print(5, "l: %.2f, r: %.2f, b: %.2f", deltaL, deltaR, deltaB);

		if(fabs(deltaL) > 1 || fabs(deltaR)>1 || fabs(deltaB)>1){
			lastL = curL;
			lastR = curR;
			lastB = curB;
			pros::delay(5);
			continue;
		}

    lastL = curL;
    lastR = curR;
    lastB = curB;

		newA = resetA + (curL * WHEEL_ENCODER_TO_INCH - curR * WHEEL_ENCODER_TO_INCH)/driveBaseWidthInch;
		// if(!redSide)newA=newA*-1;
		deltaA = newA - actualA;

    deltaB = deltaB-TRACKING_WHEEL_TO_CENTER_INCH*deltaA;//might need to be reverted

		//https://imgur.com/TvXXTGE
    //deltaX and deltaY are using local coordinates
		if (deltaA == 0)
		{
      deltaX = deltaB;
      deltaY = deltaR;
		}
		else
		{
      deltaX = (2*sin(deltaA/2))*(deltaB/deltaA + TRACKING_WHEEL_TO_CENTER_INCH);
      deltaY = (2*sin(deltaA/2))*(deltaR/deltaA +driveBaseWidthInch/2);
		}

    mediumA = actualA + deltaA/2;
    theta = atan2f(deltaY, deltaX);
    radius = sqrt(deltaX*deltaX + deltaY*deltaY);
    theta = theta-mediumA;
    deltaX = radius*cos(-1*theta);
    deltaY = radius*sin(-1*theta);

    // if(redSide)posA = newA;
		// else posA = newA * -1;
		actualA = newA;
    if(redSide)posA = actualA;
		else posA = actualA * -1;
		if(redSide)posX += deltaX;
		else posX -= deltaX;
    posY -= deltaY;
		pros::lcd::print(1, "x: %.2f, y: %.2f, a: %.2f", posX, posY, r2d(posA));
		// pros::lcd::print(2, "l: %.1f, r: %.1f, b: %.1f", avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet()) * WHEEL_ENCODER_TO_INCH, avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet()) * WHEEL_ENCODER_TO_INCH, back_enc.controllerGet()* TRACKING_WHEEL_ENCODER_TO_INCH);
		pros::delay(5);
	}
}

void startTrackPosition(){
  trackingPosition = true;
  pros::Task track_position_task(track_position_task_fn, (void *)0, "track position task");
}

void endTrackPosition(){
  trackingPosition = false;
}
