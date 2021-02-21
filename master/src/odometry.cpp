#include "odometry.hpp"
#include "util.hpp"
#include "main.h"

extern double posX = 0;
extern double posY = 0;
extern double posA = 0;//TODO: is clockwise positive or counterclockwise?
double resetA, curL, curR, curB, lastL, lastR, lastB, deltaL, deltaR, deltaB, deltaX, deltaY, deltaA, newA, mediumA, theta, radius, newminA;
bool trackingPosition = true;
void resetEverything(double x,double y,double a){
	posX = x;
	posY = y;
	resetA = posA = a;
	left_fwd_enc.reset();
	left_rear_enc.reset();
	right_fwd_enc.reset();
	right_rear_enc.reset();
}

void track_position_task_fn(void *param)
{
	lastL = avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet());
  lastR = avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet());
  lastB = (curB - lastB) * TRACKING_WHEEL_ENCODER_TO_INCH;
	while (trackingPosition)
	{
    curL = avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet());
    curR = avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet());
    curB = back_enc.controllerGet();

		deltaL = (curL - lastL) * WHEEL_ENCODER_TO_INCH;   //amount left side moved
		deltaR = (curR - lastR) * WHEEL_ENCODER_TO_INCH; //amount right side moved
		deltaB = (curB - lastB) * TRACKING_WHEEL_ENCODER_TO_INCH;	//amount back tracking wheel moved

    lastL = curL;
    lastR = curR;
    lastB = curB;

		newA = resetA + (curL * WHEEL_ENCODER_TO_INCH - curR * WHEEL_ENCODER_TO_INCH)/DRIVE_BASE_WIDTH_INCH;

		deltaA = newA - posA;

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
      deltaX = (2*sin(deltaA/2))*(deltaB/deltaA + TRACKING_WHEEL_TO_CENTER_INCH); //step 8
      deltaY = (2*sin(deltaA/2))*(deltaR/deltaA +DRIVE_BASE_WIDTH_INCH/2);
		}

    mediumA = posA + deltaA/2;
    theta = atan2f(deltaX, deltaY);
    radius = sqrt(deltaX*deltaX + deltaY*deltaY);
    theta = theta-mediumA;                          //step 10
    deltaX = radius*cos(-1*theta);
    deltaY = radius*sin(-1*theta);

    posA = newA;
    posX -= deltaX;
    posY += deltaY;
		pros::lcd::print(1, "x: %.2f, y: %.2f, a: %.2f", posX, posY, r2d(posA));
		pros::lcd::print(2, "l: %.1f, r: %.1f, b: %.1f", avgDouble(left_fwd_enc.controllerGet(), left_rear_enc.controllerGet()) * WHEEL_ENCODER_TO_INCH, avgDouble(right_fwd_enc.controllerGet(), right_rear_enc.controllerGet()) * WHEEL_ENCODER_TO_INCH, back_enc.controllerGet()* TRACKING_WHEEL_ENCODER_TO_INCH);
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
