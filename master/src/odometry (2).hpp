#pragma once

#define WHEEL_ENCODER_TO_INCH 4.05 * PI / 360
#define TRACKING_WHEEL_ENCODER_TO_INCH 3.24* PI / 360
#define TRACKING_WHEEL_TO_CENTER_INCH 3.125
void resetEverything(double x,double y,double a);
void startTrackPosition();
void endTrackPosition();
extern double posX, posY, posA;
