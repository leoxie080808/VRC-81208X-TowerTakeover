#pragma once

#define DRIVE_BASE_WIDTH_INCH 11.75
#define WHEEL_ENCODER_TO_INCH 4.1 * PI / 360
#define TRACKING_WHEEL_ENCODER_TO_INCH 3.25 * PI / 360
#define TRACKING_WHEEL_TO_CENTER_INCH 3
void resetEverything(double x,double y,double a);
void startTrackPosition();
void endTrackPosition();
extern double posX, posY, posA;
