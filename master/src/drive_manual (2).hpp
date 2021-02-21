#pragma once

#define OI_DRIVE_POWER master.get_analog(ANALOG_LEFT_Y)
#define OI_DRIVE_STEER master.get_analog(ANALOG_RIGHT_X)
#define MANUAL_DRIVE_CURVATURE 2
#define HOLD_TOLERANCE 3
void initDriveCurveLookup();
int lookupDriveCurve(int input);
void handleManualDrive();
void drive_hold();
