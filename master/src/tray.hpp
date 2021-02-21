#pragma once

#define OI_TRAY_OUT master.get_digital(DIGITAL_R2)
#define OI_TRAY_IN master.get_digital(DIGITAL_R1)
#define MAX_TRAY_SPEED 127.0
#define MIN_TRAY_SPEED 100.0
#define TRAY_STOP_ENC 2700.0
#define TRAY_SLOWDOWN_ENC 2000.0

void handleManualTray();
void moveTrayManual(double multiplier);
void moveTrayPosition(double position);
