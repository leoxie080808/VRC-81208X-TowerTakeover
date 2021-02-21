#pragma once

#define OI_TRAY_OUT master.get_digital(DIGITAL_R2)
#define OI_TRAY_IN master.get_digital(DIGITAL_R1)
#define MAX_TRAY_SPEED 127
#define MIN_TRAY_SPEED 30
#define TRAY_STOP_ENC -1280

void handleManualTray();
void trayOut(double multiplier);
void trayIn(double multiplier);
