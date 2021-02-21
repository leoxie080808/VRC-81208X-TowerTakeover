#include "tray.hpp"
#include "main.h"
void trayOut(double multiplier){
  tray_mtr.move(multiplier * (tray_enc.get() > 0 ? MAX_TRAY_SPEED : (TRAY_STOP_ENC - tray_enc.get()) / (TRAY_STOP_ENC) * (MAX_TRAY_SPEED - MIN_TRAY_SPEED) + MIN_TRAY_SPEED));
}

void trayIn(double multiplier){
  tray_mtr.move(multiplier * (tray_enc.get() > 0 ? -MAX_TRAY_SPEED : -((TRAY_STOP_ENC - tray_enc.get()) / (TRAY_STOP_ENC) * (MAX_TRAY_SPEED - MIN_TRAY_SPEED) + MIN_TRAY_SPEED)));
}

void handleManualTray(){
  if (OI_TRAY_OUT)
  {
    trayOut(1);
  }
  else if (OI_TRAY_IN)
  {
    trayIn(1);
  }
  else
  {
    tray_mtr.move(0);
  }
}
