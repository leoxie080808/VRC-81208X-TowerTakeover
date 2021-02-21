#include "tray.hpp"
#include "util.hpp"
#include "main.h"


//https://www.desmos.com/calculator/bvvs8h6ywo
double m; //m for slope
int trayStalling = 0;

bool getTrayStalling(){
  return tray_mtr.getTorque()>0.6 && tray_mtr.getActualVelocity()<20;
}

void handleTrayButton(){
  if(tray_btn.changedToReleased())tray_enc.reset();
}

//positive = out, negative = in, from 1 to -1
void moveTrayManual(double multiplier){
  handleTrayButton();
  trayFlag = 0;
  multiplier=clampDouble(multiplier, 1);
  m = (MIN_TRAY_SPEED - MAX_TRAY_SPEED)/(TRAY_STOP_ENC-TRAY_SLOWDOWN_ENC);
  if( multiplier<0 && !tray_btn.isPressed()){
    tray_mtr.move(0);
  }
  else if((multiplier<0&&tray_enc.get() < 10) || (multiplier>0&&tray_enc.get() > TRAY_STOP_ENC)){//if ur breaking the tray
    tray_mtr.move(multiplier * MIN_TRAY_SPEED * 0.5);
    // if(debugging)tray_mtr.move(multiplier * MIN_TRAY_SPEED * 0.5); //move slowly if we r just testing
    // else tray_mtr.move(0); //stop when we r in comp
  }else{
    tray_mtr.move(multiplier * (tray_enc.get() < TRAY_SLOWDOWN_ENC ? MAX_TRAY_SPEED : m*(tray_enc.get()-TRAY_SLOWDOWN_ENC) + MAX_TRAY_SPEED));//or uk, just move normally
  }
  // pros::lcd::print(3, "tray_enc:%.0f, value:%f", tray_enc.get(),tray_enc.get() < TRAY_SLOWDOWN_ENC ? MAX_TRAY_SPEED : m*(tray_enc.get()-TRAY_SLOWDOWN_ENC) + MAX_TRAY_SPEED); //debug
}

void moveTrayPosition(double position){
  trayFlag = 1;
  tray_controller.flipDisable(false);
  tray_controller.setTarget(position);
}

void handleManualTray(){
  if(getTrayStalling()){
    master.rumble(".");
    master.print(1,1,"TRAY");
    trayStalling++;
  }else{
    if(trayStalling>0)trayStalling-=5;
    if(trayStalling<0)trayStalling = 0;
  }
  if(trayStalling>15){//with pros delay on 20ms in drivercontrol
    if(trayFlag == 1){
      tray_controller.flipDisable(true);
    }
    tray_mtr.moveVoltage(0);
    return;
  }


  if (OI_TRAY_OUT)
  {
    moveTrayManual(1);
  }
  else if (OI_TRAY_IN)
  {
    moveTrayManual(-1);
  }
  else if (trayFlag == 0)
  {
    tray_mtr.move(0);
  }
  // if (OI_TRAY_OUT)
  // {
  //   moveTrayPosition(1000);
  // }
  // else if (OI_TRAY_IN)
  // {
  //   tray_mtr.move(0);
  // }
}
