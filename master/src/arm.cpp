#include "arm.hpp"
#include "main.h"
#include "tray.hpp"

int armStalling = 0;
bool getArmStalling(){
  return arm_mtr.getTorque()>0.7 && arm_mtr.getActualVelocity()<20;
}

bool tempA = false, tempB = false;
void handleManualArm(){
  tempA = tray_enc.get()>1250;
  if(tempA!=tempB){
    if(tempA){
      if(armFlag > 0){
        arm_controller.flipDisable(true);
        arm_controller.setMaxVelocity(200);
        arm_controller.flipDisable(false);
      }
    }
  }
  tempB = tempA;
  if(getArmStalling()){
    master.rumble(".");
    master.print(1,1,"ARM");
    armStalling++;
  }else{
    if(armStalling>0)armStalling-=5;
    if(armStalling<0)armStalling = 0;
  }
  if(armStalling>15){
    if(armFlag != -1){
      arm_controller.flipDisable(true);
    }
    arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    arm_mtr.moveVoltage(0);
    return;
  }


  if(OI_ARM_UP){
    armFlag = -1;
    arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    arm_mtr.moveVelocity(200);
  }else if(OI_ARM_DOWN){
    armFlag = -1;
    arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    arm_mtr.moveVelocity(-200);
  }else if(armFlag == -1){
    if(arm_enc.get()<200){
      arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      armFlag = 0;
    }else if(arm_enc.get()<850)armFlag = 1;
    else if(arm_enc.get()<975)armFlag = 2;
    else if(arm_enc.get()<1100)armFlag = 3;
    else armFlag = 4;

    arm_mtr.moveVelocity(0);
  }
  if (OI_ARM_LOW_TOWER)
  {
    if(armFlag==1){
      armFlag = 0;
      moveArmDown();
    }else if(armFlag==2){
      armFlag = 1;
      moveArmLowTower(750);
    }else{
      armFlag = 2;
      moveArmLowTower(900);
    }
  }
  else if (OI_ARM_MID_TOEWR)
  {
    if(armFlag == 3){
      armFlag = 0;
      moveArmDown();
    }else if(armFlag == 4){
      armFlag = 3;
      moveArmMidTower(1050);
    }else{
      armFlag = 4;
      moveArmMidTower(1150);
    }
  }
  // if(master.get_digital(DIGITAL_X))arm_mtr.moveVelocity(200);
  // else if(master.get_digital(DIGITAL_B))arm_mtr.moveVelocity(-200);
  // else arm_mtr.moveVelocity(0);
  if(armFlag == 0){
    if(arm_enc.get()>900&&(armStalling<10))arm_mtr.moveVoltage(-12000);
    else if (arm_enc.get()<600)arm_mtr.moveVoltage(0);
    else arm_mtr.moveVoltage((arm_enc.get()-600)/300*-6000 - 6000);
  }
}

void moveArmDown(){
  if(arm_enc.get()>600){
    if(tray_enc.get()<1250){
      tray_controller.setMaxVelocity(200);
      moveTrayPosition(1300);
      // arm_controller.setMaxVelocity(150);
      // arm_controller.setTarget(25);
    }else{
      tray_controller.setMaxVelocity(150);
      moveTrayPosition(0);
      // arm_controller.setMaxVelocity(200);
      // arm_controller.setTarget(25);
    }
  }else{
    tray_controller.setMaxVelocity(200);
    moveTrayPosition(0);
    // arm_controller.setMaxVelocity(200);
    // arm_controller.setTarget(25);
  }
  arm_controller.flipDisable(true);
  arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void moveArmLowTower(int height){
  // arm_controller.flipDisable(true);
  tray_controller.flipDisable(true);
  tray_controller.setMaxVelocity(200);
  tray_controller.flipDisable(false);

  arm_controller.setMaxVelocity(80);
  arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  arm_controller.setTarget(height);
  arm_controller.flipDisable(false);
  if(tray_enc.get()<1250)moveTrayPosition(1300);
}

void moveArmMidTower(int height){
  // arm_controller.flipDisable(true);
  tray_controller.flipDisable(true);
  tray_controller.setMaxVelocity(200);
  tray_controller.flipDisable(false);
  arm_controller.setMaxVelocity(80);
  arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  arm_controller.setTarget(height);
  arm_controller.flipDisable(false);
  if(tray_enc.get()<1250)moveTrayPosition(1300);
}
