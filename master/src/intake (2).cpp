#include "intake.hpp"
#include "main.h"

int speed = 200;
void handleManualIntake(){
  switch(armFlag){
    case 4:
    speed = 100;
    break;
    case 2:
    speed = 125;
    break;
    case 3:
    case 1:
    case 0:
    case -1:
    default:
    speed = 200;
    break;
  }
  if (OI_INTAKE_IN)
  {
    intake_mtrgrp.moveVelocity(speed);
  }
  else if (OI_INTAKE_OUT)
  {
    intake_mtrgrp.moveVelocity(0-speed);
  }
  else
  {
    intake_mtrgrp.moveVelocity(0);
  }
}
