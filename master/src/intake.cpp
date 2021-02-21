#include "intake.hpp"

void handleManualIntake(){
  if (OI_INTAKE_IN)
  {
    intake_mtrgrp.moveVelocity(200);
  }
  else if (OI_INTAKE_OUT)
  {
    intake_mtrgrp.moveVelocity(-200);
  }
  else
  {
    intake_mtrgrp.moveVelocity(0);
  }
}
