#pragma once

#include "main.h"
#define OI_INTAKE_IN master.get_digital(DIGITAL_L1)
#define OI_INTAKE_OUT master.get_digital(DIGITAL_L2)

void handleManualIntake();
