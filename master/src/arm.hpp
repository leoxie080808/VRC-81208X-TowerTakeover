#pragma once

#define OI_ARM_LOW_TOWER master.get_digital_new_press(DIGITAL_RIGHT)
#define OI_ARM_MID_TOEWR master.get_digital_new_press(DIGITAL_Y)
#define OI_ARM_UP master.get_digital(DIGITAL_UP)
#define OI_ARM_DOWN master.get_digital(DIGITAL_DOWN)

void handleManualArm();
void moveArmDown();
void moveArmLowTower(int height);
void moveArmMidTower(int height);
