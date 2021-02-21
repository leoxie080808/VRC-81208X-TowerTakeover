#include "main.h"
#include "odometry.hpp"
#include "sdcard.hpp"

void menu_task_fn(void *param) //turns then drives to a point in a striaght line
{
  int menuLimit = 4;
  int menuSelected = 0;
  int subMenu1Selected = 0;
  int subMenu1Limit = 3;
  bool inSubMenu = false;
  while (menuFlag)
  {
    if (menuSelected < 0)
      menuSelected = menuLimit;
    if (menuSelected > menuLimit)
      menuSelected = 0;
    switch (menuSelected)
    {
    case 0: //start of blue and red side selection
      pros::lcd::print(2, "\t> 0: Side Toggle");
      pros::lcd::print(3, "\t    %s", redSide ? "Red" : "Blue");
      if (ok_btn.changedToPressed()){
        redSide = !redSide;
        writeConfigs();
        master.print(1, 1, "A: %d, S: %s", autonSelection, redSide?"red_":"blue");
      }
      if (up_btn.changedToPressed())
      {
        menuSelected++;
      }
      else if (down_btn.changedToPressed())
      {
        menuSelected--;
      }
      break; //end of blue and red side selection
    case 1:
      pros::lcd::print(2, "\t> 1: \"In Competition\" Toggle");
      pros::lcd::print(3, "\t    %s", debugging ? "Debugging" : "Competition");
      if (ok_btn.changedToPressed()){
        debugging = !debugging;
        writeConfigs();
      }
      if (up_btn.changedToPressed())
      {
        menuSelected++;
      }
      else if (down_btn.changedToPressed())
      {
        menuSelected--;
      }
      break; //end of blue and red side selection
      break;
    case 2: //start of auton path adjustment
      if (inSubMenu)
      {
        pros::lcd::print(2, "\t  2: Adjust Auton Path");
        if (subMenu1Selected < 0)
          subMenu1Selected = subMenu1Limit;
        if (subMenu1Selected > subMenu1Limit)
          subMenu1Selected = 0;
        switch (subMenu1Selected)
        {
        case 0:
          pros::lcd::print(3, "> 0: thirdStackX : %.2f", thirdStackX);
          if (ok_btn.isPressed())
          { //if ok is held down
            if (up_btn.changedToPressed())
            {
              thirdStackX++;
            }
            else if (down_btn.changedToPressed())
            {
              thirdStackX--;
            }
            break;
          case 1:
            pros::lcd::print(3, "> 1: forthStackX : %.2f", forthStackX);
            if (ok_btn.isPressed())
            { //if ok is held down
              if (up_btn.changedToPressed())
              {
                forthStackX++;
              }
              else if (down_btn.changedToPressed())
              {
                forthStackX--;
              }
            }
            break;
          case 2:
            pros::lcd::print(3, "> 2: startingY : %.2f", startingY);
            if (ok_btn.isPressed())
            { //if ok is held down
              if (up_btn.changedToPressed())
              {
                startingY++;
              }
              else if (down_btn.changedToPressed())
              {
                startingY--;
              }
            }
            break;
          case 3:
            pros::lcd::print(3, "> 3: Drive Base Width: %.2f", driveBaseWidthInch);
            if (ok_btn.isPressed())
            { //if ok is held down
              if (up_btn.changedToPressed())
              {
                driveBaseWidthInch += 0.05;
              }
              else if (down_btn.changedToPressed())
              {
                driveBaseWidthInch -= 0.05;
              }
            }
            break;
          }
        }
        if (up_btn.isPressed() && down_btn.isPressed() && !ok_btn.isPressed())
        {
          inSubMenu = false;
        }
        if (up_btn.changedToPressed())
        {
          subMenu1Selected++;
        }
        else if (down_btn.changedToPressed())
        {
          subMenu1Selected--;
        }
      }
      else
      {
        pros::lcd::print(2, "\t> 2: Adjust Auton Path");
        pros::lcd::print(3, "_");
        if (up_btn.changedToPressed())
        {
          menuSelected++;
        }
        else if (down_btn.changedToPressed())
        {
          menuSelected--;
        }
        if (ok_btn.changedToPressed())
        {
          inSubMenu = true;
        }
      }
      break; //end of auton path adjustment
    case 3:  //start of auton path selection
      pros::lcd::print(2, "\t> 3: Auton PATH Selection");
      if(autonSelection == 0){
        pros::lcd::print(3, "\t    7 cube");
      }else if(autonSelection == 1){
        pros::lcd::print(3, "\t    5 cube");
      }else if(autonSelection == 2){
        pros::lcd::print(3, "\t    upper auton");
      }else if(autonSelection == 3){
        pros::lcd::print(3, "\t    Programming Skills");
      }

      if (ok_btn.changedToPressed()){
        if(autonSelection == 0)autonSelection = 1;
        else if(autonSelection == 1)autonSelection = 2;
        else if(autonSelection == 2)autonSelection = 3;
        else if(autonSelection == 3)autonSelection = 0;
        writeConfigs();
        master.print(1, 1, "A: %d, S: %s", autonSelection, redSide?"red":"blue");
        switch (autonSelection)
        {
        case 0:
          resetEverything(thirdStackX, startingY, 0);
          break;
        case 1:
          resetEverything(forthStackX, startingY, 0);
          break;
        case 2:
          resetEverything(firstStackX, startingY, 0);
          break;
        case 3:
          resetEverything(thirdStackX, startingY, 0);
          break;
        }
      }
      if (up_btn.changedToPressed())
      {
        menuSelected++;
      }
      else if (down_btn.changedToPressed())
      {
        menuSelected--;
      }
      break; //end of blue and red side selection
      break; //end of auton path selection
    case 4:  //start of auton path adjustment
      if (inSubMenu)
      {
        pros::lcd::print(2, "\t  4: Resetting Everything");
        pros::lcd::print(3, ">    Are you sure you want to reset everything?");
        if (ok_btn.changedToPressed())
        { //if ok is held dow
          resetEverything(0, startingY, 0);
          pros::lcd::print(5, ">    Resetted things!");
          inSubMenu = false;
        }
        if (up_btn.isPressed() && down_btn.isPressed() && !ok_btn.isPressed())
        {
          inSubMenu = false;
        }
      }
      else
      {
        pros::lcd::print(2, "\t> 4: Resetting Everything");
        pros::lcd::print(3, "_");
        if (up_btn.changedToPressed())
        {
          menuSelected++;
        }
        else if (down_btn.changedToPressed())
        {
          menuSelected--;
        }
        if (ok_btn.changedToPressed())
        {
          inSubMenu = true;
        }
      }
      break; //end of resetting
    }
    pros::delay(20);
  }
}

void startMenuTask()
{
  if (menuFlag == false)
    pros::Task menu_task(menu_task_fn, (void *)0, "menu task");
  menuFlag = true;
}
void endMenuTask()
{
  menuFlag = false;
}
