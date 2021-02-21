#include "main.h"
#include "selfcheck.hpp"
#include "util.hpp"
#include <stdio.h>


// char lastMessage[100];
// void printToController(const char message[]){
//   //TODO: fix this
//   if(strcmp(message, lastMessage)!=0) {
//       strncpy(lastMessage, message, sizeof(lastMessage));
//       master.print(1,1,"%s",message);
//   }
//   master.print(1,1,"%s",message);
//   pros::lcd::print(4,message,0);
// }



bool selfcheckFlag = false;
int portsUsed[] = {1,2,3,4,5,6,8,10};
char temp [100];
int lastPort = -1;
bool errFlag = false, lastErrFlag = false;
void selfcheck_task_fn(void *param)
{
	while (selfcheckFlag)
	{
		errFlag = false;
    for(int i  = 0; i<8; i++){
      if(pros::c::motor_get_faults(portsUsed[i])==__INT_MAX__){
				errFlag = true;
				if(lastPort != i || lastErrFlag == false){
					lastPort = i;
					master.print(1,1,"Motor %d DISCONNECTED", portsUsed[i]);
					// std::sprintf(temp, "Motor %d DISCONNECTED", portsUsed[i]);
					// printToController(temp);
					// master.print(1,1,"!!Motor %d is DISCONNECTED", i);
				}
      }
    }
		if(lastErrFlag!=errFlag){
			lastErrFlag = errFlag;
			if(!errFlag){
				master.print(1,1,"_________________________");
			}
		}
		pros::delay(200);
		if(errFlag)master.rumble("---");
		pros::delay(200);
	}
}

void startSelfcheck(){
  selfcheckFlag = true;
  pros::Task selfcheck_task(selfcheck_task_fn, (void *)0, "selfcheck task");
}

void endSelfcheck(){
  selfcheckFlag = false;
}
