#include "main.h"
#include "sdcard.hpp"
#include <fstream>

bool fileExists(const char *filename) {
  FILE* file = fopen(filename, "r");
  if(file == NULL){
    fclose(file);
    return false;
  }else {
    fclose(file);
    return true;
  }
}

void createFile(const char *filename){
  FILE* file = fopen(filename, "w");
  fclose(file);
}


//overrides the current config
void writeConfigs(){
  std::ofstream configFile(CONFIGFILE);
  configFile << std::to_string(autonSelection) + "\n";
  configFile << std::to_string(redSide?1:0) + "\n";
  configFile << std::to_string(debugging?1:0);
  configFile.close();
}

void readConfigs(){
  if(!fileExists(CONFIGFILE)){
    createFile(CONFIGFILE);
    writeConfigs();
  }else{
    int lineNum = 0, temp;
    std::string input;
    std::ifstream configFile(CONFIGFILE);
    while(getline(configFile,input))
    {
      lineNum++;
      switch(lineNum){
        case 1:
        autonSelection = std::stoi(input);
        break;
        case 2:
        temp = std::stoi(input);
        if(temp==1)redSide = true;
        else redSide = false;
        break;
        case 3:
        temp = std::stoi(input);
        if(temp==1)debugging = true;
        else debugging = false;
        break;
      }
    }
    configFile.close();
  }
}
