#pragma once
#include "util.hpp"
#include <vector>

void drive_line(MyPoint inTargetPoint, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance, double inaTolerance, double inaccelmilli, bool inStopAtEnd);
void turn_angle(double inTargetA, double inTurningfactor, double inaTolerance, double inturnAccelTimeMili, bool inStopAtEnd);
void follow_path(std::vector<MyPoint> &inPathToFollow, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance, double inaTolerance, double inaccelmilli, double inMaxLookAhead, double inMinLookAhead, double inlookAheadIncreaseDistance, bool inStopAtEnd);
extern double errD;
extern double errA;
