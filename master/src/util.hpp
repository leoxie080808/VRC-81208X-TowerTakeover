#pragma once
#include <cmath>

struct MyPoint{
  double x;
  double y;
};

double avgDouble(double a, double b);
double signDouble(double input);
// void printToController(const char message[]);
double clampDouble(double input, double limit);
double clampAdvDouble(double input, double limitA, double limitB);
double wrapAngle(double angle);
double distanceBetween(MyPoint pointA, MyPoint pointB);

struct TargetPoint{
  double x;
  double y;
  double d;
  bool online;
};

struct MyLine{
  MyPoint pointA;
  MyPoint pointB;
  //https://www.desmos.com/calculator/gqb45ium43
  TargetPoint getClosestPoint(MyPoint cur);
  //cur must be on this line
  TargetPoint getLookAheadPoint(MyPoint cur, double lookAheadDistance);
};
