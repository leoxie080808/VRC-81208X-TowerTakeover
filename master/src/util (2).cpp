#include "main.h"
#include "util.hpp"
#include <cmath>
#include <string>
#include <cstring>


//https://www.desmos.com/calculator/gqb45ium43
TargetPoint MyLine::getClosestPoint(MyPoint cur)
{
  double m, x, y;
  if (pointB.x != pointA.x)
  {
    m = (pointB.y - pointA.y) / (pointB.x - pointA.x);                               //m for slope of the line
    x = (pow(m, 2) * pointA.x - m * pointA.y + cur.x + m * cur.y) / (pow(m, 2) + 1); //finds the x of where the lines meet
    x = clampAdvDouble(x, pointA.x, pointB.x);                                       //clamp it, line segment
    y = m * (x - pointA.x) + pointA.y;                                               //find the y
  }
  else
  {
    x = pointA.x;
    y = clampAdvDouble(cur.y, pointA.y, pointB.y);
  }
  return {x, y, distanceBetween(cur, {x, y}), true};
}

//cur must be on this line
TargetPoint MyLine::getLookAheadPoint(MyPoint cur, double lookAheadDistance)
{
  double x, y, m, clamped;
  bool online = true;
  if (pointB.x != pointA.x)
  {
    //https://www.desmos.com/calculator/gqb45ium43
    m = (pointB.y - pointA.y) / (pointB.x - pointA.x); //m for slope of the line
    x = cur.x + signDouble(pointB.x - pointA.x) * fabs(sqrt(pow(lookAheadDistance, 2) / (1 + pow(m, 2))));
    clamped = clampAdvDouble(x, pointA.x, pointB.x);
    if (fabs(x - clamped) > 0.05)
      online = false;
    x = clamped;
    y = m * (x - pointA.x) + pointA.y;
  }
  else
  {
    x = pointA.x;
    y = cur.y + lookAheadDistance * signDouble(pointB.y - pointA.y);
    clamped = clampAdvDouble(y, pointA.y, pointB.y);
    if (fabs(y - clamped) > 0.05)
      online = false;
    y = clamped;
  }
  return {x, y, distanceBetween(cur, {x, y}), online};
}

//MATH STUFFS
double avgDouble(double a, double b)
{
  return (a + b) / 2;
}
double signDouble(double input)
{
  if (std::signbit(input))
    return -1;
  else
    return 1;
}
double clampDouble(double input, double limit)
{
  return fabs(input) > limit ? limit * signDouble(input) : input;
}

//outputs input clamped between A and B
double clampAdvDouble(double input, double limitA, double limitB)
{
  double bigger = limitA >= limitB ? limitA : limitB;
  double smaller = limitA <= limitB ? limitA : limitB;
  if (input < smaller)
  {
    return smaller;
  }
  else if (input > bigger)
  {
    return bigger;
  }
  else
    return input;
}

double wrapAngle(double angle)
{
  while (fabs(angle) > M_PI)
  {
    if (angle >= M_PI)
    {
      angle -= 2 * M_PI;
    }
    else if (angle <= -M_PI)
    {
      angle += 2 * M_PI;
    }
  }
  return angle;
}

double distanceBetween(MyPoint pointA, MyPoint pointB)
{
  return sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2));
}
