#include "util.hpp"
#include <cmath>
//MATH STUFFS
double avgDouble(double a, double b)
{
	return (a + b) / 2;
}
int signDouble(double input)
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
double modulo(double a, double b) {
  while (a>b) {
    a-=b;
  }
  return a;
}
double minimizeAngle(double angle){
	while(fabs(angle)>M_PI){
		if (angle >= M_PI) {
				angle-=2*M_PI;
			}
			else if (angle <= -M_PI) {
				angle+=2*M_PI;
			}
	}
	return angle;
}
