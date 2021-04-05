#include "math_util.hpp"

Point::Point(double x, double y)
{
    this->x = x;
    this->y = y;
}

bool sgn(double x)
{
    if (x >= 0)
        return true;
    return false;
}