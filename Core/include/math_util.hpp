#ifndef MATH_UTIL_HPP
#define MATH_UTIL_HPP

struct Point {
    public:
        Point();
        Point(double x, double y);

        double x,y;
};

bool sgn(double x);

#endif