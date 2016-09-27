//
// Created by asctec on 9/20/16.
//

#ifndef PATH_PLANNER_CYLINDER_H
#define PATH_PLANNER_CYLINDER_H

#include "surface_function.h"

class Cylinder: public Surface_function{
public:
    Cylinder(double a, double b, double x0, double y0);
    double computeFunctionValue(double x, double y, double z);
    Eigen::Vector3d computeGradientValue(double x, double y, double z);
private:
    double a;
    double b;
    double x0;
    double y0;
};

#endif //PATH_PLANNER_CYLINDER_H