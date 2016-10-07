//
// Created by asctec on 9/20/16.
//

#ifndef PATH_PLANNER_PLANE_H
#define PATH_PLANNER_PLANE_H

#include "surface_function.h"

class Plane: public Surface_function{
    public:
        Plane(double a, double b, double c, double d,double steepness);
        double computeFunctionValue(double x,double y,double z);
        Eigen::Vector3d computeGradientValue(double x, double y, double z);
    private:
        double a;
        double b;
        double c;
        double d;
        double steepness;
};


#endif //PATH_PLANNER_PLANE_H
