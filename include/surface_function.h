//
// Created by asctec on 9/20/16.
//

#ifndef PATH_PLANNER_SURFACE_FUNCTION_H
#define PATH_PLANNER_SURFACE_FUNCTION_H

#include <cmath>
#include <eigen3/Eigen/Dense>

class Surface_function {

public:

    virtual double computeFunctionValue(double x, double y, double z)=0;

    virtual Eigen::Vector3d computeGradientValue(double x, double y, double z)=0;
};


#endif //PATH_PLANNER_SURFACE_FUNCTION_H
