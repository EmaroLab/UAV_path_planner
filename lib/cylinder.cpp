#include <Eigen/Dense>
#include "cylinder.h"

Cylinder::Cylinder(double a, double b, double x0, double y0) {
    this->a = a;
    this->b = b;
    this->x0 = x0;
    this->y0 = y0;
}

double Cylinder::computeFunctionValue(double x, double y, double z) {
    return pow((x-x0)/a,2)+pow((y-y0)/b,2)-1;
}

Eigen::Vector3d Cylinder::computeGradientValue(double x, double y, double z) {
    Eigen::Array3d grad ;
    grad << (2/a)*((x-x0)/a),(2/b)*((y-y0)/b),0;
    return grad;
}