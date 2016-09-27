#include "plane.h"

Plane::Plane(double a, double b, double c, double d) {
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
}

double Plane::computeFunctionValue(double x, double y, double z) {
    return a*x + b*y + c*z + d;
}

Eigen::Vector3d Plane::computeGradientValue(double x, double y, double z) {
    Eigen::Array3d grad;
    grad << a,b,c;
    return grad;
}