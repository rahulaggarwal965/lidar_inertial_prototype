#ifndef FEATURE_TYPES_H
#define FEATURE_TYPES_H

#include <Eigen/Core>

struct Plane {
    Eigen::Vector4f coefficients;

    Plane(float x, float y, float z, float d) 
    : coefficients(x, y, z, d) {}
};

#endif
