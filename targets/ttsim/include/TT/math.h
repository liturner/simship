#pragma once
#include <complex>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tt::math {

    Eigen::Vector4d homogeneousPoint(const double x, const double y, const double z) {
        return {x, y, z, 1};
    }

    Eigen::Vector4d homogeneousVector(const double x, const double y, const double z) {
        return {x, y, z, 0};
    }
}
