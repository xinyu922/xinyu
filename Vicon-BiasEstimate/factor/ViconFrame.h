#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <map>
#include <iostream>

using namespace Eigen;
using namespace std;

class ViconFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ViconFrame() {};

    ViconFrame(const Vector3d &_Position,const double &_t) : T{_Position},t{_t} {
    };
    double t;
    Matrix3d R;
    Vector3d T;
    IntegrationBase *pre_integration;
};

