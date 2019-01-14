#ifndef VICONDATA_H_
#define VICONDATA_H_

#include "common.h"


using namespace Eigen;

class ViconData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ViconData(const double &qw, const double &qx, const double &qy, const double &qz,
            const double &tx, const double &ty, const double &tz,
            const double &t) : _q(qw, qx, qy, qz), _t(tx, ty, tz), _time(t) {}

    Quaterniond _q;    //quaternion
    Vector3d _t;    //translation
    double _time;      //timestamp
};


#endif // VICONDATA_H
