#ifndef LEICADATA_H_
#define LEICADATA_H_

#include "common.h"


using namespace Eigen;

class LeicaData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LeicaData(const double &px, const double &py, const double &pz, const double &t) : _p(px, py, pz), _time(t) {}

    Vector3d _p;    //translation
    double _time;      //timestamp: ns
};


#endif // LEICADATA_H_
