//
// Created by sid on 18-10-31.
//

#ifndef VICON_BIASESTIMATE_QUAT_LOCAL_PARAMETERIZATION_H
#define VICON_BIASESTIMATE_QUAT_LOCAL_PARAMETERIZATION_H
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class QuatLocalParameterization : public ceres::LocalParameterization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

    virtual bool ComputeJacobian(const double *x, double *jacobian) const;

    virtual int GlobalSize() const { return 4; };

    virtual int LocalSize() const { return 3; };
};
#endif //VICON_BIASESTIMATE_QUAT_LOCAL_PARAMETERIZATION_H
