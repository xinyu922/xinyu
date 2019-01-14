#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class RotationFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RotationFactor() = delete;

    // 对比两帧间的相对旋转
    RotationFactor(IntegrationBase *_pre_integration, const Quaterniond &vicon_quaternion_C,
                   const Quaterniond &vicon_quaternion_L) :
            pre_integration(_pre_integration), last_Q(vicon_quaternion_L), Current_Q(vicon_quaternion_C) {
    }

    bool operator()(const double bg[3],
                    const double Qvi[4],
                    double residuals[4]) const {

        Vector3d Bg(bg[0], bg[1], bg[2]);
        Quaterniond qvi(Qvi[0], Qvi[1], Qvi[2], Qvi[3]);

        Quaterniond delta_q(1, 0, 0, 0);
        Vector3d un_gyr;
        double _dt;

        Vector3d gyr_0 = pre_integration->gyr_buf[0];
        Vector3d gyr_1 = pre_integration->gyr_buf[0];
        for (int i = 0; i < pre_integration->acc_buf.size(); i++) {
            _dt = pre_integration->dt_buf[i];
            gyr_1 = pre_integration->gyr_buf[i];
            Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1) - Bg;
            delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
            gyr_0 = gyr_1;
            delta_q.normalize();
        }
//        for (int i = 0; i < pre_integration->gyr_buf.size(); i++) {
//            _dt = pre_integration->dt_buf[i];
//            un_gyr = pre_integration->gyr_buf[i] - Bg;
//            delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2.0, un_gyr(1) * _dt / 2.0, un_gyr(2) * _dt / 2.0);
//            delta_q.normalize();
//        }
        qvi.normalize();
        Quaterniond delta_Q = qvi.inverse()*last_Q.normalized().inverse() * Current_Q*qvi;
        delta_Q.normalize();

        Quaterniond Q_residuals = delta_q.inverse() * delta_Q;
//        Q_residuals.normalize();
        residuals[0] = Q_residuals.w() -1 ;
        residuals[1] = Q_residuals.x();
        residuals[2] = Q_residuals.y();
        residuals[3] = Q_residuals.z();
        return true;
    }

    IntegrationBase *pre_integration;
    Quaterniond Current_Q;
    Quaterniond last_Q;
};

