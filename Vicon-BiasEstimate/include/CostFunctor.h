#ifndef IMUERROR_H_
#define IMUERROR_H_

#include "ceres/rotation.h"
#include <ceres/ceres.h>
#include "rotation.h"

#define compareLength 1000

// 一个全局的位置\速度变量, 旋转则使用AHRS
double positon[3] = {0};
double velocity[3] = {0};
extern double deltaT;

// Each Residual block takes a imu measurement and a imu pose as input and
// outputs a 3 dimensional residual (translation)
struct IMUError {
    IMUError(vector<IMUData> imuDatas,
             vector<ViconData> viconDatas)
            : _imuDatas(imuDatas), _viconDatas(viconDatas) {}

    template<typename T>
    bool operator()(const T *const q,
                    const T *const t,
                    const T *const ba,
                    const T *const bg,
                    T *residuals) const {

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        const T xp = 1;
        const T yp = 1;

        // Apply second and fourth order radial distortion.
        const T &l1 = 1;
        const T &l2 = 1;
        const T r2 = xp * xp + yp * yp;
        const T distortion = 1.0 + r2 * (l1 + l2 * r2);


        // Compute final projected point position.
        const T &focal = 0;
        const T predicted_x = focal * distortion * xp;
        const T predicted_y = focal * distortion * yp;
        const T predicted_z = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        for (int i = 0; i <compareLength ; i++) {
            residuals[3*i+0] = predicted_x - _viconDatas.at(i)._t.x();
            residuals[3*i+1] = predicted_y - _viconDatas.at(i)._t.y();
            residuals[3*i+2] = predicted_z - _viconDatas.at(i)._t.z();
        }


        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const IMUData imuData,
                                       const ViconData viconData) {
        // 不知道 residule的维度是否可以遮阳定
        return (new ceres::AutoDiffCostFunction<IMUError, 3 * compareLength, 4, 3, 3, 3>(
                new IMUError(imuData, viconData)));
    }

    vector<IMUData> _imuDatas;
    vector<ViconData> _viconDatas;
    vector<Eigen::Vector3d> _positons;
};

#endif