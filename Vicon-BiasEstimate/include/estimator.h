#pragma once

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

#include <assert.h>
#include <ceres/ceres.h>
#include "../factor/imu_factor.h"
#include "../factor/rotation_factor.h"
#include "../factor/VelocityFactor.h"
#include "../factor/pose_local_parameterization.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include "../parameters.h"
#include "../factor/ViconFrame.h"
#include "../utility/AHRS.h"

using namespace Eigen;

class Estimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Estimator();

    void clearState();

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    void processVicon(const Vector3d &position, const double &timestamp, const Quaterniond &quaternion, const Vector3d &velocity);

    // internal
    void optimizationRotation();
    void optimizationTranslation();

    Vector3d g;

    // transform between imu and vicon
    Matrix3d rvi;
    Quaterniond qvi;
    Vector3d tvi;
    Matrix3d rew;
    Quaterniond qew;
    Vector3d tew;

    // 优化的初值
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Quaterniond Qs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];

    Vector3d Pvs[(WINDOW_SIZE +1)];
    Quaterniond Qvs[(WINDOW_SIZE+1)];
    Vector3d Vvs[(WINDOW_SIZE +1)];

    double Timestamps[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    bool first_imu;
    int frame_count;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_Vs[WINDOW_SIZE +1][3];
    double para_Ps[WINDOW_SIZE +1][3];
    double para_Td[1] = {0};
    double para_g[3] ={0,0,9.81};
    double para_Ba[SIZE_BA] = {0,0,0};
    double para_Bg[SIZE_BG] = {0,0,0};
    double para_Q_E_W[SIZE_QUATERNION] = {1,0,0,0}; // qew 为单位四元数
//    double para_T_E_W[SIZE_TRANSLATION] = {0,0,0};  // tew 为优化时的初始 位置
//    double para_Q_G[SIZE_QUATERNION]; // 惯性系与v0系之间的误差，
    double para_Q_V_I[SIZE_QUATERNION] = {0, 0.707 ,0, 0.707};
    double para_T_V_I[SIZE_TRANSLATION] = {0.0935521,-0.0289465,-0.106346};
//    double para_Ex_V_I[SIZE_POSE] = {0, 0, 0, 0, 0.7071, 0.7071, 0}; // for tango 0506

    map<double, ViconFrame> all_vicon_frame;
    IntegrationBase *tmp_pre_integration;

};
