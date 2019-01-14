//
// Created by sid on 18-10-14.
//

#ifndef VICON_BIASESTIMATE_IMU_FACTOR_NEW_H
#define VICON_BIASESTIMATE_IMU_FACTOR_NEW_H
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactorNew {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUFactorNew() = delete;

    IMUFactorNew(IntegrationBase *_pre_integration, const Vector3d &vicon_pose_C, const Vector3d &vicon_velocity_C,const Quaterniond &vicon_quaternion_C,
              const Vector3d &vicon_pose_L, const Vector3d &vicon_velocity_L, const Quaterniond &vicon_quaternion_L) :
            pre_integration(_pre_integration), last_P(vicon_pose_L),last_V(vicon_velocity_L), last_Q(vicon_quaternion_L), Current_P(vicon_pose_C), Current_V(vicon_velocity_C), Current_Q(vicon_quaternion_C) {
    }

    bool operator()(const double ba[3],
                    const double bg[3],
                    const double Qew[4],
                    const double Tew[3],
                    const double Qvi[4], // 两个传感器体坐标系之间的 transform.
                    const double Tvi[3],
                    double residuals[9]) const {

        // vicon 提供的信息, P Q
        Vector3d Pl_wv(last_P);
        Vector3d Vl_wv(last_V);
        Quaterniond Ql_wv(last_Q);
        Vector3d Pc_wv(Current_P);
        Vector3d Vc_wv(Current_V);
        Quaterniond Qc_wv(Current_Q);

        // 系统参数
        Vector3d Bg(bg[0], bg[1], bg[2]);
        Vector3d Ba(ba[0], ba[1], ba[2]);

        Vector3d tew(Tew[0], Tew[1], Tew[2]);
        Quaterniond qew(Qew[0], Qew[1], Qew[2], Qew[3]);

        Vector3d tvi(Tvi[0], Tvi[1], Tvi[2]);
        Quaterniond qvi(Qvi[0], Qvi[1], Qvi[2], Qvi[3]);

        Vector3d Vl_ei_imu;
        Vector3d Pl_ei_imu;
        Vector3d Vc_ei_imu;
        Vector3d Pc_ei_imu;


        // ***************************************** integrate *************************************** //
        Vector3d delta_p(0, 0, 0);
        Vector3d delta_v(0, 0, 0);
        Quaterniond delta_q(1, 0, 0, 0);
        Vector3d un_acc, un_gyr;
        double _dt;

        //euler
        for (int i = 0; i < pre_integration->acc_buf.size(); i++) {
            _dt = pre_integration->dt_buf[i];
            un_acc = delta_q * (pre_integration->acc_buf[i] - Ba);
            un_gyr = pre_integration->gyr_buf[i] - Bg;
            delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt /2.0, un_gyr(1) * _dt / 2.0, un_gyr(2) * _dt / 2.0);
            delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
            delta_v = delta_v + un_acc * _dt;
            delta_q.normalize();
        }

        double sum_dt = pre_integration->sum_dt;

        /*
         * 上一时刻 的位置通过 imu 与当前时刻link
         * pk-1 ---- pk
         */
        // 上一时刻的 姿态直接由 vicon 给定
        Quaterniond Q_ei_vicon = qew*Ql_wv*qvi;
        Q_ei_vicon.normalize();


        Vector3d P_ei_est = Q_ei_vicon*delta_p + Pl_ei_imu + Vl_ei_imu*sum_dt - 0.5*g*sum_dt*sum_dt;
        residuals[0] = P_ei_est[0] - Pc_ei_imu[0];
        residuals[1] = P_ei_est[1] - Pc_ei_imu[1];
        residuals[2] = P_ei_est[2] - Pc_ei_imu[2];

        /*
         * 上一时刻的 速度 与当前速度也有 link
         */
        Vector3d V_ei_est = Q_ei_vicon*delta_v + Vl_ei_imu -g*sum_dt;
        residuals[3] = V_ei_est[0] - Vc_ei_imu[0];
        residuals[4] = V_ei_est[1] - Vc_ei_imu[1];
        residuals[5] = V_ei_est[2] - Vc_ei_imu[2];
        return true;
    }

    IntegrationBase *pre_integration;
    Vector3d Current_P,Current_V;
    Quaterniond Current_Q;
    Vector3d last_P, last_V;
    Quaterniond last_Q;
};


#endif //VICON_BIASESTIMATE_IMU_FACTOR_NEW_H
