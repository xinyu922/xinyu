#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUFactor() = delete;

    // 都需要转到vicon坐标系进行计算
    IMUFactor(IntegrationBase *_pre_integration, const Vector3d &vicon_pose_C, const Vector3d &vicon_velocity_C,
              const Quaterniond &vicon_quaternion_C,
              const Vector3d &vicon_pose_L, const Vector3d &vicon_velocity_L, const Quaterniond &vicon_quaternion_L) :
            pre_integration(_pre_integration), last_P(vicon_pose_L), last_V(vicon_velocity_L),
            last_Q(vicon_quaternion_L),
            Current_P(vicon_pose_C), Current_V(vicon_velocity_C), Current_Q(vicon_quaternion_C) {
    }

    bool operator()(const double ba[3],
                    const double bg[3],
                    const double Qew[4],
                    const double Qvi[4], // 两个传感器体坐标系之间的 transform.
                    const double Tvi[3],
                    const double Td[1],
                    double residuals[10]) const {

        double time_delay(Td[0]);

        Vector3d Pl_wv(last_P);
        Vector3d Vl_wv(last_V);
        Quaterniond Ql_wv(last_Q);
        Vector3d Pc_wv(Current_P);
        Vector3d Vc_wv(Current_V);
        Quaterniond Qc_wv(Current_Q);

        Vector3d Bg(bg[0], bg[1], bg[2]);
        Vector3d Ba(ba[0], ba[1], ba[2]);

        Quaterniond qew(Qew[0], Qew[1], Qew[2], Qew[3]);

        Vector3d tvi(Tvi[0], Tvi[1], Tvi[2]);
        Quaterniond qvi(Qvi[0], Qvi[1], Qvi[2], Qvi[3]);

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
        {
            un_acc = delta_q * (pre_integration->acc_buf.back() - Ba);
            un_gyr = pre_integration->gyr_buf.back() - Bg;
            delta_q = delta_q * Quaterniond(1, un_gyr(0) * time_delay /2.0, un_gyr(1) * time_delay / 2.0, un_gyr(2) * time_delay / 2.0);
            delta_p = delta_p + delta_v * time_delay + 0.5 * un_acc * time_delay * time_delay;
            delta_v = delta_v + un_acc * time_delay;
            delta_q.normalize();
        }
        /* mid-point
        Vector3d acc_0 = pre_integration->acc_buf[0];
        Vector3d acc_1 = pre_integration->acc_buf[0];
        Vector3d gyr_0 = pre_integration->gyr_buf[0];
        Vector3d gyr_1 = pre_integration->gyr_buf[0];
        for (int i = 0; i < pre_integration->acc_buf.size(); i++) {
            _dt = pre_integration->dt_buf[i];
            acc_1 = pre_integration->acc_buf[i];
            gyr_1 = pre_integration->gyr_buf[i];
            Vector3d un_acc_0 = delta_q * (acc_0 - Ba);
            Vector3d un_gyr = 0.5 * (gyr_0 + gyr_1) - Bg;
            delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
            Vector3d un_acc_1 = delta_q * (acc_1 - Ba);
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            delta_v = delta_v + un_acc * _dt;
            acc_0 = acc_1;
            gyr_0 = gyr_1;
            delta_q.normalize();
        }
        */


        double sum_dt = pre_integration->sum_dt;

        // Delta_Q
        qvi.normalize();
        Quaterniond delta_Q = qvi.inverse()*last_Q.normalized().inverse() * Current_Q*qvi;
        delta_Q.normalize();

        Quaterniond Q_residuals = delta_q.inverse() * delta_Q;
        residuals[0] = Q_residuals.w() -1 ;
        residuals[1] = Q_residuals.x();
        residuals[2] = Q_residuals.y();
        residuals[3] = Q_residuals.z();

        //  Delta_V
        Vector3d Vl_wi, Vl_ei;
        Vl_wi = Vl_wv;
        Vl_ei = qew * Vl_wi;

        Vector3d Vc_wi, Vc_ei;
        Vc_wi = Vc_wv;
        Vc_ei = qew * Vc_wi;

        Vector3d delta_V = (qew * Ql_wv * qvi).normalized().inverse() * (Vc_ei - Vl_ei + g * sum_dt);

        residuals[4] = delta_V.x() - delta_v.x();
        residuals[5] = delta_V.y() - delta_v.y();
        residuals[6] = delta_V.z() - delta_v.z();
//        residuals[4] =0;
//        residuals[5] =0;
//        residuals[6] =0;

        //  Delta_P
        Vector3d Pl_ei = qew*Ql_wv*tvi + qew*Pl_wv;
        Vector3d Pc_ei = qew*Qc_wv*tvi + qew*Pc_wv;

        Vector3d delta_P = (qew * Ql_wv * qvi).normalized().inverse() * ( Pc_ei-Pl_ei -Vl_ei*sum_dt + 0.5* g *sum_dt*sum_dt);
        residuals[7] = delta_P.x() - delta_p.x();
        residuals[8] = delta_P.y() - delta_p.y();
        residuals[9] = delta_P.z() - delta_p.z();
//        residuals[7] =0;
//        residuals[8] =0;
//        residuals[9] =0;

        return true;
    }

    IntegrationBase *pre_integration;
    Vector3d Current_P;
    Vector3d Current_V;
    Quaterniond Current_Q;
    Vector3d last_P;
    Vector3d last_V;
    Quaterniond last_Q;
};

