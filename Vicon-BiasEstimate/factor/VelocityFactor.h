#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class VelocityFactor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VelocityFactor() = delete;

    // 都需要转到vicon坐标系进行计算
    VelocityFactor(IntegrationBase *_pre_integration, const Vector3d &vicon_velocity_C,
              const Quaterniond &vicon_quaternion_C,
                   const Vector3d &vicon_velocity_L, const Quaterniond &vicon_quaternion_L) :
            pre_integration(_pre_integration),
            last_V(vicon_velocity_L), last_Q(vicon_quaternion_L),
            Current_V(vicon_velocity_C), Current_Q(vicon_quaternion_C) {
    }

    bool operator()(const double ba[3],
                    const double bg[3],// ba:[0-2], bg:[3-5]
                    const double Tew[7], // t: [0-2] , q: [3-6] , e:imu世界系, w: vicon世界系
                    const double Tvi[7], // 两个传感器体坐标系之间的 transform.
                    double residuals[3]) const {

        Vector3d Vl_wv(last_V);
        Quaterniond Ql_wv(last_Q);
        Vector3d Vc_wv(Current_V);
        Quaterniond Qc_wv(Current_Q);


        Vector3d Ba(ba[0], ba[1], ba[2]);
        Vector3d Bg(bg[0], bg[1], bg[2]);

//        Vector3d tew(Tew[0], Tew[1], Tew[2]);
        Vector3d tew(0, 0, 0);
        Quaterniond qew(Tew[3], Tew[4], Tew[5], Tew[6]);

        Vector3d tvi(Tvi[0], Tvi[1], Tvi[2]);
        Quaterniond qvi(Tvi[3], Tvi[4], Tvi[5], Tvi[6]);

        Vector3d delta_v(0, 0, 0);
        Quaterniond delta_q(1, 0, 0, 0);
        double _dt;

        Vector3d un_acc, un_gyr;
        //euler 积分
//        for (int i = 0; i < pre_integration->acc_buf.size(); i++) {
//            _dt = pre_integration->dt_buf[i];
//            un_acc = delta_q * (pre_integration->acc_buf[i] - Ba);
//            un_gyr = pre_integration->gyr_buf[i] - Bg;
//            delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2.0, un_gyr(1) * _dt / 2.0, un_gyr(2) * _dt / 2.0);
//            delta_v = delta_v + un_acc * _dt;
//            delta_q.normalize();
//        }
        // mid-point
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

        // try to combined the velocity and rotation
//        Quaterniond delta_Q = qvi.inverse() * last_Q.normalized().inverse() * Current_Q * qvi;
//        delta_Q.normalize();
//        Quaterniond Q_residuals = delta_q.inverse() * delta_Q;
//        Q_residuals.normalize();
//        residuals[3] = Q_residuals.w() - 1;
//        residuals[4] = Q_residuals.x();
//        residuals[5] = Q_residuals.y();
//        residuals[6] = Q_residuals.z();

        Vector3d Vl_wi, Vl_ei;
//        Quaterniond Ql_wi, Ql_ei;
//        Vector3d Pl_wi, Pc_wi, Pl_wi_, Pc_wi_;
//        Ql_wi = Ql_wv.normalized() * qvi;

        Vl_wi = Vl_wv;
//        Vl_wi = -(qvi.inverse() * (-Vl_wv));
//        Vl_wi = Vl_wv+ last_Omiga.cross(-tvi);
//        Ql_ei = qew * Ql_wi.normalized();
        Vl_ei = qew * Vl_wi;


        Vector3d Vc_wi, Vc_ei;
//        Quaterniond Qc_wi, Qc_ei;
//        Qc_wi = Qc_wv.normalized() * qvi;

//        Vc_wi = Vc_wv+ Current_Omiga.cross(-tvi);
//        Vc_wi = -(qvi.inverse() * (-Vc_wv));
        Vc_wi = Vc_wv;

//        qew.normalize();
//        Qc_ei = qew * Qc_wi.normalized();
        Vc_ei = qew * Vc_wi;

        Vector3d delta_V = (qew * Ql_wv * qvi).inverse().normalized() * (Vc_ei - Vl_ei + g * pre_integration->sum_dt);
//        Vector3d delta_V = (qew * Ql_wv * qvi).inverse().normalized() * (Vc_wi - Vl_wi + g * pre_integration->sum_dt);

//        cout << "V_ei " << Vc_ei - Vl_ei << endl;
//        cout << "V_wi" << Vc_wi - Vl_wi << endl;

//        cout<< "delta_v "<<delta_v<<endl;
//        cout<< "delta_V"<<delta_V << endl;

        residuals[0] = delta_V.x() - delta_v.x();
        residuals[1] = delta_V.y() - delta_v.y();
        residuals[2] = delta_V.z() - delta_v.z();
//        cout<< residuals[0] <<residuals[1]<<residuals[2]<<endl;
        return true;
    }

    IntegrationBase *pre_integration;
    Vector3d Current_V;
    Quaterniond Current_Q;
    Vector3d last_V;
    Quaterniond last_Q;
};

