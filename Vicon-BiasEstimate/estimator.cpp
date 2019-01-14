#include <estimator.h>
#include "factor/imu_factor.h"
#include "factor/imu_factor_new.h"
#include "factor/quat_local_parameterization.h"


Estimator::Estimator() {
    clearState();
}

void Estimator::clearState() {
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();
//        Ps[i].setZero();
//        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr) {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    tvi = Vector3d::Zero();
    rvi = Matrix3d::Identity();
    tew = Vector3d::Zero();
    rew = Matrix3d::Identity();

    first_imu = false;
    frame_count = 0;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;

    tmp_pre_integration = nullptr;
}


/**
 * 处理加速度数据，调用imu的预积分
 * 1.每一帧会创建一个 IntegrationBase
 * 2.调用push_back()方法将imu数据存入，同时其内调用了propagation()方法，计算出对应的状态量，协方差和雅可比
 */
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0) {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    //std::cout << "acc_0 : " << acc_0 << std::endl;
    //std::cout << "gyr_0 : " << gyr_0 << std::endl;
}

void Estimator::processVicon(const Vector3d &position, const double &timestamp, const Quaterniond &quaternion, const Vector3d &velocity) {
    Timestamps[frame_count] = timestamp;

    ViconFrame viconframe(position, timestamp);
    viconframe.pre_integration = tmp_pre_integration;
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    all_vicon_frame.insert(make_pair(timestamp, viconframe));
    Pvs[frame_count] = position;
    Qvs[frame_count] = quaternion;
    Vvs[frame_count] = velocity;

    //把所有的帧都加进去，然后做一个联合优化
    if (frame_count == WINDOW_SIZE) {
        optimizationRotation();
        optimizationTranslation();

        cout << "Ba:" << para_Ba[0]
             << " " << para_Ba[1]
             << " " << para_Ba[2] << endl;

        cout << "Bg:" << para_Bg[0]
             << " " << para_Bg[1]
             << " " << para_Bg[2] << endl;
    } else
        frame_count++;
}

void Estimator::optimizationRotation() {
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    TicToc t_whole, t_prepare;

    {
        ceres::LocalParameterization *localParameterization = new QuatLocalParameterization();
        problem.AddParameterBlock(para_Bg, 3);
        problem.AddParameterBlock(para_Q_V_I, 4, localParameterization);
    }
    for (int i = 0; i < WINDOW_SIZE-1; i++) {
        int j = i + 1;
        if (!pre_integrations[j])
            continue;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        ceres::CostFunction *costFunction =
                new ceres::NumericDiffCostFunction<RotationFactor, ceres::FORWARD, 4, 3, 4>(
                        new RotationFactor(pre_integrations[j], Qvs[j], Qvs[i])
                );
        problem.AddResidualBlock(costFunction, loss_function, para_Bg, para_Q_V_I);
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = NUM_ITERATIONS;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;

    cout << "Bg:" << para_Bg[0]
         << " " << para_Bg[1]
         << " " << para_Bg[2] << endl;
    cout << "Qvi:" << Quaterniond(para_Q_V_I[0], para_Q_V_I[1], para_Q_V_I[2], para_Q_V_I[3]).normalized().coeffs()
         << endl;
    cout << "Rvi:" << Quaterniond(para_Q_V_I[0], para_Q_V_I[1], para_Q_V_I[2], para_Q_V_I[3]).normalized().toRotationMatrix().transpose()<<endl;
    cout<< "tvi" << -Quaterniond(para_Q_V_I[0], para_Q_V_I[1], para_Q_V_I[2], para_Q_V_I[3]).normalized().toRotationMatrix().transpose()*Vector3d(0.06901,-0.02781,-0.12395);
}

/*
void Estimator::optimizationVelocity() {
    //由rotation和madgwick提供一个qew的先验
    double q[4] = {q0, q1, q2, q3}; //w,x,y,z
    Quaterniond quat_ei = Quaterniond(q0, q1, q2, q3).normalized(); // AHRS
    Quaterniond quat_vi = Quaterniond(para_Ex_V_I[3], para_Ex_V_I[4], para_Ex_V_I[5],
                                      para_Ex_V_I[6]).normalized(); //roation
    Quaterniond quat_wv = vicon_quaternion_buf[0].normalized(); //vicon
    Quaterniond quat_ew = quat_ei * quat_vi.inverse() * quat_wv.inverse();
    // 对euroc ，已知先验为 pi,0,0
    para_Ex_E_W[3] = 0;
    para_Ex_E_W[4] = 0;
    para_Ex_E_W[5] = 0;
    para_Ex_E_W[6] = 1;
    cout << "init euler_ew: " << quat_ew.toRotationMatrix().eulerAngles(2, 1, 0) * 180 / 3.1415926 << endl;

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    TicToc t_whole, t_prepare;

    for (int i = 0; i < WINDOW_SIZE -1 ; i++) {
        int j = i + 1;
        if (!pre_integrations[j])
            continue;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        ceres::CostFunction *costFunction =
                new ceres::NumericDiffCostFunction<VelocityFactor, ceres::CENTRAL, 3, 3, 3, 7, 7>(
                        new VelocityFactor(pre_integrations[j], vicon_velocity_buf[j], vicon_quaternion_buf[j],
                                           vicon_velocity_buf[i], vicon_quaternion_buf[i])
                );
        problem.AddResidualBlock(costFunction, loss_function, para_Ba, para_Bg, para_Ex_E_W, para_Ex_V_I);
    }

    problem.SetParameterBlockConstant(para_Bg);
    problem.SetParameterBlockConstant(para_Ex_V_I);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = NUM_ITERATIONS;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;
//    cout << summary.FullReport() <<endl;
//    cout << "solver costs: " << t_solver.toc() << endl;
//    cout << "whole time for ceres: %f" << t_whole.toc() << endl;

    cout << "Ba:" << para_Ba[0]
         << " " << para_Ba[1]
         << " " << para_Ba[2] << endl;

    cout << "Bg:" << para_Bg[0]
         << " " << para_Bg[1]
         << " " << para_Bg[2] << endl;

    cout << "Qew:" << Quaterniond(para_Ex_E_W[3], para_Ex_E_W[4], para_Ex_E_W[5], para_Ex_E_W[6]).normalized().coeffs()
         << endl;
    cout << "Euler_ew: " << Quaterniond(para_Ex_E_W[3], para_Ex_E_W[4], para_Ex_E_W[5],
                                        para_Ex_E_W[6]).normalized().toRotationMatrix().eulerAngles(2, 1, 0) * 180 /
                            3.1415926 << endl;

    cout << "tew:" << para_Ex_E_W[0]
         << " " << para_Ex_E_W[1]
         << " " << para_Ex_E_W[2] << endl;
    cout << "Qvi:" << Quaterniond(para_Ex_V_I[3], para_Ex_V_I[4], para_Ex_V_I[5], para_Ex_V_I[6]).normalized().coeffs()
         << endl;
    cout << "Euler_vi: " << Quaterniond(para_Ex_V_I[3], para_Ex_V_I[4], para_Ex_V_I[5],
                                        para_Ex_V_I[6]).normalized().toRotationMatrix().eulerAngles(2, 1, 0) * 180 /
                            3.1415926 << endl;

    cout << "tvi:" << para_Ex_V_I[0]
         << " " << para_Ex_V_I[1]
         << " " << para_Ex_V_I[2] << endl;
}
 */

void Estimator::optimizationTranslation() {
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    TicToc t_whole, t_prepare;

//
//    Quaterniond qei(q0,q1,q2,q3);
//    Quaterniond qvi(para_Q_V_I[0],para_Q_V_I[1],para_Q_V_I[2],para_Q_V_I[3]);
//
//
//    qew = qei*qvi.inverse()*Qvs[ViconInitNum].inverse();
//    qew.normalize();
//    para_Q_E_W[0] = qew.w();
//    para_Q_E_W[1] = qew.x();
//    para_Q_E_W[2] = qew.y();
//    para_Q_E_W[3] = qew.z();
//
//    tew =(qei*qvi.inverse()).normalized()*Vector3d(-Pvs[ViconInitNum].x(), -Pvs[ViconInitNum].y(), -Pvs[ViconInitNum].z());
//    para_T_E_W[0] = tew.x();
//    para_T_E_W[1] = tew.y();
//    para_T_E_W[2] = tew.z();
//
//    para_T_E_W[0] = -Pvs[ViconInitNum].x();
//    para_T_E_W[1] = -Pvs[ViconInitNum].y();
//    para_T_E_W[2] = -Pvs[ViconInitNum].z();

    {
        ceres::LocalParameterization *localParameterization = new QuatLocalParameterization();
        problem.AddParameterBlock(para_Ba, 3);
        problem.AddParameterBlock(para_Bg, 3);
        problem.AddParameterBlock(para_Q_E_W, 4,localParameterization);
//        problem.AddParameterBlock(para_T_E_W, 3);
        problem.AddParameterBlock(para_Q_V_I, 4, localParameterization);
        problem.AddParameterBlock(para_T_V_I, 3);
        problem.AddParameterBlock(para_Td,1);
    }


    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        int j = i + 1;
        if (!pre_integrations[j])
            continue;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        ceres::CostFunction *costFunction =
                new ceres::NumericDiffCostFunction<IMUFactor, ceres::FORWARD, 10, 3, 3, 4, 4, 3, 1 >(
                        new IMUFactor(pre_integrations[j], Pvs[j],Vvs[j], Qvs[j],
                                      Pvs[i],Vvs[i], Qvs[i])
                );
        problem.AddResidualBlock(costFunction, loss_function, para_Ba, para_Bg, para_Q_E_W, para_Q_V_I, para_T_V_I, para_Td);
    }

    problem.SetParameterBlockConstant(para_Bg);
    problem.SetParameterBlockConstant(para_Q_V_I);
    problem.SetParameterBlockConstant(para_Td);
    problem.SetParameterBlockConstant(para_T_V_I);
//    problem.SetParameterBlockConstant(para_Q_E_W);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = NUM_ITERATIONS;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;

    cout << "Ba:" << para_Ba[0]
         << " " << para_Ba[1]
         << " " << para_Ba[2] << endl;

    cout << "Bg:" << para_Bg[0]
         << " " << para_Bg[1]
         << " " << para_Bg[2] << endl;

    cout << "Qew:" << Quaterniond(para_Q_E_W[0], para_Q_E_W[1], para_Q_E_W[2], para_Q_E_W[3]).normalized().coeffs()
         << endl;
//    cout << "tew:" << para_T_E_W[0]
//         << " " << para_T_E_W[1]
//         << " " << para_T_E_W[2] << endl;

    cout << "Qvi:" << Quaterniond(para_Q_V_I[0], para_Q_V_I[1], para_Q_V_I[2], para_Q_V_I[3]).normalized().coeffs()
         << endl;

    cout << "tvi:" << para_T_V_I[0]
         << " " << para_T_V_I[1]
         << " " << para_T_V_I[2] << endl;

    cout << "time delay:" << para_Td[0] <<endl;
}

