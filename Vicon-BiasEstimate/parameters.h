#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <string>

using namespace std;

const int ImuInitNum = 0;
const int WINDOW_SIZE = 500;
const int stepSize = 10; // imu:200hz , image:20hz, vicon: 100hz

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d g;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;

enum SIZE_PARAMETERIZATION {
    SIZE_POSE = 7,
    SIZE_QUATERNION =4,
    SIZE_TRANSLATION =3,
    SIZE_SPEEDBIAS = 9,
    SIZE_BIAS = 6,
    SIZE_BA = 3,
    SIZE_BG = 3
};


