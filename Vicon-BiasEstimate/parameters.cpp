#include "parameters.h"
#include <iostream>

using namespace std;

double ACC_N = 0.2;
double ACC_W = 0.0002;
double GYR_N = 0.02;
double GYR_W = 2.0e-5;
Eigen::Vector3d g(0,0,9.8);

double BIAS_ACC_THRESHOLD = 0.6; // 官网数据  BMX055 0.68
double BIAS_GYR_THRESHOLD = 0.0175; // 官网传感器参数上下限 , 实际使用中温度变化很大,对温漂影响未知
//查询datasheet 得知， 加速度计温漂 灵敏度：+- 0.03%/K ， 零偏: +-1 mg/K (1度相差0.098)
// 陀螺仪 灵敏度：+- 0.03%/K ， 零偏: +-0.015 度/s/K (1度相差0.000262)
double SOLVER_TIME = 0.04;
int NUM_ITERATIONS = 5000;
int ESTIMATE_EXTRINSIC = 1;
int ViconInitNum=0;

