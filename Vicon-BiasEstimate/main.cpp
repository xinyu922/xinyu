#include <iostream>
#include <fstream>
#include <sstream>

#include "include/common.h"
#include "include/imudata.h"
#include "include/vicondata.h"

#include "utility/AHRS.h"
#include "factor/pose_local_parameterization.h"
#include "utility/tic_toc.h"
#include "estimator.h"
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void LoadImus(const string &strImuPath, vector<IMUData> &vImus);

void LoadVicons(const string &strViconPath, vector<ViconData> &vVicons);

void LoadUtcTimeOffset(const string &strTimeoffsetPath, double &utcTimeOffset);

Estimator estimator;


double utcTimeOffset;
double current_time = -1;

int main(int argc, char **argv) {

    if (argc != 4 && argc!=3 ) {
        cerr << endl
             << "Usage: ./Vicon_BiasEstimate path_to_vicon_file path_to_imu path_to_utcTimeOffset"
             << endl;
        return 1;
    }

    /*
    Matrix3d riv;
    riv<<0.33638, -0.01749,  0.94156,
            -0.02078, -0.99972, -0.01114,
            0.94150, -0.01582, -0.33665;
    Vector3d tiv(0.06901,-0.02781,-0.12395);
    Quaterniond qrv(0.99318990133362184,-0.010589344034740935,0.022261260519047747,0.11386888056851825); // vicon obj to ref frame
    Vector3d g(0,0,9.8);
    Vector3d mean_static(9.059418196,0.116280019,-3.682405261);
    cout<< "norm of mean"<< mean_static.norm()<<endl;
    cout<<"sample estimate ba: "<< mean_static - riv*(qrv.normalized().inverse()*g)<<endl;
*/

//#define visualize_roation
#ifdef visualize_roation
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget",viz::WCoordinateSystem());
    viz::WCameraPosition IMU_widget;
    myWindow.showWidget("IMU Widget", IMU_widget);
    viz::WCameraPosition Vicon_widget;
    myWindow.showWidget("Vicon Widget",Vicon_widget);
    cv::Mat imu_rot = Mat::zeros(1,3,CV_32F);
    cv::Mat vicon_rot = Mat::zeros(1,3,CV_32F);
#endif

    //---------------------------------数据读取-----------------------------
    vector<IMUData> vImus;
    vector<ViconData> vVicons;
    LoadVicons(argv[1], vVicons);
    LoadImus(argv[2], vImus);
    if (argc !=4 ) utcTimeOffset = 0;
    else LoadUtcTimeOffset(argv[3], utcTimeOffset);
    cout << "Imu vector size:" << vImus.size() << endl;
    cout << "Imu time interval:" << vImus.at(100)._t - vImus.at(99)._t << endl;
    cout << "Vicon vector size:" << vVicons.size() << endl;
    cout << "Vicon time interval:" << vVicons.at(100)._time - vVicons.at(99)._time << endl;

    // -----------------------使用AHRS对齐惯性坐标系------------------
    for(int i=0; i< ImuInitNum; i++){
        AHRSupdateIMU(vImus.at(i)._g.x(), vImus.at(i)._g.y(), vImus.at(i)._g.z(),
                      vImus.at(i)._a.x(), vImus.at(i)._a.y(), vImus.at(i)._a.z());
    }
    Quaterniond init_Qei(q0, q1, q2, q3);

    //---------------------------------主循环----------------------
    // 找到时间同步点
    int ViconInitNum = 0;
    for (int i = 0; i < vVicons.size(); i++) {
        if (vVicons.at(i)._time - utcTimeOffset >= vImus.at(ImuInitNum)._t &&
            vVicons.at(i)._time - utcTimeOffset < vImus.at(ImuInitNum + 2)._t) {
            ViconInitNum = i;
            break;
        }
    }
    cout<<" Vicon init number:"<<ViconInitNum<<endl;

    Quaterniond Qv0 = vVicons[ViconInitNum]._q;
    Vector3d Tv0 = vVicons[ViconInitNum]._t;
    for(int i=ViconInitNum; i<vVicons.size();i++){
        vVicons.at(i)._q = Qv0.inverse()*vVicons.at(i)._q;
        vVicons.at(i)._t = Qv0.inverse()*(vVicons.at(i)._t - Tv0);
    }

    int j = ImuInitNum;
    for (int i = ViconInitNum + 1; i < ViconInitNum + WINDOW_SIZE * stepSize + stepSize; i = i + stepSize) {
        while (vImus.at(j)._t < vVicons.at(i)._time - utcTimeOffset) {
            j++;
            // process the IMU data
            double t = vImus.at(j)._t;
            if (current_time < 0)
                current_time = t;
            double dt = t - current_time;
            current_time = t;
            estimator.processIMU(dt, vImus.at(j)._a, vImus.at(j)._g);

#ifdef visualize_roation
            AHRSupdateIMU(vImus.at(j)._g.x(), vImus.at(j)._g.y(), vImus.at(j)._g.z(),
                          vImus.at(j)._a.x(), vImus.at(j)._a.y(), vImus.at(j)._a.z());
            AngleAxisd a_vicon(vVicons.at(i)._q.normalized());
            AngleAxisd a_imu(Quaterniond(q0,q1,q2,q3));
            vicon_rot.at<float>(0,0) = a_vicon.angle()*a_vicon.axis()[0];
            vicon_rot.at<float>(0,1) = a_vicon.angle()*a_vicon.axis()[1];
            vicon_rot.at<float>(0,2) = a_vicon.angle()*a_vicon.axis()[2];

            imu_rot.at<float>(0,0) = a_imu.angle()*a_imu.axis()[0];
            imu_rot.at<float>(0,1) = a_imu.angle()*a_imu.axis()[1];
            imu_rot.at<float>(0,2) = a_imu.angle()*a_imu.axis()[2];
            Mat vicon_rot_mat, imu_rot_mat;
            Rodrigues(vicon_rot,vicon_rot_mat);
            Rodrigues(imu_rot, imu_rot_mat);
            cv::Affine3f vicon_viz_pose(vicon_rot_mat, Vec3f(1,1,0));
            cv::Affine3f imu_viz_pose(imu_rot_mat,Vec3f(1,1,0));
            myWindow.setWidgetPose("IMU Widget",imu_viz_pose);
            myWindow.setWidgetPose("Vicon Widget", vicon_viz_pose);
            myWindow.spinOnce(1, true);
            cout<<"imu num :"<<j<<endl;
            waitKey(0);
#endif
        }
        Eigen::Vector3d ViconVelocity =
                                (vVicons.at(i+1)._t - vVicons.at(i-1)._t) / (vVicons.at(i+1)._time - vVicons.at(i-1)._time);

        estimator.processVicon(vVicons.at(i)._t, vVicons.at(i)._time, vVicons.at(i)._q, ViconVelocity);
    }

}

/*******************************************************************************************/

void LoadImus(const string &strImuPath, vector<IMUData> &vImus) {
    ifstream fImus;
    fImus.open(strImuPath.c_str());
    vImus.reserve(30000);

    bool first_line = true;
    while (!fImus.eof()) {
        string s;
        getline(fImus, s);
        if (!s.empty()) {
            if (first_line) {      //remove first line in data.csv
                first_line = false;
                continue;
            }
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[7];
            while (ss >> tmpd) {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            IMUData imuData(data[1], data[2], data[3],
                            data[4], data[5], data[6], data[0] / 1e9);
            vImus.push_back(imuData);
        }
    }
}

void LoadVicons(const string &strViconPath, vector<ViconData> &vVicons) {
    ifstream fVicons;
    fVicons.open(strViconPath.c_str());
    vVicons.reserve(30000);

    while (!fVicons.eof()) {
        string s;
        getline(fVicons, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[8];
            while (ss >> tmpd) {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 8)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            ViconData viconData(data[4], data[5], data[6], data[7],
                                data[1], data[2], data[3], data[0]/1e9);
            vVicons.push_back(viconData);
        }
    }
}

// 该文件仅仅只保存了 当前的时间戳与utc时间戳的差值.
void LoadUtcTimeOffset(const string &strTimeoffsetPath, double &utcTimeOffset) {
    ifstream fTimeOffset;
    fTimeOffset.open(strTimeoffsetPath.c_str());
    while (!fTimeOffset.eof()) {
        string s;
        getline(fTimeOffset, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            while (ss >> tmpd) {
                utcTimeOffset = tmpd / 1e9;
                cnt++;
                if (cnt == 1)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
        }
    }
}
