#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <utility> 

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
using namespace Eigen;

std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");

Eigen::Vector3d translation_sum;
Eigen::Quaterniond rotation_sum;          //全局变量
Eigen::Vector3d translation_sum1;
Eigen::Quaterniond rotation_sum1;

Eigen::Matrix3d rotation_avg_by_mult;
float rmse_avg;

int iteration_counter=0;

Eigen::Quaterniond addQ(Eigen::Quaterniond a, Eigen::Quaterniond b)
{
	Eigen::Quaterniond retval;
	if(a.x()*b.x() + a.y()*b.y() + a.z()*b.z() + a.w()*b.w() < 0.0)
	{
		b.x() = -b.x();
		b.y() = -b.y();
		b.z() = -b.z();
		b.w() = -b.w();
	}
	retval.x() = a.x() + b.x();
	retval.y() = a.y() + b.y();
	retval.z() = a.z() + b.z();
	retval.w() = a.w() + b.w();
	return retval;
}

std::pair<MatrixXd, MatrixXd> readArray()
{
	std::ifstream infile(pkg_loc + "/conf/points.txt");     //在之前存储雷达坐标点的文件打开读取
	int num_points=0;

	infile >> num_points;

	ROS_ASSERT(num_points > 0);
	
	MatrixXd lidar1(3,num_points), camera1(3,num_points);

	std::cout << "Num points is:" << num_points << std::endl;
	std::vector<int> sequence;

   std::cout<<3<<std::endl;
	for(int i=0; i<num_points; i++)                                      //读取txt文件里数据
	{
		infile >> lidar1(0,i) >> lidar1(1,i) >> lidar1(2,i);
	}
	for(int i=0; i<num_points; i++)
	{
		infile >> camera1(0,i) >> camera1(1,i) >> camera1(2,i);
	}
	infile.close();

	std::cout<<"after reading txt   lidar//camera"<<std::endl;
std::cout<<lidar1<<std::endl;
std::cout<<camera1<<std::endl;
///std::cout<<lidar1(0,3)<<std::endl;
std::cout<<4<<std::endl;

    for(int i=0; i<num_points; i++)                 //开始把没打到的雷达点序号挑出来
    {
      if((lidar1(0,i)!=0)||(lidar1(1,i)!=0)||(lidar1(2,i)!=0))
      {
std::cout<<50<<std::endl;
            sequence.push_back(i);
      }
    }
std::cout<<5<<std::endl;
    int vecsize=sequence.size();
    std::vector<float > distancelidar;
    std::vector<float > distancecam;
    MatrixXd lidar2(3,vecsize), camera2(3,vecsize);
    for(int i=0; i<vecsize; i++)
    {
        lidar2(0,i)=lidar1(0,sequence.at(i));
        lidar2(1,i)=lidar1(1,sequence.at(i));             //重建一个matrix放进去有效的点对
        lidar2(2,i)=lidar1(2,sequence.at(i));

        camera2(0,i)=camera1(0,sequence.at(i));
        camera2(1,i)=camera1(1,sequence.at(i));
        camera2(2,i)=camera1(2,sequence.at(i));

    }
    for(int i=0; i<vecsize-1; i++)
    {
      distancelidar.push_back(sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
						  pow(lidar2(0,i) - lidar2(0,i+1), 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
						+ pow(lidar2(1,i) - lidar2(1,i+1), 2)
                          + pow(lidar2(2,i) - lidar2(2,i+1), 2)
      ));

        distancecam.push_back(sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
                pow(camera2(0,i) - camera2(0,i+1), 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
                + pow(camera2(1,i) - camera2(1,i+1), 2)
                + pow(camera2(2,i) - camera2(2,i+1), 2)
        ));

    }



    std::cout<<"after being chosen from txt  lidar//camera"<<std::endl;
std::cout<<lidar2<<std::endl;
std::cout<<camera2<<std::endl;
    for(int i=0; i<vecsize-1; i++)
    {
std::cout<<"lidar distance:"<<distancelidar.at(i)<<"  camera distance:"<<distancecam.at(i)<<std::endl;    //把距离算出来，做一个小断点验证得到的点是否准确
    }



    // camera values are stored in variable 'lidar' and vice-versa
	// need to change this
	return std::pair<MatrixXd, MatrixXd>(lidar2, camera2);
	//return std::pair<MatrixXd, MatrixXd>(camera, lidar);
}

// calculates rotation and translation that transforms points in the lidar frame to the camera frame
Matrix4d calc_RT(MatrixXd lidar, MatrixXd camera, int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
{
	if(iteration_counter == 0)      //第一次迭代初始化
	{
		std::ofstream clean_file(pkg_loc + "/log/avg_values.txt", std::ios_base::trunc);
		clean_file.close();

        std::ofstream cleanfile(pkg_loc + "/conf/resultrecord.txt", std::ios_base::trunc);
        clean_file.close();

		translation_sum << 0.0, 0.0, 0.0; 
		rotation_sum = Quaterniond(0.0, 0.0, 0.0, 0.0);
        translation_sum1 << 0.0, 0.0, 0.0;
        rotation_sum1 = Quaterniond(0.0, 0.0, 0.0, 0.0);
		rotation_avg_by_mult << 1.0, 0.0, 0.0, 
								0.0, 1.0, 0.0, 
								0.0, 0.0, 1.0;
		rmse_avg = 0.0;
	}
	int num_points = lidar.cols();
	std::cout << "Number of points: " << num_points << std::endl;
	Vector3d mu_lidar, mu_camera;
	
	mu_lidar << 0.0, 0.0, 0.0;
	mu_camera << 0.0, 0.0, 0.0;

	for(int i=0; i<num_points; i++)
	{
		mu_lidar(0) += lidar(0,i);
		mu_lidar(1) += lidar(1,i);                   //全部加起来，最后要求中心平均值
		mu_lidar(2) += lidar(2,i);
	}
	for(int i=0; i<num_points; i++)
	{
		mu_camera(0) += camera(0,i);               //全部加起来，最后要求中心平均值
		mu_camera(1) += camera(1,i);
		mu_camera(2) += camera(2,i);
	}

	mu_lidar = mu_lidar/num_points;
	mu_camera = mu_camera/num_points;

	if(iteration_counter == 0)
	{
		std::cout << "mu_lidar: \n" << mu_lidar << std::endl;
		std::cout << "mu_camera: \n" << mu_camera << std::endl;
	}

	MatrixXd lidar_centered = lidar.colwise() - mu_lidar;                      //所有点都减去了一个质心
	MatrixXd camera_centered = camera.colwise() - mu_camera;

	if(iteration_counter == 0)
	{
		std::cout << "lidar_centered: \n" << lidar_centered << std::endl;
		std::cout << "camera_centered: \n" << camera_centered << std::endl;           //根据去质心坐标计算旋转矩阵
	}

	Matrix3d cov = camera_centered*lidar_centered.transpose();

	std::cout << cov << std::endl;

	JacobiSVD<MatrixXd> svd(cov, ComputeFullU | ComputeFullV);

	Matrix3d rotation;
	rotation = svd.matrixU() * svd.matrixV().transpose();           //求解，用SVD
	if( rotation.determinant() < 0 )
	{
		Vector3d diag_correct;
		diag_correct << 1.0, 1.0, -1.0; 

		rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
	}
	
	Vector3d translation = mu_camera - rotation*mu_lidar;          //根据所求的旋转矩阵计算t

	// averaging translation and rotation
	translation_sum += translation;
	Quaterniond temp_q(rotation);
	rotation_sum = addQ(rotation_sum, temp_q);

	// averaging rotations by multiplication
	rotation_avg_by_mult = rotation_avg_by_mult.pow(1.0*iteration_counter/(iteration_counter+1))*rotation.pow(1.0/(iteration_counter+1));

	Vector3d ea = rotation.eulerAngles(2, 1, 0);

	std::cout << "ICP Rotation matrix: \n" << rotation << std::endl;
	std::cout << "ICP Rotation in Euler angles: \n" << ea*57.3 << std::endl;
	std::cout << "ICP Translation: \n" << translation << std::endl;

	MatrixXd eltwise_error = (camera - ((rotation*lidar).colwise() + translation)).array().square().colwise().sum();
	double error = sqrt(eltwise_error.sum()/num_points);
	std::cout << "RMSE: " << error << std::endl;

	rmse_avg = rmse_avg + error;

	Matrix4d T;
	T.setIdentity(4,4);
	T.topLeftCorner(3, 3) = rotation;
	T.col(3).head(3) = translation;

	std::cout << "Rigid-body transformation: \n" << T << std::endl;

	iteration_counter++;
	//if(iteration_counter == MAX_ITERS)
	if(iteration_counter%1 == 0)
	{
		std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

		std::cout << "--------------------------------------------------------------------\n";
		std::cout << "After " << iteration_counter << " iterations\n";
		std::cout << "--------------------------------------------------------------------\n";

		std::cout << "Average translation is:" << "\n" << translation_sum/iteration_counter << "\n";
		log_avg_values << iteration_counter << "\n";
		log_avg_values << translation_sum/iteration_counter << "\n";


		rotation_sum.x() = rotation_sum.x()/iteration_counter;
		rotation_sum.y() = rotation_sum.y()/iteration_counter;
		rotation_sum.z() = rotation_sum.z()/iteration_counter;
		rotation_sum.w() = rotation_sum.w()/iteration_counter;
		double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
					 rotation_sum.y()*rotation_sum.y() +
					 rotation_sum.z()*rotation_sum.z() +
					 rotation_sum.w()*rotation_sum.w());
		rotation_sum.x() = rotation_sum.x()/mag;
		rotation_sum.y() = rotation_sum.y()/mag;
		rotation_sum.z() = rotation_sum.z()/mag;
		rotation_sum.w() = rotation_sum.w()/mag;

		Eigen::Matrix3d rotation_avg = rotation_sum.toRotationMatrix();
		std::cout << "Average rotation is:" << "\n" << rotation_avg << "\n";
		Eigen::Matrix3d final_rotation = rotation_avg*lidarToCamera;
		Eigen::Vector3d final_angles = final_rotation.eulerAngles(2, 1, 0);

		//std::cout << "Average rotation by multiplication is:" << "\n" << rotation_avg_by_mult << "\n";

		/*std::cout      << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
					   << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
					   << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";*/

		log_avg_values << std::fixed << std::setprecision(8)
						<< rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
					   << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
					   << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";

		Matrix4d T;
		T.setIdentity(4,4);
		T.topLeftCorner(3, 3) = rotation_avg;
		T.col(3).head(3) = translation_sum/iteration_counter;
		std::cout << "Average transformation is: \n" << T << "\n";
		std::cout << "Final rotation is:" << "\n" << final_rotation << "\n";
		std::cout << "Final ypr is:" << "\n" <<final_angles << "\n";

		std::cout << "Average RMSE is: " <<  rmse_avg*1.0/iteration_counter << "\n";

		MatrixXd eltwise_error_temp = (camera - ((rotation_avg*lidar).colwise() + (translation_sum/iteration_counter))).array().square().colwise().sum();
		double error_temp = sqrt(eltwise_error_temp.sum()/num_points);

		std::cout << "RMSE on average transformation is: " << error_temp << std::endl;
		log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";

 	}


    if(iteration_counter == 50||iteration_counter == 100||iteration_counter == 150||iteration_counter == 200||iteration_counter == 250||
            iteration_counter == 300||iteration_counter == 350||iteration_counter == 400||iteration_counter == 450||iteration_counter == 500||
            iteration_counter == 550||iteration_counter == 600||iteration_counter == 650||iteration_counter == 700||iteration_counter == 750||
            iteration_counter == 800||iteration_counter == 850||iteration_counter == 900||iteration_counter == 950||iteration_counter == 1000)              //对平均点的ICP
    {

        std::cout << "开始计算平均结果！！！" << std::endl;
        std::cout << "开始计算平均结果！！！" << std::endl;
        std::cout << "开始计算平均结果！！！" << std::endl;
        std::cout << "开始计算平均结果！！！" << std::endl;
        std::cout << "开始计算平均结果！！！" << std::endl;

        std::ifstream infile(pkg_loc + "/conf/averagepoints.txt");     //在之前存储雷达坐标点的文件打开读取
        int num_points = 0;

        infile >> num_points;

        ROS_ASSERT(num_points > 0);

        MatrixXd lidar1(3, num_points), camera1(3, num_points);

        std::cout << "Num points is:" << num_points << std::endl;
        std::vector<int> sequence;

        std::cout << 3 << std::endl;
        for (int i = 0; i < num_points; i++)                                      //读取txt文件里数据
        {
            infile >> lidar1(0, i) >> lidar1(1, i) >> lidar1(2, i);
        }
        for (int i = 0; i < num_points; i++) {
            infile >> camera1(0, i) >> camera1(1, i) >> camera1(2, i);
        }
        infile.close();


        std::ifstream infile1(pkg_loc + "/conf/marker_coordinates.txt");         //纸板的参数，空白部分的尺寸
        float temp;
        std::vector<float> boardx;
        std::vector<float> boardz;
        //std::vector<std::pair<float, float> > corner_points;
        for (int j = 0; j < 26; j++)              //11组信息点在板子上的坐标
        {
            infile1 >> temp;
            boardx.push_back(temp / 100.0);     //这里就是计算考虑相机系下角点延展到纸板角点补足的部分，除100是单位转换，一个一个往下读，读5行
            infile1 >> temp;                    //记录版上点坐标
            boardz.push_back(temp / 100.0);
        }

        infile1.close();


        for (int i = 0; i < num_points; i++)                 //开始把没打到的雷达点序号挑出来
        {
            if ((lidar1(0, i) != 0) || (lidar1(1, i) != 0) || (lidar1(2, i) != 0)) {
                sequence.push_back(i);
            }
        }
        int vecsize = sequence.size();
        MatrixXd point2(2, vecsize);
        for (int i = 0; i < vecsize; i++) {
            for (int j = 0; j < 26; j++) {
                if (sequence.at(i) == j) {
                    point2(0, i) = boardx.at(j);
                    point2(1, i) = boardz.at(j);
                }


            }
        }

        std::vector<float> distancelidar;
        std::vector<float> distancecam;
        std::vector<float> distancepoint;
        MatrixXd lidar2(3, vecsize), camera2(3, vecsize);
        for (int i = 0; i < vecsize; i++) {
            lidar2(0, i) = lidar1(0, sequence.at(i));
            lidar2(1, i) = lidar1(1, sequence.at(i));             //重建一个matrix放进去有效的点对
            lidar2(2, i) = lidar1(2, sequence.at(i));

            camera2(0, i) = camera1(0, sequence.at(i));
            camera2(1, i) = camera1(1, sequence.at(i));
            camera2(2, i) = camera1(2, sequence.at(i));

        }
        for (int i = 0; i < vecsize - 1; i++) {
            distancelidar.push_back(
                    sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
                            pow(lidar2(0, i) - lidar2(0, i + 1), 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
                            + pow(lidar2(1, i) - lidar2(1, i + 1), 2)
                            + pow(lidar2(2, i) - lidar2(2, i + 1), 2)
                    ));

            distancecam.push_back(
                    sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
                            pow(camera2(0, i) - camera2(0, i + 1), 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
                            + pow(camera2(1, i) - camera2(1, i + 1), 2)
                            + pow(camera2(2, i) - camera2(2, i + 1), 2)
                    ));
            distancepoint.push_back(
                    sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
                            pow(point2(0, i) - point2(0, i + 1), 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
                            + pow(point2(1, i) - point2(1, i + 1), 2)
                    ));

        }

        for (int i = 0; i < vecsize - 1; i++) {
            std::cout << "lidar distance:" << distancelidar.at(i) << "  camera distance:" << distancecam.at(i)
                      << "points distance:" << distancepoint.at(i) << std::endl;    //把距离算出来，做一个小断点验证得到的点是否准确
        }

////////////////////////////////////////////ICP计算！！！！！！！！！！！
      num_points = lidar2.cols();
        std::cout << "Number of points: " << num_points << std::endl;


        mu_lidar << 0.0, 0.0, 0.0;
        mu_camera << 0.0, 0.0, 0.0;

        for(int i=0; i<num_points; i++)
        {
            mu_lidar(0) += lidar2(0,i);
            mu_lidar(1) += lidar2(1,i);                   //全部加起来，最后要求中心平均值
            mu_lidar(2) += lidar2(2,i);
        }
        for(int i=0; i<num_points; i++)
        {
            mu_camera(0) += camera2(0,i);               //全部加起来，最后要求中心平均值
            mu_camera(1) += camera2(1,i);
            mu_camera(2) += camera2(2,i);
        }

        mu_lidar = mu_lidar/num_points;
        mu_camera = mu_camera/num_points;


        MatrixXd lidar_centered = lidar2.colwise() - mu_lidar;                      //所有点都减去了一个质心
        MatrixXd camera_centered = camera2.colwise() - mu_camera;


        Matrix3d cov = camera_centered*lidar_centered.transpose();

        std::cout << cov << std::endl;

        JacobiSVD<MatrixXd> svd(cov, ComputeFullU | ComputeFullV);

        Matrix3d rotation;
        rotation = svd.matrixU() * svd.matrixV().transpose();           //求解，用SVD
        if( rotation.determinant() < 0 )
        {
            Vector3d diag_correct;
            diag_correct << 1.0, 1.0, -1.0;

            rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
        }

        Vector3d translation = mu_camera - rotation*mu_lidar;          //根据所求的旋转矩阵计算t


        Vector3d ea = rotation.eulerAngles(2, 1, 0);


        translation_sum1 = translation_sum1+translation;          ////////////////////平均的计算
        Quaterniond temp_q1(rotation);
        rotation_sum1 = addQ(rotation_sum1, temp_q1);





        std::cout << "icp Rotation matrix: \n" << rotation << std::endl;
        std::cout << "icp Rotation in Euler angles: \n" << ea*57.3 << std::endl;
        std::cout << "icp Translation: \n" << translation << std::endl;
//
//        MatrixXd eltwise_error = (camera - ((rotation*lidar).colwise() + translation)).array().square().colwise().sum();
//        double error = sqrt(eltwise_error.sum()/num_points);
//        std::cout << "RMSE: " << error << std::endl;
//
//        rmse_avg = rmse_avg + error;
//
        Matrix4d T;
        T.setIdentity(4,4);
        T.topLeftCorner(3, 3) = rotation;
        T.col(3).head(3) = translation;
//
        std::cout << "Rigid-body transformation: \n" << T << std::endl;
//
//        iteration_counter++;
//        //if(iteration_counter == MAX_ITERS)
//        if(iteration_counter%1 == 0)
//        {
//            std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);
//
//            std::cout << "--------------------------------------------------------------------\n";
//            std::cout << "After " << iteration_counter << " iterations\n";
//            std::cout << "--------------------------------------------------------------------\n";
//
//            std::cout << "Average translation is:" << "\n" << translation_sum/iteration_counter << "\n";
//            log_avg_values << iteration_counter << "\n";
//            log_avg_values << translation_sum/iteration_counter << "\n";
//
//
//            rotation_sum.x() = rotation_sum.x()/iteration_counter;
//            rotation_sum.y() = rotation_sum.y()/iteration_counter;
//            rotation_sum.z() = rotation_sum.z()/iteration_counter;
//            rotation_sum.w() = rotation_sum.w()/iteration_counter;
//            double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
//                              rotation_sum.y()*rotation_sum.y() +
//                              rotation_sum.z()*rotation_sum.z() +
//                              rotation_sum.w()*rotation_sum.w());
//            rotation_sum.x() = rotation_sum.x()/mag;
//            rotation_sum.y() = rotation_sum.y()/mag;
//            rotation_sum.z() = rotation_sum.z()/mag;
//            rotation_sum.w() = rotation_sum.w()/mag;
//
//            Eigen::Matrix3d rotation_avg = rotation_sum.toRotationMatrix();
//            std::cout << "Average rotation is:" << "\n" << rotation_avg << "\n";
                std::cout<<"pre-rotation:"<<lidarToCamera.eulerAngles(2, 1, 0)<<std::endl;

            Eigen::Matrix3d final_rotation = rotation*lidarToCamera;
            Eigen::Vector3d final_angles = final_rotation.eulerAngles(                    );
//
//            //std::cout << "Average rotation by multiplication is:" << "\n" << rotation_avg_by_mult << "\n";
//
//            /*std::cout      << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
//                           << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
//                           << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";*/
//
//            log_avg_values << std::fixed << std::setprecision(8)
//                           << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
//                           << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
//                           << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";
//
         
            
            T.topLeftCorner(3, 3) =  final_rotation;
            T.col(3).head(3) = translation;
            std::cout << "final average transformation: \n" << T << "\n";
            std::cout << "Final average rotation is:" << "\n" << final_rotation << "\n";
            std::cout << "Final Euler angles is:" << "\n" <<final_angles*57.3 << "\n";
//
//            std::cout << "Average RMSE is: " <<  rmse_avg*1.0/iteration_counter << "\n";
//
//            MatrixXd eltwise_error_temp = (camera - ((rotation_avg*lidar).colwise() + (translation_sum/iteration_counter))).array().square().colwise().sum();
//            double error_temp = sqrt(eltwise_error_temp.sum()/num_points);
//
//            std::cout << "RMSE on average transformation is: " << error_temp << std::endl;
//            log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";



        std::ofstream outfile(pkg_loc + "/conf/resultrecord.txt", std::ios_base::app);
        outfile<<iteration_counter<<"\n";
        outfile << final_angles(0,0)*57.3<<" " <<final_angles(1,0)*57.3<<" "<<final_angles(2,0)*57.3<< "\n";
	outfile << final_rotation(0,0)<<" " <<final_rotation(0,1)<<" "<<final_rotation(0,2)<<final_rotation(1,0)<<" " <<final_rotation(1,1)<<" "<<final_rotation(1,2)<<final_rotation(2,0)<<" " <<final_rotation(2,1)<<" "<<final_rotation(2,2)<< "\n";
        outfile << translation(0,0)<<" " <<translation(1,0)<<" "<<translation(2,0)<< "\n";
         outfile <<sqrt(pow(translation(0,0), 2)+pow(translation(1,0), 2)+pow(translation(2,0), 2))<<"\n";                  
                          
        outfile<<"\n";
        outfile.close();







        if(iteration_counter == MAX_ITERS)
        {
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "After " << iteration_counter << " iterations\n";
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "！！！！！！！！！！！！！！！！！！！！！！最终结果来啦！！！！！！！！！！！！！！！！！！！！！！！\n";
            std::cout << "Average translation is:" << "\n" << translation_sum1/20.0 << "\n";

             double divided=20.0;

            rotation_sum1.x() = rotation_sum1.x()/divided;
            rotation_sum1.y() = rotation_sum1.y()/divided;
            rotation_sum1.z() = rotation_sum1.z()/divided;
            rotation_sum1.w() = rotation_sum1.w()/divided;
            double mag1 = sqrt(rotation_sum1.x()*rotation_sum1.x() +
                              rotation_sum1.y()*rotation_sum1.y() +
                              rotation_sum1.z()*rotation_sum1.z() +
                              rotation_sum1.w()*rotation_sum1.w());
            rotation_sum1.x() = rotation_sum1.x()/mag1;
            rotation_sum1.y() = rotation_sum1.y()/mag1;
            rotation_sum1.z() = rotation_sum1.z()/mag1;
            rotation_sum1.w() = rotation_sum1.w()/mag1;

            Eigen::Matrix3d rotation_avg1 = rotation_sum1.toRotationMatrix();
            std::cout << "Average rotation is:" << "\n" << rotation_avg1 << "\n";
            Eigen::Matrix3d final_rotation1 = rotation_avg1*lidarToCamera;
            Eigen::Vector3d final_angles1 = final_rotation1.eulerAngles(2, 1, 0);



            Matrix4d T;
            T.setIdentity(4,4);
            T.topLeftCorner(3, 3) = final_rotation1;
            T.col(3).head(3) = translation_sum1/divided;
            std::cout << "Average transformation is: \n" << T << "\n";
            std::cout << "Final rotation is:" << "\n" << final_rotation1 << "\n";
            std::cout << "Final euler angles is:" << "\n" <<final_angles1*57.3 << "\n";




        }








    }








 	//writing files to generate plots
 	
 	/*if(iteration_counter%1 == 0)
	{
		std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

		log_avg_values << iteration_counter << "\n";
		log_avg_values << translation_sum/iteration_counter << "\n";

		double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
					 rotation_sum.y()*rotation_sum.y() +
					 rotation_sum.z()*rotation_sum.z() +
					 rotation_sum.w()*rotation_sum.w());

		Eigen::Quaterniond rot_temp_sum;
		rot_temp_sum.x() = rotation_sum.x()/(mag*iteration_counter);
		rot_temp_sum.y() = rotation_sum.y()/(mag*iteration_counter);
		rot_temp_sum.z() = rotation_sum.z()/(mag*iteration_counter);
		rot_temp_sum.w() = rotation_sum.w()/(mag*iteration_counter);
		
		Eigen::Matrix3d rotation_avg = rot_temp_sum.toRotationMatrix();
		//log_avg_values << rotation_avg << "\n";

		log_avg_values << std::fixed << std::setprecision(8)
						<< rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
					   << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
					   << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";

		MatrixXd eltwise_error_temp = (camera - ((rotation_avg*lidar).colwise() + (translation_sum/iteration_counter))).array().square().colwise().sum();
		double error_temp = sqrt(eltwise_error_temp.sum()/num_points);
		log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";

		log_avg_values.close();
 	}*/
 	


	return T; 
}


void readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config)
{
	std::vector<Matrix4d> marker_pose;

	ROS_ASSERT(marker_info.size()/7 == num_of_marker_in_config);
           

  std::cout<<1<<std::endl;

	int j=0;
	for(int i = 0; i < marker_info.size()/7; i++)
	{

		//std::cout << "In readArucoPose(): " << std::endl;
		std::cout<<2<<std::endl;
		Vector3d trans, rot;
		int marker_id = marker_info[j++];
		trans(0) = marker_info[j++];
		trans(1) = marker_info[j++];
		trans(2) = marker_info[j++];
		rot(0) = marker_info[j++];
		rot(1) = marker_info[j++];
		rot(2) = marker_info[j++];

		//std::cout << "\n" << "Marker id:" << marker_id << "\n" << trans << "\n" << rot << std::endl;

		
		Transform<double,3,Affine> aa;
		aa = AngleAxis<double>(rot.norm(), rot/rot.norm());

		Matrix4d g;
		g.setIdentity(4,4);
		//std::cout << "Rot matrix is: \n" << aa*g << std::endl;
		g = aa*g;

		Matrix4d T;
		T.setIdentity(4,4);
		T.topLeftCorner(3, 3) = g.topLeftCorner(3,3);//.transpose();
		T.col(3).head(3) = trans;

		marker_pose.push_back(T);                                  //这一串for循环做的事情就是把marker_pose得到，后面要用
		

std::cout<<"transformation between board and camera:"<<T<<std::endl;                                                           //marker_pose做的事情就是把相机系marker系之间的变换关系传输过来

		//std::cout << "transformation matrix is: \n" << T << std::endl;
	}


	//std::vector<std::vector<std::pair<float, float> > > marker_coordinates;
	std::ifstream infile(pkg_loc + "/conf/marker_coordinates.txt");         //纸板的参数，空白部分的尺寸
	std::ofstream outfile(pkg_loc + "/conf/points.txt", std::ios_base::app);    //读CONF文件，为了写入aruco得到的相机下角点坐标

	int num_of_markers;
	infile >> num_of_markers;    //第一行读一下marker数量

	for(int i=0; i<num_of_markers; i++)        //一块板子读一次信息
	{
		float temp;
		std::vector<float> boardx;
        std::vector<float> boardz;
		//std::vector<std::pair<float, float> > corner_points;
		for(int j=0; j<26; j++)              //11组信息点在板子上的坐标
		{
            infile >> temp;
			boardx.push_back(temp/100.0);     //这里就是计算考虑相机系下角点延展到纸板角点补足的部分，除100是单位转换，一个一个往下读，读5行
            infile >> temp;                    //记录版上点坐标
            boardz.push_back(temp/100.0);
		}
//		float la1, ba1,la2,ba2;                //换成两组参数，总共八个点
//		la1=board[4]/2+board[2];              //marker长的一半
//    	ba1=board[4]/2+board[3];               //marker宽的一半
//
//        la2=board[9]/2+board[7];              //marker长的一半
//        ba2=board[9]/2+board[8];               //marker宽的一半

    	/*corner_points.push_back(std::make_pair(ba, 		   board[0]-la));
    	corner_points.push_back(std::make_pair(ba-board[1], board[0]-la));
    	corner_points.push_back(std::make_pair(ba-board[1], -la 		  ));
    	corner_points.push_back(std::make_pair(ba, 		   -la 		  ));*/

    	Matrix4d points_board1,points_board2,points_board3,points_board4,points_board5,points_board6,points_board7;
    	points_board1 << boardx[0], 0, boardz[0], 1,         //最后一维全是1，齐次向量，只用看前三维！！！就是板上角点在marker坐标系下的坐标
    					 boardx[1], 0, boardz[1], 1,
    					 boardx[2], 0, boardz[2], 1,
    					 boardx[3], 0, boardz[3], 1;

        points_board2 << boardx[4], 0, boardz[4], 1,         //最后一维全是1，齐次向量，只用看前三维！！！就是板上角点在marker坐标系下的坐标
                         boardx[5], 0, boardz[5], 1,
                         boardx[6], 0, boardz[6], 1,
                         boardx[7], 0, boardz[7], 1;
        points_board3 << boardx[8], 0, boardz[8], 1,
                        boardx[9], 0, boardz[9], 1,
                         boardx[10], 0, boardz[10], 1,
                         boardx[11], 0, boardz[11], 1;

        points_board4 << boardx[12], 0, boardz[12], 1,
                        boardx[13], 0, boardz[13], 1,
                         boardx[14], 0, boardz[14], 1,
                         boardx[15], 0, boardz[15], 1;
        points_board5 << boardx[16], 0, boardz[16], 1,
                        boardx[17], 0, boardz[17], 1,
                         boardx[18], 0, boardz[18], 1,
                         boardx[19], 0, boardz[19], 1;
        points_board6 << boardx[20], 0, boardz[20], 1,
                        boardx[21], 0, boardz[21], 1,
                         boardx[22], 0, boardz[22], 1,
                         boardx[23], 0, boardz[23], 1;
        points_board7 << boardx[24], 0, boardz[24], 1,
                        boardx[25], 0, boardz[25], 1,
                         boardx[22], 0, boardz[22], 1,
                         boardx[23], 0, boardz[23], 1;

        //                           board[18], 0, board[19], 1,
//                           board[20], 0, board[21], 1,
//                         board[22], 0, board[23], 1;
//        points_board4 << board[24], 0, board[25], 1,         //最后一维全是1，齐次向量，只用看前三维！！！就是板上角点在marker坐标系下的坐标
//                          board[26], 0, board[27], 1,
//                          board[28], 0, board[29], 1,
//                          board[30], 0, board[31], 1;
//        points_board5 << board[32], 0, board[33], 1,         //最后一维全是1，齐次向量，只用看前三维！！！就是板上角点在marker坐标系下的坐标
//                         board[34], 0, board[35], 1,
//                         board[36], 0, board[37], 1,
//                         board[38], 0, board[39], 1;

    	/*std::cout << "Points in before transform: \n" << points_board << std::endl;*/

    	points_board1 = marker_pose[i]*(points_board1.transpose());          //一个简单的变换矩阵之间相乘，四个点在相机系下的坐标就确定了
        points_board2 = marker_pose[i]*(points_board2.transpose());           //transpose转置
        points_board3 = marker_pose[i]*(points_board3.transpose());
       points_board4 = marker_pose[i]*(points_board4.transpose());
       points_board5 = marker_pose[i]*(points_board5.transpose());
       points_board6 = marker_pose[i]*(points_board6.transpose());
    points_board7 = marker_pose[i]*(points_board7.transpose());
//        points_board4 = marker_pose[i]*(points_board2.transpose());
//        points_board5 = marker_pose[i]*(points_board2.transpose());

        /*std::cout << "Board number: " << i+1 << "\n";
        std::cout << "P1: " << ba << " " << board[0]-la << "\n";0.00648776  0.0701962   0.997512  0.0563946
         0          0          0          1

        std::cout << "P2: " << ba-board[1] << " " << board[0]-la << "\n";
        std::cout << "P3: " << ba-board[1] << " " << -la << "\n";
        std::cout << "P4: " << ba << " " << -la << "\n\n";

        std::cout << "Points in camera frame: \n" << points_board << std::endl;*/

    	//marker_coordinates.push_back(corner_points);

    	
    	for(int k=0; k < 4; k++)
    	{
    		outfile << points_board1(0,k) << " " << points_board1(1,k) << " " << points_board1(2,k) <<  "\n";
    	}
        for(int k=0; k < 4; k++)
        {
            outfile << points_board2(0,k) << " " << points_board2(1,k) << " " << points_board2(2,k) <<  "\n";
        }
        for(int k=0; k < 4; k++)
        {
            outfile << points_board3(0,k) << " " << points_board3(1,k) << " " << points_board3(2,k) <<  "\n";
        }
        for(int k=0; k < 4; k++)
        {
            outfile << points_board4(0,k) << " " << points_board4(1,k) << " " << points_board4(2,k) <<  "\n";
        }
        for(int k=0; k < 4; k++)
        {
            outfile << points_board5(0,k) << " " << points_board5(1,k) << " " << points_board5(2,k) <<  "\n";
        }
        for(int k=0; k < 4; k++)
        {
            outfile << points_board6(0,k) << " " << points_board6(1,k) << " " << points_board6(2,k) <<  "\n";
        }
        for(int k=0; k < 2; k++)
        {
            outfile << points_board7(0,k) << " " << points_board7(1,k) << " " << points_board7(2,k) <<  "\n";
        }
//        for(int k=0; k < 4; k++)
//        {
//            outfile << points_board4(0,k) << " " << points_board4(1,k) << " " << points_board4(2,k) <<  "\n";
//        }
//        for(int k=0; k < 4; k++)
//        {
//            outfile << points_board5(0,k) << " " << points_board5(1,k) << " " << points_board5(2,k) <<  "\n";
//        }
    	                        //把纸板角点相机系坐标写到points.txt里


        outfile.close();
        infile.close();


        if(iteration_counter==49||iteration_counter==99||iteration_counter==149||iteration_counter==199||iteration_counter==249||iteration_counter==299||
                iteration_counter==349||iteration_counter==399||iteration_counter==449||iteration_counter==499||iteration_counter==549||iteration_counter==599||
                iteration_counter==649||iteration_counter==699||iteration_counter==749||iteration_counter==799||iteration_counter==849||iteration_counter==899||
                iteration_counter==949||iteration_counter==999)
        {
            std::ofstream outfile(pkg_loc + "/conf/averagepoints.txt", std::ios_base::app);

            for(int k=0; k < 4; k++)
            {
                outfile << points_board1(0,k) << " " << points_board1(1,k) << " " << points_board1(2,k) <<  "\n";
            }
            for(int k=0; k < 4; k++)
            {
                outfile << points_board2(0,k) << " " << points_board2(1,k) << " " << points_board2(2,k) <<  "\n";
            }
            for(int k=0; k < 4; k++)
            {
                outfile << points_board3(0,k) << " " << points_board3(1,k) << " " << points_board3(2,k) <<  "\n";
            }
            for(int k=0; k < 4; k++)
            {
                outfile << points_board4(0,k) << " " << points_board4(1,k) << " " << points_board4(2,k) <<  "\n";
            }
            for(int k=0; k < 4; k++)
            {
                outfile << points_board5(0,k) << " " << points_board5(1,k) << " " << points_board5(2,k) <<  "\n";
            }
            for(int k=0; k < 4; k++)
            {
                outfile << points_board6(0,k) << " " << points_board6(1,k) << " " << points_board6(2,k) <<  "\n";
            }
            for(int k=0; k < 2; k++)
            {
                outfile << points_board7(0,k) << " " << points_board7(1,k) << " " << points_board7(2,k) <<  "\n";
            }



        }
	}






}


void find_transformation(std::vector<float> marker_info, int num_of_marker_in_config, int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
{
	readArucoPose(marker_info, num_of_marker_in_config);    //处理相机系下角点，写到points.txt里,lidar系下点在corners.cpp里就写好了
	std::pair<MatrixXd, MatrixXd> point_clouds = readArray();     //读points.txt,两组点写到PAIR容器里，point_clouds变量是个一一对应pair容器
	Matrix4d T = calc_RT(point_clouds.first, point_clouds.second, MAX_ITERS, lidarToCamera);    //calc_RT在这个文件上方有定义，为ICP，用PAIR容器里的两组点
}
