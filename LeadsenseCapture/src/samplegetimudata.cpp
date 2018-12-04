/***************************************************************************************************************
** This sample simply shows how to get IMU data for 10 seconds.                                               **
** There are 4 evo::imu::IMU_DATA_TYPE:                                                                       **
**        evo::imu::IMU_DATA_TYPE_RAW, evo::imu::IMU_DATA_TYPE_RAW_CALIBRATED,                                **
**        evo::imu::IMU_DATA_TYPE_POSITION_6_AXES and evo::imu::IMU_DATA_TYPE_POSITION_9_AXES.                **
** When using evo::imu::IMU_DATA_TYPE_RAW,                                                                    **
**        only `raw_value`, `temperature`, `timestamp` will be filled.                                        **
** When using evo::imu::IMU_DATA_TYPE_RAW_CALIBRATED,                                                         **
**        `raw_value`, `raw_calibrated_value`, `temperature`, `timestamp` will be filled.                     **
** When using evo::imu::IMU_DATA_TYPE_POSITION_6_AXES,                                                        **
**        `raw_value`, `raw_calibrated_value`, `position_6_value`, `temperature`, `timestamp` will be filled. **
** When using evo::imu::IMU_DATA_TYPE_POSITION_9_AXES,                                                        **
**        `raw_value`, `raw_calibrated_value`, `position_9_value`, `temperature`, `timestamp` will be filled. **
** IMU may need calibration before getting position data.                                                     **
***************************************************************************************************************/

#include <iostream>
#include <thread>
#include <iomanip>
#define _USE_MATH_DEFINES 
#include <math.h>

#include <fstream>
#include <time.h>
#include <string>
 
#include "evo_bino_config.h"

//EvoBinoSDK header
// We use StereoCamera here because we only want to get IMU data, and do not need depth calculation.
#include "evo_stereocamera.h"
// #include " evo_global_define.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#define TIME_INTERVAL 10

using namespace std;

int main(int argc, char* argv[])
{

	std::string data_dir="/home/dzsb078/Desktop/";
	string imu_txt = data_dir + "imu.txt";
	ofstream file_imu;
	file_imu.open(imu_txt);

	std::cout << "usage: SampleGetIMUData [data type(0: IMU_DATA_TYPE_RAW(default), 1: IMU_DATA_TYPE_RAW_CALIBRATED, 2: IMU_DATA_TYPE_POSITION_6_AXES 3: IMU_DATA_TYPE_POSITION_9_AXES]" << std::endl;
	std::cout << "you may need to do IMU calibration first, or position result may be wrong." << std::endl;
	
	// Define IMU data type
	evo::imu::IMU_DATA_TYPE data_type = evo::imu::IMU_DATA_TYPE_RAW;

	// Create a camera object
	evo::bino::StereoCamera camera;

	// Open camera
	evo::RESULT_CODE res = camera.open(evo::bino::RESOLUTION_FPS_MODE_HD720_60);
	
	// If successed
	if (res == evo::RESULT_CODE_OK)
	{
		// Print serial number and SDK version
		std::cout << "serial number: " << camera.getSerialNumber() << std::endl;
		std::cout << "SDK version: " << camera.getSDKVersion() << std::endl;

		// Set IMU data type
		camera.setIMUDataType(data_type);
		
		// Start retrieve IMU data
		res = camera.startRetrieveIMU();

		// If successed
		if (res == evo::RESULT_CODE_OK)
		{
			// Time point
			std::chrono::time_point<std::chrono::high_resolution_clock> now, start;
			start = std::chrono::high_resolution_clock::now();

			while (true)
			{
				
				std::vector<evo::imu::IMUData> vector_data = camera.retrieveIMUData();

				// Print newest result
				if (vector_data.size() > 0)
				{
					evo::imu::IMUData data = vector_data.at(vector_data.size() - 1);

					if (data_type == evo::imu::IMU_DATA_TYPE_RAW)
					{
						file_imu << std::setprecision(4) << std::fixed << "accel/gyro/magnet/time:\t"
							<< data.accel[0] << " " << data.accel[1] << " " << data.accel[2] << "\t"
							<< data.gyro[0] << " " << data.gyro[1] << " " << data.gyro[2] << "\t"
							<< data.mag[0] << " " << data.mag[1] << " " << data.mag[2] << "\t"
							<< data.timestamp
							<< std::endl;

						std::cout << std::setprecision(4) << std::fixed << "accel/gyro/magnet/time:\t"
							<< data.accel[0] << " " << data.accel[1] << " " << data.accel[2] << "\t"
							<< data.gyro[0] << " " << data.gyro[1] << " " << data.gyro[2] << "\t"
							<< data.mag[0] << " " << data.mag[1] << " " << data.mag[2] << "\t"
							<< data.timestamp
							<< std::endl;	
					}
					
				}

				now = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> elapsed = now - start;
				if (elapsed.count() > TIME_INTERVAL)
				{
					break;
				}
			}
		}

		// Stop retrieve IMU data
		camera.stopRetrieveIMU();

		std::cout << std::endl << "run over" << std::endl;

		// Close camera
		camera.close();
	}
	else
	{
		std::cerr << "open camera failed: " << evo::result_code2str(res) << std::endl;
	}
	return 0;
}
