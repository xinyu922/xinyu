// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "MarkerDetection.h"
#include <fstream>
#include <iomanip>
#include <chrono>

#include <glog/logging.h>

#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE
std::vector<int> roiRegion;

int main(int argc, char *argv[]) {
  auto &&api = API::Create(argc, argv);
  if (!api) return 1;

  ofstream outfile;
  outfile.open("mynt_depth.csv");
  outfile << "time," << "u," << "v," << "depth" << "\n";
  roiRegion.resize(4);
  CMarkerDetection markDetector;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);

  api->SetDisparityComputingMethodType(DisparityComputingMethod::BM);
  api->EnableStreamData(Stream::DEPTH);
  api->EnableStreamData(Stream::DISPARITY_NORMALIZED);

  api->Start(Source::VIDEO_STREAMING);

  cv::namedWindow("frame");
  //cv::namedWindow("depth_real");
  //cv::namedWindow("depth_normalized");
  while (true) {
    for (int i=0; i<4; i++)
    {
        roiRegion[i] = 0;
    }

    api->WaitForStreams();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&depth_data = api->GetStreamData(Stream::DEPTH);

    if (!left_data.frame.empty() && !depth_data.frame.empty()) 
    {
      cv::Mat img=left_data.frame;
      cv::cvtColor(img,img,CV_GRAY2RGB);
      
      markDetector.MarkerDetection(&img, roiRegion, 100);
    
      int x = (roiRegion[0] + roiRegion[1]) / 2;
      int y = (roiRegion[2] + roiRegion[3]) / 2;
    
      if (x == 0 || y == 0)
      {
          cv::imshow("frame",img);
          cv::waitKey(1);
          // char key = static_cast<char>(cv::waitKey(1));
          // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
          //   break;
          // }
          // continue;
      }
      else
      {
        cv::circle(img, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 3);
      
        float distance = static_cast<float>(depth_data.frame.at<ushort>(cv::Point(x, y)));
        // float distance = depth_data.frame.at<ushort>(cv::Point(x, y));
        if (distance >= 10000 || distance == 0)
           continue;
        string str_distance = to_string(distance/1000);
        cv::putText(img, str_distance+'m', cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);
        cv::imshow("frame",img);
        cv::waitKey(1);
        
        double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        now = now / 1000;
        outfile << std::setprecision(20) << now << "," << x << "," << y << "," << distance/1000 << std::endl;
      } 

    }

    // char key = static_cast<char>(cv::waitKey(1));
    // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
    //   break;
    // }
  }

  api->Stop(Source::VIDEO_STREAMING);
  return 0;
}
