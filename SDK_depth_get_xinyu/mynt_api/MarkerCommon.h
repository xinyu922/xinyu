#pragma once

#include <string>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

struct sMarkerAreaInfo {
    std::vector<cv::Point> corner;
	int min_x, min_y;
	int max_x, max_y;

	void CalcSize(int percentage)
	{
		min_x =  1000000; 
		min_y =  1000000;
		max_x = -1000000;
		max_y = -1000000;

		for (int i=0; i<4; ++i) {
			min_x = min(min_x, corner[i].x);
			max_x = max(max_x, corner[i].x);
			min_y = min(min_y, corner[i].y);
			max_y = max(max_y, corner[i].y);
		}
		int halfPerCut_X = (max_x - min_x) * (100 - percentage) / 100 / 2;
		int halfPerCut_y = (max_y - min_y) * (100 - percentage) / 100 / 2;
		min_x += halfPerCut_X; 		
		max_x -= halfPerCut_y;
		min_y += halfPerCut_X;
		max_y -= halfPerCut_y;
		
	}
};

struct sMarkerImg
{
	string name;
	double matching;

    std::vector<cv::Mat> storage;
    cv::Mat gray;
    cv::Mat image;

	sMarkerImg (const char *fileName)
	{
		name = fileName;
		matching = 0.;

        image = cv::imread (fileName);

        cv::cvtColor(image, gray, CV_RGB2GRAY);
        cv::threshold( gray, gray, 128, 255, CV_THRESH_BINARY_INV );

//        cv::findContours (gray, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
//		cvDrawContours (image, contours, CV_RGB(255,255,0), CV_RGB(0,255,0), 10, 1, CV_AA);
	}

	~sMarkerImg ()
	{
		// cvReleaseImage(&image);
		// cvReleaseImage(&gray);
//		cvReleaseMemStorage (&storage);
	}
};