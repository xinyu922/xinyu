#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include "MarkerCommon.h"

using namespace std;

class CMarkerDetection
{
public:
    CMarkerDetection();
    ~CMarkerDetection();

    void MarkerDetection(cv::Mat *src, std::vector<int> &roiRegion, int percentage);
    void DrawResult(cv::Mat &src, sMarkerAreaInfo &marker, std::vector<int> &roiRegion, int percentage);

private:
    bool IsQuadrangle(std::vector<cv::Point> &box1, std::vector<cv::Point> &box2);
    void FindQuadrangleRegion(std::vector<std::vector<cv::Point> > contours, int level, int percentage);
    cv::Mat *CreateMarkerImage(cv::Mat *img, sMarkerAreaInfo &marker);
    int FindMarker(cv::Mat *imgMarker);

public:
    vector<sMarkerImg *> _markerDB;
    vector<sMarkerAreaInfo> _markerDetect;
};