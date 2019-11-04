#include "MarkerDetection.h"

CMarkerDetection::CMarkerDetection()
{
}

CMarkerDetection::~CMarkerDetection()
{
    for (unsigned int i=0; i<_markerDB.size(); i++)
    {
        delete _markerDB[i];
    }
}

void CMarkerDetection::DrawResult(cv::Mat &src, sMarkerAreaInfo &marker, std::vector<int> &roiRegion, int percentage)
{
    cv::Scalar color = cv::Scalar(0, 0, 255);


//    cv::line(src, marker.corner[0], marker.corner[1], color, 2);
//    cv::line(src, marker.corner[1], marker.corner[2], color, 2);
//    cv::line(src, marker.corner[2], marker.corner[3], color, 2);
//    cv::line(src, marker.corner[3], marker.corner[0], color, 2);
    // cv::fillConvexPoly(src, &marker.corner[0], marker.corner.size(), 255, 8, 0);
    // marker.CalcSize(percentage);
    roiRegion[0] = marker.min_x;
    roiRegion[1] = marker.max_x;
    roiRegion[2] = marker.min_y;
    roiRegion[3] = marker.max_y;
    std::vector<cv::Point> corners;
    corners.resize(4);
    corners[0].x = marker.min_x;
    corners[0].y = marker.min_y;
    corners[1].x = marker.max_x;
    corners[1].y = marker.min_y;
    corners[2].x = marker.max_x;
    corners[2].y = marker.max_y;
    corners[3].x = marker.min_x;
    corners[3].y = marker.max_y;
    cv::fillConvexPoly(src, &corners[0], corners.size(), 255, 8, 0);
    // std::cout << marker.corner[0] << ":" << marker.corner[1] << ":" << marker.corner[2] << ":" << marker.corner[3] << std::endl;



//    cv::line(src, marker.corner[0], marker.corner[2], color, 2);
//    cv::line(src, marker.corner[1], marker.corner[3], color, 2);
//    cv::imshow("detector", src);
}

inline double perpendicular(cv::Point &l0, cv::Point &l1, cv::Point &p)
{
    double vx = l1.x - l0.x;
    double vy = l1.y - l0.y;
    double px = p.x - l0.x;
    double py = p.y - l0.y;
    double vl = sqrt(vx*vx + vy*vy);

    return (px*vy - py*vx) / vl;
}

bool CMarkerDetection::IsQuadrangle(std::vector<cv::Point> &box1, std::vector<cv::Point> &box2)
{
    cv::Point p1[4];
    cv::Point p2[4];

    for (int i=0; i<4; i++)
    {
        p1[i] = box1[i];
        p2[i] = box2[i];
    }

    double d[8] = {
        fabs(perpendicular(p1[0], p1[2], p2[0])),
        fabs(perpendicular(p1[0], p1[2], p2[1])),
        fabs(perpendicular(p1[0], p1[2], p2[2])),
        fabs(perpendicular(p1[0], p1[2], p2[3])),

        fabs(perpendicular(p1[1], p1[3], p2[0])),
        fabs(perpendicular(p1[1], p1[3], p2[1])),
        fabs(perpendicular(p1[1], p1[3], p2[2])),
        fabs(perpendicular(p1[1], p1[3], p2[3])),
    };

    const double threshold = 5;

    if (d[0] < threshold && d[2] < threshold && d[5] < threshold && d[7] < threshold) return true;
    if (d[1] < threshold && d[3] < threshold && d[4] < threshold && d[6] < threshold) return true;
    return false;
}

void CMarkerDetection::FindQuadrangleRegion(std::vector<std::vector<cv::Point> > contours, int level, int percentage)
{

    for (int i=0; i<contours.size()-1; i++)
    {
        if(level%2 == 0 && contours[i].size() == 4 )
        {
            if(IsQuadrangle(contours[i], contours[i+1]))
            {
                sMarkerAreaInfo marker;
                marker.corner = contours[i+1];

                marker.CalcSize(percentage);

                _markerDetect.push_back(marker);
            }
        }

    }
}

void CMarkerDetection::MarkerDetection(cv::Mat *src, std::vector<int> &roiRegion, int percentage)
{
    cv::Mat imgGray;
    cv::cvtColor(*src, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(imgGray, imgGray, cv::Size(3, 3), 0.0);

    cv::Mat imgBinary = imgGray.clone();


    cv::adaptiveThreshold(imgBinary, imgBinary, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 71, 15);
//    cv::imshow("img1", imgBinary);

    CvMemStorage *storage = cvCreateMemStorage(0);
    std::vector<std::vector<cv::Point> > contours0;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(imgBinary, contours0, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<std::vector<cv::Point> > contours;
    contours.resize(contours0.size());
    for(std::size_t k=0; k<contours0.size(); k++)
    {
        cv::approxPolyDP(cv::Mat(contours0[k]), contours[k], 3, true);
    }
//    cv::Mat cnt_img = cv::Mat::zeros(500, 500, CV_8UC3);
//    cv::drawContours(cnt_img, contours, -1, cv::Scalar(128, 255, 255), 3, cv::LINE_AA, hierarchy, std::abs(4));
//    cv::imshow("contours", cnt_img);

    _markerDetect.clear();

    FindQuadrangleRegion(contours, 6, percentage);


    for (unsigned int i=0; i<_markerDetect.size(); ++i) {
        DrawResult(*src, _markerDetect[i], roiRegion, percentage);
    }


}