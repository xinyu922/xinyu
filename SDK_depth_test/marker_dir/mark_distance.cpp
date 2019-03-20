// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "MarkerDetection.h"
#include <librealsense2/rs.hpp>
// #include <feature_msgs/FeaturePointsStampedArray.h>
#include <fstream>
#include <iostream>
#include <time.h>

#include <fstream>
#include <iomanip>


class DistanceCalc
{
public:
    DistanceCalc() {}
    cv::Mat GrabColorImg(rs2::frameset &frames, rs2::pipeline &pipe);
    int CalcDistance(std::vector<int> &roiRegion);

};

cv::Mat DistanceCalc::GrabColorImg(rs2::frameset &frames, rs2::pipeline &pipe)
{
    frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();

    cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    return color;
}


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current))
        {
            return true;
        }
    }
    return false;
}

void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x ++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            // Check if the depth value is invalid (<=0) or greator than the threshold
            if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
            {
                auto offset = depth_pixel_index * other_bpp;
                std::memset(&p_other_frame[offset], 0x99, other_bpp);
            }
        }
    }
}


int main(int argc, char** argv)
{
    ofstream outfile;
    outfile.open("RealSense_depth.csv");
    outfile << "time," << "depth" << "\n";
    time_t timep;


    rs2::pipeline pipe;
    rs2::pipeline_profile profile = pipe.start();

    float depth_scale = get_depth_scale(profile.get_device());

    rs2_stream align_to = find_stream_to_align(profile.get_streams());

    rs2::align align(align_to);
    rs2::colorizer color_map;

    float depth_clipping_distance = 4.f;
    std::vector<int> roiRegion;
    roiRegion.resize(4);
    CMarkerDetection markDetector;

    while(true)
    {
        for(int i=0; i<4; i++)
        {
            roiRegion[i] = 0;
        }
        // Using the align object, we block the application until a frameset is available
        rs2::frameset frameset = pipe.wait_for_frames();

        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        // Get processed aligned frame
        auto processed = align.process(frameset);

        rs2::video_frame other_frame = processed.first(align_to);
//        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
        rs2::depth_frame aligned_depth_frame = color_map(processed.get_depth_frame());
        rs2::frame color_frame = processed.get_color_frame();

        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }

        cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        markDetector.MarkerDetection(&color, roiRegion, 100);

        int x = (roiRegion[0] + roiRegion[1]) / 2;
        int y = (roiRegion[2] + roiRegion[3]) / 2;
        cv::Point point;
        point.x = x;
        point.y = y;
        if(x == 0 || y == 0)
            continue;
//        std::cout << aligned_depth_frame.get_distance(x, y) << std::endl;
        double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        now = now/1000;
        outfile << std::setprecision(20) << now << "," << x << "," << y << "," << aligned_depth_frame.get_distance(x, y) << std::endl;
        cv::circle(color, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 3);
        cv::imshow("colorImg", color);
        cv::waitKey(1);
    }
}


