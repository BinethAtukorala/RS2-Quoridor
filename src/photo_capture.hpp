#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>

class PhotoCapture
{
public:
    PhotoCapture(); 
    ~PhotoCapture();

    // Changed to accept a filename string
    void saveImage(const std::string& filename); 

private:
    void cameraThread();

    rs2::pipeline pipeline_;
    cv::Mat color_image_;
    std::thread cam_thread_;
    std::atomic<bool> running_{true}; // Using atomic for thread safety
};