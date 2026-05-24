#include "photo_capture.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <iostream>

PhotoCapture::PhotoCapture()
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    pipeline_.start(cfg);

    // Warm-up: wait for a second to let auto-exposure settle
    for(int i=0; i < 10; i++) pipeline_.wait_for_frames();

    cam_thread_ = std::thread(&PhotoCapture::cameraThread, this);
}

PhotoCapture::~PhotoCapture()
{
    running_ = false;
    if (cam_thread_.joinable()) cam_thread_.join();
    pipeline_.stop();
}

void PhotoCapture::cameraThread()
{
    try {
        while (running_)
        {
            rs2::frameset frames = pipeline_.wait_for_frames(5000);
            rs2::video_frame color_frame = frames.get_color_frame();

            if (!color_frame) continue;

            // Update the internal image buffer
            cv::Mat temp = cv::Mat(
                cv::Size(color_frame.get_width(), color_frame.get_height()),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP
            );
            
            // Deep copy to prevent data being overwritten by next frame
            color_image_ = temp.clone();

            cv::imshow("Live Feed", color_image_);
            cv::waitKey(1);
        }
    } catch (const std::exception& e) {
        std::cerr << "Camera thread error: " << e.what() << std::endl;
    }
}

void PhotoCapture::saveImage(const std::string& filename)
{
    if (color_image_.empty()) {
        std::cerr << "Cannot save: Image buffer is empty!" << std::endl;
        return;
    }

    std::filesystem::create_directories("screenshots");
    std::string path = "screenshots/" + filename;

    if (cv::imwrite(path, color_image_)) {
        std::cout << "Successfully saved: " << path << std::endl;
    } else {
        std::cerr << "Failed to save: " << path << std::endl;
    }
}