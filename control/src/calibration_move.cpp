// #include <memory>
// #include <thread>
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <string>
// #include <fstream>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>

// #include <opencv2/opencv.hpp>
// #include <librealsense2/rs.hpp> // RealSense SDK

// // ================== UTILS ==================
// double deg2rad(double deg) { return deg * M_PI / 180.0; }

// std::vector<double> toRadians(const std::vector<double>& degs)
// {
//     std::vector<double> rads;
//     for (double d : degs)
//         rads.push_back(deg2rad(d));
//     return rads;
// }

// // ================== GLOBAL DATA ==================
// cv::Mat latest_image;
// bool image_received = false;

// std::vector<geometry_msgs::msg::Pose> ee_poses;
// int image_count = 0;

// // ================== CAPTURE DATA ==================
// // void captureData(
// //     moveit::planning_interface::MoveGroupInterface &move_group,
// //     rs2::pipeline &pipeline,
// //     rclcpp::Logger logger)
// // {
// //     //! Wait to ensure robot is fully stopped
// //     rclcpp::sleep_for(std::chrono::milliseconds(1500));

// //     //! Wait for fresh image
// //     int tries = 0;
// //     while (!image_received && tries < 10) {
// //         rs2::frameset frames;
// //         if (pipeline.poll_for_frames(&frames)) {
// //             rs2::frame color_frame = frames.get_color_frame();
// //             if (color_frame) {
// //                 latest_image = cv::Mat(
// //                     cv::Size(color_frame.get_width(), color_frame.get_height()),
// //                     CV_8UC3,
// //                     (void*)color_frame.get_data(),
// //                     cv::Mat::AUTO_STEP
// //                 ).clone();
// //                 image_received = true;
// //             }
// //         }
// //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
// //         tries++;
// //     }

// //     if (!image_received) {
// //         RCLCPP_ERROR(logger, "No image received from RealSense!");
// //         return;
// //     }

// //     //! Save image
// //     std::string filename = "calib_images/img_" + std::to_string(image_count) + ".png";
// //     cv::imwrite(filename, latest_image);
// //     RCLCPP_INFO(logger, "Saved %s", filename.c_str());

// //     //! Save pose
// //     auto pose = move_group.getCurrentPose().pose;
// //     ee_poses.push_back(pose);

// //     image_count++;
// // }

// // void captureData(
// //     moveit::planning_interface::MoveGroupInterface &move_group,
// //     rs2::pipeline &pipeline,
// //     rclcpp::Logger logger)
// // {
// //     //! Wait to ensure robot is fully stopped
// //     rclcpp::sleep_for(std::chrono::milliseconds(1500));

// //     //! Wait for frames from RealSense
// //     rs2::frameset frames;
// //     int tries = 0;
// //     while (tries < 10)
// //     {
// //         frames = pipeline.wait_for_frames();
// //         if (frames)
// //             break;
// //         rclcpp::sleep_for(std::chrono::milliseconds(100));
// //         tries++;
// //     }

// //     if (!frames)
// //     {
// //         RCLCPP_ERROR(logger, "No frames received from RealSense!");
// //         return;
// //     }

// //     //! Get color frame
// //     rs2::video_frame color_frame = frames.get_color_frame(); // cast to video_frame

// //     //! Convert to OpenCV Mat
// //     cv::Mat img(
// //         cv::Size(color_frame.get_width(), color_frame.get_height()),
// //         CV_8UC3,
// //         (void*)color_frame.get_data(),
// //         cv::Mat::AUTO_STEP);

// //     //! Save image
// //     std::string filename = "calib_images/img_" + std::to_string(image_count) + ".png";
// //     cv::imwrite(filename, img);
// //     RCLCPP_INFO(logger, "Saved %s", filename.c_str());

// //     //! Save pose
// //     auto pose = move_group.getCurrentPose().pose;
// //     ee_poses.push_back(pose);

// //     image_count++;
// // }

// void captureData(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     rs2::pipeline &pipeline,
//     rclcpp::Logger logger)
// {
//     rclcpp::sleep_for(std::chrono::milliseconds(1500));

//     rs2::frameset frames;
//     try {
//         frames = pipeline.wait_for_frames();
//     } catch (...) {
//         RCLCPP_ERROR(logger, "Failed to get frames!");
//         return;
//     }

//     rs2::video_frame color_frame = frames.get_color_frame();

//     //! CRITICAL CHECK
//     if (!color_frame)
//     {
//         RCLCPP_ERROR(logger, "Invalid color frame!");
//         return;
//     }

//     int width = color_frame.get_width();
//     int height = color_frame.get_height();

//     //! ANOTHER CRITICAL CHECK
//     if (width == 0 || height == 0)
//     {
//         RCLCPP_ERROR(logger, "Frame has invalid size!");
//         return;
//     }

//     cv::Mat img(
//         cv::Size(width, height),
//         CV_8UC3,
//         (void*)color_frame.get_data(),
//         cv::Mat::AUTO_STEP
//     );

//     //! Clone to avoid memory issues
//     cv::Mat img_copy = img.clone();

//     std::string filename = "calib_images/img_" + std::to_string(image_count) + ".png";
//     cv::imwrite(filename, img_copy);

//     RCLCPP_INFO(logger, "Saved %s", filename.c_str());

//     auto pose = move_group.getCurrentPose().pose;
//     ee_poses.push_back(pose);

//     image_count++;
// }

// // ================== MOVE FUNCTION ==================
// bool moveToWaypoint(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<double> &target,
//     const std::string &name,
//     rclcpp::Logger logger)
// {
//     move_group.setJointValueTarget(target);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) ==
//                     moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//         RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
//         return false;
//     }

//     RCLCPP_INFO(logger, "Executing %s", name.c_str());
//     auto exec_result = move_group.execute(plan);

//     if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
//         RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
//         return false;
//     }

//     return true;
// }

// // ================== SAVE POSES ==================
// void savePoses()
// {
//     std::ofstream file("ee_poses.txt");

//     for (auto &p : ee_poses) {
//         file << p.position.x << " "
//              << p.position.y << " "
//              << p.position.z << " "
//              << p.orientation.x << " "
//              << p.orientation.y << " "
//              << p.orientation.z << " "
//              << p.orientation.w << "\n";
//     }

//     file.close();
// }

// // ================== MAIN ==================
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>("eye_in_hand_calibration");
//     auto logger = node->get_logger();

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     //! ==================== RealSense setup ====================
//     rs2::pipeline pipeline;
//     rs2::config cfg;
//     cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
//     pipeline.start(cfg);

//     rclcpp::sleep_for(std::chrono::seconds(2));

//     //! ================== 20 WAYPOINTS ==================
//     std::vector<std::vector<double>> waypoints = {
//         toRadians({-69.03, -109.52, -9.89, -150.27, 89.49, 19.20}),
//         toRadians({-114.20, -126.17, -9.82, -138.49, 113.92, 334.49}),
//         toRadians({-29.49, -125.41, -9.21, -163.51, 71.25, 46.75}),
//         toRadians({-66.98, -91.91, -84.68, -95.64, 91.81, 15.78}),
//         toRadians({-101.36, -97.99, -83.09, -88.67, 117.16, 357.57}),
//         toRadians({-121.11, -121.88, -55.57, -107.48, 120.22, 317.69}),
//         toRadians({-31.17, -123.17, -37.12, -139.24, 72.34, 62.50}),
//         toRadians({-22.38, -129.29, -23.68, -146.13, 70.85, 56.98}),
//         toRadians({-67.84, -93.80, -97.07, -84.51, 90.93, 19.23}),
//         toRadians({-98.68, -93.35, -96.07, -82.22, 110.39, 357.60}),
//         toRadians({-112.44, -102.56, -95.34, -81.22, 128.70, 340.47}),
//         toRadians({-118.64, -112.90, -90.61, -79.55, 123.94, 329.78}),
//         toRadians({-35.64, -98.77, -89.75, -102.19, 72.40, 44.71}),
//         toRadians({-25.91, -111.90, -71.89, -122.33, 64.32, 55.40}),
//         toRadians({-28.75, -120.71, -67.30, -117.22, 70.73, 57.27}),
//         toRadians({-75.99, -83.74, -52.24, -124.81, 99.84, 24.31}),
//         toRadians({-124.45, -103.17, -53.34, -123.86, 115.85, 354.84}),
//         toRadians({-9.47, -105.80, -39.29, -143.01, 66.06, 78.62}),
//         toRadians({-119.82, -132.91, -65.04, -104.68, 130.93, 339.04}),
//         toRadians({-20.61, -129.42, -61.15, -124.18, 65.65, 61.05})
//     };

//     //! ================== EXECUTION LOOP ==================
//     int wp_id = 0;
//     for (auto &wp : waypoints)
//     {
//         std::string name = "Waypoint " + std::to_string(wp_id);

//         if (!moveToWaypoint(move_group, wp, name, logger))
//             continue;

//         captureData(move_group, pipeline, logger);

//         wp_id++;
//     }

//     savePoses();
//     RCLCPP_INFO(logger, "Data collection complete.");

//     pipeline.stop();
//     rclcpp::shutdown();
//     return 0;
// }





// #include <memory>
// #include <thread>
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <string>
// #include <fstream>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>

// #include <opencv2/opencv.hpp>
// #include <librealsense2/rs.hpp> 

// // ================== UTILS ==================
// double deg2rad(double deg) { return deg * M_PI / 180.0; }

// std::vector<double> toRadians(const std::vector<double>& degs)
// {
//     std::vector<double> rads;
//     for (double d : degs)
//         rads.push_back(deg2rad(d));
//     return rads;
// }

// // ================== GLOBAL DATA ==================
// std::vector<geometry_msgs::msg::Pose> ee_poses;
// int image_count = 0;

// // ================== CAPTURE DATA ==================
// void captureData(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     rs2::pipeline &pipeline,
//     rclcpp::Logger logger)
// {
//     // Wait to ensure robot vibrations have settled
//     rclcpp::sleep_for(std::chrono::milliseconds(1500));

//     rs2::frameset frames;
//     try {
//         // Wait for a coherent set of frames
//         frames = pipeline.wait_for_frames(5000); 
//     } catch (const rs2::error & e) {
//         RCLCPP_ERROR(logger, "RealSense error: %s", e.what());
//         return;
//     } catch (...) {
//         RCLCPP_ERROR(logger, "Failed to get frames!");
//         return;
//     }

//     rs2::video_frame color_frame = frames.get_color_frame();

//     if (!color_frame) {
//         RCLCPP_ERROR(logger, "Invalid color frame pointer!");
//         return;
//     }

//     int width = color_frame.get_width();
//     int height = color_frame.get_height();

//     // PREVENT std::bad_array_new_length
//     // Check for 0 or ridiculously large values that cause memory allocation crashes
//     if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
//         RCLCPP_ERROR(logger, "Sanity check failed! Garbage frame size detected: %dx%d", width, height);
//         return;
//     }

//     try {
//         // Create Mat pointing to RS2 data buffer
//         cv::Mat img(
//             cv::Size(width, height),
//             CV_8UC3,
//             (void*)color_frame.get_data(),
//             cv::Mat::AUTO_STEP
//         );

//         // Clone to a new memory block so we own the data before the frame goes out of scope
//         cv::Mat img_copy = img.clone();

//         std::string filename = "calib_images/img_" + std::to_string(image_count) + ".png";
//         if (cv::imwrite(filename, img_copy)) {
//             RCLCPP_INFO(logger, "Saved %s", filename.c_str());
            
//             // Only save the pose if the image was successfully saved
//             auto pose = move_group.getCurrentPose().pose;
//             ee_poses.push_back(pose);
//             image_count++;
//         } else {
//             RCLCPP_ERROR(logger, "Failed to save image! Check if 'calib_images' folder exists.");
//         }
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(logger, "Memory/OpenCV error during capture: %s", e.what());
//     }
// }

// // ================== MOVE FUNCTION ==================
// bool moveToWaypoint(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<double> &target,
//     const std::string &name,
//     rclcpp::Logger logger)
// {
//     move_group.setJointValueTarget(target);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//         RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
//         return false;
//     }

//     RCLCPP_INFO(logger, "Executing %s", name.c_str());
//     auto exec_result = move_group.execute(plan);

//     return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
// }

// // ================== SAVE POSES ==================
// void savePoses(rclcpp::Logger logger)
// {
//     std::ofstream file("ee_poses.txt");
//     if (!file.is_open()) {
//         RCLCPP_ERROR(logger, "Could not open ee_poses.txt for writing!");
//         return;
//     }

//     for (const auto &p : ee_poses) {
//         file << p.position.x << " " << p.position.y << " " << p.position.z << " "
//              << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << "\n";
//     }
//     file.close();
//     RCLCPP_INFO(logger, "Poses saved to ee_poses.txt");
// }

// // ================== MAIN ==================
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>("eye_in_hand_calibration");
//     auto logger = node->get_logger();

//     // Use a multi-threaded executor or a separate thread for MoveIt spinning
//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();

//     moveit::planning_interface::MoveGroupInterface move_group(node, "ur_onrobot_manipulator");
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     //! ==================== RealSense setup ====================
//     rs2::pipeline pipeline;
//     rs2::config cfg;
//     // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
//     cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    
//     RCLCPP_INFO(logger, "Starting RealSense pipeline...");
//     pipeline.start(cfg);

//     // Warm-up: Throw away the first 30 frames to let auto-exposure settle
//     for (int i = 0; i < 30; i++) {
//         pipeline.wait_for_frames();
//     }

//     //! ================== 20 WAYPOINTS ==================
//     std::vector<std::vector<double>> waypoints = {
//         toRadians({-69.03, -109.52, -9.89, -150.27, 89.49, 19.20}),
//         toRadians({-114.20, -126.17, -9.82, -138.49, 113.92, 334.49}),
//         toRadians({-29.49, -125.41, -9.21, -163.51, 71.25, 46.75}),
//         toRadians({-66.98, -91.91, -84.68, -95.64, 91.81, 15.78}),
//         toRadians({-101.36, -97.99, -83.09, -88.67, 117.16, 357.57}),
//         toRadians({-121.11, -121.88, -55.57, -107.48, 120.22, 317.69}),
//         toRadians({-31.17, -123.17, -37.12, -139.24, 72.34, 62.50}),
//         toRadians({-22.38, -129.29, -23.68, -146.13, 70.85, 56.98}),
//         toRadians({-67.84, -93.80, -97.07, -84.51, 90.93, 19.23}),
//         toRadians({-98.68, -93.35, -96.07, -82.22, 110.39, 357.60}),
//         toRadians({-112.44, -102.56, -95.34, -81.22, 128.70, 340.47}),
//         toRadians({-118.64, -112.90, -90.61, -79.55, 123.94, 329.78}),
//         toRadians({-35.64, -98.77, -89.75, -102.19, 72.40, 44.71}),
//         toRadians({-25.91, -111.90, -71.89, -122.33, 64.32, 55.40}),
//         toRadians({-28.75, -120.71, -67.30, -117.22, 70.73, 57.27}),
//         toRadians({-75.99, -83.74, -52.24, -124.81, 99.84, 24.31}),
//         toRadians({-124.45, -103.17, -53.34, -123.86, 115.85, 354.84}),
//         toRadians({-9.47, -105.80, -39.29, -143.01, 66.06, 78.62}),
//         toRadians({-119.82, -132.91, -65.04, -104.68, 130.93, 339.04}),
//         toRadians({-20.61, -129.42, -61.15, -124.18, 65.65, 61.05})
//     };

//     //! ================== EXECUTION LOOP ==================
//     for (size_t i = 0; i < waypoints.size(); ++i)
//     {
//         std::string name = "Waypoint " + std::to_string(i);

//         if (!moveToWaypoint(move_group, waypoints[i], name, logger)) {
//             RCLCPP_WARN(logger, "Skipping capture for %s due to move failure.", name.c_str());
//             continue;
//         }

//         captureData(move_group, pipeline, logger);
//     }

//     savePoses(logger);
//     RCLCPP_INFO(logger, "Data collection complete. Shutting down.");

//     pipeline.stop();
//     rclcpp::shutdown();
//     return 0;
// }

// #include <memory>
// #include <thread>
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <string>
// #include <fstream>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>

// #include <opencv2/opencv.hpp>
// #include <librealsense2/rs.hpp> 

// // ================== UTILS ==================
// double deg2rad(double deg) { return deg * M_PI / 180.0; }

// std::vector<double> toRadians(const std::vector<double>& degs)
// {
//     std::vector<double> rads;
//     for (double d : degs)
//         rads.push_back(deg2rad(d));
//     return rads;
// }

// // ================== GLOBAL DATA ==================
// std::vector<geometry_msgs::msg::Pose> ee_poses;
// int image_count = 0;

// // ================== CAPTURE DATA ==================
// void captureData(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     rs2::pipeline &pipeline,
//     rclcpp::Logger logger)
// {
//     // Ensure robot is steady
//     rclcpp::sleep_for(std::chrono::milliseconds(1500));

//     rs2::frameset frames;
//     try {
//         frames = pipeline.wait_for_frames(5000); 
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(logger, "RealSense capture failed: %s", e.what());
//         return;
//     }

//     rs2::video_frame color_frame = frames.get_color_frame();
//     if (!color_frame) return;

//     int width = color_frame.get_width();
//     int height = color_frame.get_height();

//     // Sanity check dimensions to avoid std::bad_array_new_length
//     if (width <= 0 || height <= 0 || width > 5000) return;

//     try {
//         cv::Mat img(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
//         cv::Mat img_copy = img.clone();

//         std::string filename = "calib_images/img_" + std::to_string(image_count) + ".png";
//         if (cv::imwrite(filename, img_copy)) {
//             RCLCPP_INFO(logger, "Saved %s", filename.c_str());
//             ee_poses.push_back(move_group.getCurrentPose().pose);
//             image_count++;
//         }
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(logger, "OpenCV Error: %s", e.what());
//     }
// }

// // ================== MOVE FUNCTION (from your working code) ==================
// bool moveToWaypoint(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<double> &target,
//     const std::string &name,
//     rclcpp::Logger logger)
// {
//     move_group.setJointValueTarget(target);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//         RCLCPP_INFO(logger, "Executing %s", name.c_str());
//         auto exec_result = move_group.execute(plan);
//         if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
//             RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
//             return false;
//         }
//     } else {
//         RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
//         return false;
//     }

//     rclcpp::sleep_for(std::chrono::seconds(1));
//     return true;
// }

// // ================== SAVE POSES ==================
// void savePoses()
// {
//     std::ofstream file("ee_poses.txt");
//     for (const auto &p : ee_poses) {
//         file << p.position.x << " " << p.position.y << " " << p.position.z << " "
//              << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << " " << p.orientation.w << "\n";
//     }
//     file.close();
// }

// // ================== MAIN ==================
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     // Using the NodeOptions from your working code
//     auto node = std::make_shared<rclcpp::Node>(
//         "eye_in_hand_calibration",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     auto logger = node->get_logger();

//     // Start executor thread
//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();
//     RCLCPP_INFO(logger, "Works 3");

//     // Initialization delay (common in UR/MoveIt setups)
//     rclcpp::sleep_for(std::chrono::seconds(2));

//     // Initialize MoveGroupInterface
//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "ur_onrobot_manipulator");
//     RCLCPP_INFO(logger, "Works 4");

//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);
//     RCLCPP_INFO(logger, "Works 5");

//     //! ==================== RealSense setup ====================
//     rs2::pipeline pipeline;
//     rs2::config cfg;
//     cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
//     RCLCPP_INFO(logger, "Works 6");
    
//     try {
//         RCLCPP_INFO(logger, "Starting RealSense...");
//         pipeline.start(cfg);
//         for (int i = 0; i < 30; i++) pipeline.wait_for_frames(); // Warmup
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(logger, "Failed to start RealSense: %s", e.what());
//         return 1;
//     }
//     RCLCPP_INFO(logger, "Works 7");

//     //! ================== 20 WAYPOINTS ==================
//     std::vector<std::vector<double>> waypoints = {
//         toRadians({-69.03, -109.52, -9.89, -150.27, 89.49, 19.20}),
//         toRadians({-114.20, -126.17, -9.82, -138.49, 113.92, 334.49}),
//         toRadians({-29.49, -125.41, -9.21, -163.51, 71.25, 46.75}),
//         toRadians({-66.98, -91.91, -84.68, -95.64, 91.81, 15.78}),
//         toRadians({-101.36, -97.99, -83.09, -88.67, 117.16, 357.57}),
//         toRadians({-121.11, -121.88, -55.57, -107.48, 120.22, 317.69}),
//         toRadians({-31.17, -123.17, -37.12, -139.24, 72.34, 62.50}),
//         toRadians({-22.38, -129.29, -23.68, -146.13, 70.85, 56.98}),
//         toRadians({-67.84, -93.80, -97.07, -84.51, 90.93, 19.23}),
//         toRadians({-98.68, -93.35, -96.07, -82.22, 110.39, 357.60}),
//         toRadians({-112.44, -102.56, -95.34, -81.22, 128.70, 340.47}),
//         toRadians({-118.64, -112.90, -90.61, -79.55, 123.94, 329.78}),
//         toRadians({-35.64, -98.77, -89.75, -102.19, 72.40, 44.71}),
//         toRadians({-25.91, -111.90, -71.89, -122.33, 64.32, 55.40}),
//         toRadians({-28.75, -120.71, -67.30, -117.22, 70.73, 57.27}),
//         toRadians({-75.99, -83.74, -52.24, -124.81, 99.84, 24.31}),
//         toRadians({-124.45, -103.17, -53.34, -123.86, 115.85, 354.84}),
//         toRadians({-9.47, -105.80, -39.29, -143.01, 66.06, 78.62}),
//         toRadians({-119.82, -132.91, -65.04, -104.68, 130.93, 339.04}),
//         toRadians({-20.61, -129.42, -61.15, -124.18, 65.65, 61.05})
//     };

//     //! ================== EXECUTION LOOP ==================
//     for (size_t i = 0; i < waypoints.size(); ++i)
//     {
//         std::string name = "Waypoint " + std::to_string(i);
//         if (moveToWaypoint(move_group, waypoints[i], name, logger)) {
//             captureData(move_group, pipeline, logger);
//         }
//     }

//     savePoses();
//     RCLCPP_INFO(logger, "Calibration data collection finished.");

//     pipeline.stop();
//     rclcpp::shutdown();
//     return 0;
// }

// #include <memory>
// #include <thread>
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <string>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <std_msgs/msg/string.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// // RealSense + OpenCV
// #include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>

// double deg2rad(double deg) { return deg * M_PI / 180.0; }

// std::vector<double> toRadians(const std::vector<double>& degs)
// {
//     std::vector<double> rads;
//     for (double d : degs)
//         rads.push_back(deg2rad(d));
//     return rads;
// }

// ================== IMAGE CAPTURE ==================
// void captureImage(rs2::pipeline &pipeline, int &image_id, rclcpp::Logger logger)
// {
//     //! Wait a bit to ensure robot is stable
//     rclcpp::sleep_for(std::chrono::milliseconds(500));

//     rs2::frameset frames;

//     try {
//         frames = pipeline.wait_for_frames();
//     } catch (...) {
//         RCLCPP_ERROR(logger, "Failed to get frames!");
//         return;
//     }

//     rs2::video_frame color_frame = frames.get_color_frame();

//     if (!color_frame) {
//         RCLCPP_ERROR(logger, "Invalid color frame!");
//         return;
//     }

//     int width = color_frame.get_width();
//     int height = color_frame.get_height();

//     if (width == 0 || height == 0) {
//         RCLCPP_ERROR(logger, "Invalid frame size!");
//         return;
//     }

//     cv::Mat img(
//         cv::Size(width, height),
//         CV_8UC3,
//         (void*)color_frame.get_data(),
//         cv::Mat::AUTO_STEP
//     );

//     //! Clone to avoid memory issues
//     cv::Mat img_copy = img.clone();

//     std::string filename = "image_" + std::to_string(image_id) + ".png";
//     cv::imwrite(filename, img_copy);

//     RCLCPP_INFO(logger, "Saved %s", filename.c_str());

//     image_id++;
// // }

// // ================== MOVE FUNCTION ==================
// bool moveToWaypoint(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<double> &target,
//     const std::string &name,
//     rclcpp::Logger logger)
// {
//     move_group.setJointValueTarget(target);

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (success) {
//         RCLCPP_INFO(logger, "Executing %s", name.c_str());

//         auto exec_result = move_group.execute(plan);

//         if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
//             RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
//             return false;
//         }
//     } else {
//         RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
//         return false;
//     }

//     //! Allow robot to settle
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     return true;
// }

// // ================== MAIN ==================
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "control",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     auto logger = node->get_logger();

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     // std::thread([&executor]() { executor.spin(); }).detach();

//     // using moveit::planning_interface::MoveGroupInterface;
//     // MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

//     std::thread([&executor]() { executor.spin(); }).detach();

//     // Give executor time to process initial messages
//     rclcpp::sleep_for(std::chrono::seconds(2));  // ← ADD THIS

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

//     move_group.setMaxVelocityScalingFactor(0.3);
//     move_group.setMaxAccelerationScalingFactor(0.3);

//     //! ==================== RealSense setup ====================
//     // rs2::pipeline pipeline;
//     // rs2::config cfg;
//     // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
//     // pipeline.start(cfg);

//     // rs2::pipeline pipeline;
//     // rs2::config cfg;
//     // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);

//     // try {
//     //     pipeline.start(cfg);
//     //     RCLCPP_INFO(logger, "RealSense pipeline started successfully");
//     // } catch (const rs2::error &e) {
//     //     RCLCPP_FATAL(logger, "RealSense error: %s", e.what());
//     //     rclcpp::shutdown();
//     //     return 1;
//     // } catch (const std::exception &e) {
//     //     RCLCPP_FATAL(logger, "Failed to start pipeline: %s", e.what());
//     //     rclcpp::shutdown();
//     //     return 1;
//     // }

//     // int image_id = 0;

//     rclcpp::sleep_for(std::chrono::seconds(2));

//     // -------------------- GRIPPER COMMAND --------------------
//     // auto gripper_pub = node->create_publisher<std_msgs::msg::String>(
//     //     "/gripper/command", 10
//     // );

//     // bool gripper_done = false;

//     // auto gripper_sub = node->create_subscription<std_msgs::msg::String>(
//     //     "/gripper/status", 10, [&](std_msgs::msg::String::SharedPtr msg){
//     //         if(msg->data == "done") gripper_done = true;
//     //     }
//     // );

//     // auto publishGripperCommand = [&](const std::string &cmd){
//     //     gripper_done = false;

//     //     std_msgs::msg::String msg;
//     //     msg.data = cmd;
//     //     gripper_pub->publish(msg);

//     //     RCLCPP_INFO(logger, "Sent gripper command: %s", cmd.c_str());

//     //     rclcpp::Rate rate(10);
//     //     while(!gripper_done && rclcpp::ok())
//     //         rate.sleep();
//     // };

//     // ================== WAYPOINT LIST ==================
//     std::vector<std::vector<double>> waypoints = {
//         toRadians({-69.03, -109.52, -9.89, -150.27, 89.49, 19.20}),
//         toRadians({-114.20, -126.17, -9.82, -138.49, 113.92, 334.49}),
//         toRadians({-29.49, -125.41, -9.21, -163.51, 71.25, 46.75}),
//         toRadians({-66.98, -91.91, -84.68, -95.64, 91.81, 15.78}),
//         toRadians({-101.36, -97.99, -83.09, -88.67, 117.16, 357.57}),
//         toRadians({-121.11, -121.88, -55.57, -107.48, 120.22, 317.69}),
//         toRadians({-31.17, -123.17, -37.12, -139.24, 72.34, 62.50}),
//         toRadians({-22.38, -129.29, -23.68, -146.13, 70.85, 56.98}),
//         toRadians({-67.84, -93.80, -97.07, -84.51, 90.93, 19.23}),
//         toRadians({-98.68, -93.35, -96.07, -82.22, 110.39, 357.60}),
//         toRadians({-112.44, -102.56, -95.34, -81.22, 128.70, 340.47}),
//         toRadians({-118.64, -112.90, -90.61, -79.55, 123.94, 329.78}),
//         toRadians({-35.64, -98.77, -89.75, -102.19, 72.40, 44.71}),
//         toRadians({-25.91, -111.90, -71.89, -122.33, 64.32, 55.40}),
//         toRadians({-28.75, -120.71, -67.30, -117.22, 70.73, 57.27}),
//         toRadians({-75.99, -83.74, -52.24, -124.81, 99.84, 24.31}),
//         toRadians({-124.45, -103.17, -53.34, -123.86, 115.85, 354.84}),
//         toRadians({-9.47, -105.80, -39.29, -143.01, 66.06, 78.62}),
//         toRadians({-119.82, -132.91, -65.04, -104.68, 130.93, 339.04}),
//         toRadians({-20.61, -129.42, -61.15, -124.18, 65.65, 61.05})
//     };

//     // ================== EXECUTION LOOP ==================
//     int wp_id = 0;

//     for (auto &wp : waypoints)
//     {
//         //! Safety check (prevents your previous crash)
//         if (wp.size() != move_group.getJointNames().size())
//         {
//             RCLCPP_ERROR(logger, "Waypoint %d has wrong size!", wp_id);
//             continue;
//         }

//         std::string name = "Waypoint " + std::to_string(wp_id);

//         //! Move robot
//         if (!moveToWaypoint(move_group, wp, name, logger))
//         {
//             RCLCPP_ERROR(logger, "Skipping waypoint %d", wp_id);
//             wp_id++;
//             continue;
//         }

//         //! 📸 Capture image after reaching waypoint
//         // captureImage(pipeline, image_id, logger);

//         wp_id++;
//     }

//     // pipeline.stop();

//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

// -------------------- DEG → RAD --------------------
std::vector<double> toRadians(const std::vector<double>& degs)
{
    std::vector<double> rads;
    for (auto d : degs)
        rads.push_back(d * M_PI / 180.0);
    return rads;
}

// ================== MOVE FUNCTION ==================
bool moveToWaypoint(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const std::vector<double> &target,
    const std::string &name,
    rclcpp::Logger logger)
{
    move_group.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(logger, "Planning failed for %s", name.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Executing %s", name.c_str());

    auto exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger, "Execution failed for %s", name.c_str());
        return false;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    return true;
}

// ================== MAIN ==================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = node->get_logger();

    //! Executor thread (REQUIRED for callbacks)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::sleep_for(std::chrono::seconds(2));

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_onrobot_manipulator");

    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    // ================= CAMERA COMM =================
    auto capture_pub = node->create_publisher<std_msgs::msg::String>("/camera/trigger", 10);

    bool capture_done = false;

    auto capture_sub = node->create_subscription<std_msgs::msg::String>(
        "/camera/done", 10,
        [&](std_msgs::msg::String::SharedPtr msg)
        {
            if (msg->data == "done")
            {
                capture_done = true;
                RCLCPP_INFO(logger, "Camera confirmed capture");
            }
        }
    );

    auto triggerCapture = [&]()
    {
        capture_done = false;

        std_msgs::msg::String msg;
        msg.data = "capture";
        capture_pub->publish(msg);

        RCLCPP_INFO(logger, "Waiting for camera...");

        rclcpp::Rate rate(10);
        while (!capture_done && rclcpp::ok())
            rate.sleep();
    };

    // ================= WAYPOINTS =================
    // std::vector<std::vector<double>> waypoints = {
    //     toRadians({-69.03, -109.52, -9.89, -150.27, 89.49, 19.20}),
    //     toRadians({-114.20, -126.17, -9.82, -138.49, 113.92, 334.49}),
    //     toRadians({-29.49, -125.41, -9.21, -163.51, 71.25, 46.75}),
    //     toRadians({-66.98, -91.91, -84.68, -95.64, 91.81, 15.78}),
    //     toRadians({-101.36, -97.99, -83.09, -88.67, 117.16, 357.57}),
    //     toRadians({-121.11, -121.88, -55.57, -107.48, 120.22, 317.69}),
    //     toRadians({-31.17, -123.17, -37.12, -139.24, 72.34, 62.50}),
    //     toRadians({-22.38, -129.29, -23.68, -146.13, 70.85, 56.98}),
    //     toRadians({-67.84, -93.80, -97.07, -84.51, 90.93, 19.23}),
    //     toRadians({-98.68, -93.35, -96.07, -82.22, 110.39, 357.60}),
    //     toRadians({-112.44, -102.56, -95.34, -81.22, 128.70, 340.47}),
    //     toRadians({-118.64, -112.90, -90.61, -79.55, 123.94, 329.78}),
    //     toRadians({-35.64, -98.77, -89.75, -102.19, 72.40, 44.71}),
    //     toRadians({-25.91, -111.90, -71.89, -122.33, 64.32, 55.40}),
    //     toRadians({-28.75, -120.71, -67.30, -117.22, 70.73, 57.27}),
    //     toRadians({-75.99, -83.74, -52.24, -124.81, 99.84, 24.31}),
    //     toRadians({-124.45, -103.17, -53.34, -123.86, 115.85, 354.84}),
    //     toRadians({-9.47, -105.80, -39.29, -143.01, 66.06, 78.62}),
    //     toRadians({-119.82, -132.91, -65.04, -104.68, 130.93, 339.04}),
    //     toRadians({-20.61, -129.42, -61.15, -124.18, 65.65, 61.05})
    // };

    std::vector<std::vector<double>> waypoints = {
        toRadians({-52.63, -92.84, -14.33, -160.38, 88.81, 41.67}),
        toRadians({-120.8, -110.10, -15.02, -152.63, 116.56, 332.95}),
        toRadians({-140.11, -119.04, -5.55, -156.37, 112.21, 310.43}),
        toRadians({-13.14, -117.39, -4.28, -166.47, 78.92, 75.14}),
        toRadians({-6.99, -121.59, 8.09, -173.47, 77.16, 96.89}),
        toRadians({-71.63, -79.10, -59.38, -130.16, 91.17, 18.91}),
        toRadians({-113.19, -90.35, -48.58, -135.44, 104.09, 348.89}),
        toRadians({-135.57, -109.86, -38.41, -132.82, 111.57, 323.41}),
        toRadians({-21.59, -104.06, -44.06, -140.26, 80.28, 69.90}),
        toRadians({-20.88, -120.29, -17.26, -151.08, 74.78, 97.60}),
        toRadians({-56.42, -61.29, -108.51, -100.98, 87.67, 41.50}),
        toRadians({-123.76, -76.63, -90.47, -103.28, 112.29, 344.91}),
        toRadians({-146.73, -119.02, -42.52, -129.22, 114.51, 310.45}),
        toRadians({-10.10, -94.42, -74.86, -129.40, 75.06, 80.32}),
        toRadians({0.26, -132.75, -37.61, -141.47, 78.37, 99.17}),
        toRadians({-63.40, -51.02, -38.41, -152.44, 84.42, 41.77}),
        toRadians({-2.86, -104.22, -30.47, -155.74, 66.54, 132.64}),
        toRadians({-94.90, -100.98, -13.24, -156.22, 109.28, 43.81}),
        toRadians({-24.38, -112.52, -0.44, -166.88, 83.83, 43.11}),
        toRadians({-90.61, -37.41, -104.41, -107.87, 100.40, 0.45})
    };


    // ================= EXECUTION LOOP =================
    int wp_id = 0;

    for (auto &wp : waypoints)
    {
        if (wp.size() != move_group.getJointNames().size())
        {
            RCLCPP_ERROR(logger, "Waypoint %d invalid size!", wp_id);
            continue;
        }

        std::string name = "Waypoint " + std::to_string(wp_id);

        if (!moveToWaypoint(move_group, wp, name, logger))
        {
            wp_id++;
            continue;
        }

        //! TRIGGER CAMERA
        triggerCapture();

        wp_id++;
    }

    rclcpp::shutdown();
    return 0;
}