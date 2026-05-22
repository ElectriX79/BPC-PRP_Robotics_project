#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mutex>
#include <string>
#include "algorithms/aruco_detector.hpp"

class CameraNode : public rclcpp::Node {
public:
    // Information from aruco marker
    struct MarkerInfo {
        int  id = -1;
        std::string instruction;
        rclcpp::Time stamp;
        bool valid = false;
    };

    struct PairedDetection {
        MarkerInfo escape;
        MarkerInfo treasure;
    };

    CameraNode();

    // Gets pair of the aruco markers
    PairedDetection get_paired_detection() const;

    // Timer of seen marker
    double get_escape_age_seconds(const rclcpp::Time& now) const;
    double get_treasure_age_seconds(const rclcpp::Time& now) const;

private:
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    std::string getInstruction(int id);
    bool isEscapeId(int id) const   { return id >= 0  && id <= 9; }
    bool isTreasureId(int id) const { return id >= 10 && id <= 19; }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    image_transport::Publisher image_pub_;

    algorithms::ArucoDetector detector_;
    cv::Mat last_frame_;
    std::vector<algorithms::ArucoDetector::Aruco> last_detections_;

    mutable std::mutex mutex_;
    PairedDetection paired_;
};