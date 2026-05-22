#include "nodes/camera_node.hpp"

CameraNode::CameraNode() : Node("camera_node") {
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/bpc_prp_robot/camera/compressed", 10, std::bind(&CameraNode::imageCallback, this, std::placeholders::_1));

    auto node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    image_transport::ImageTransport it(node);
    image_pub_ = it.advertise("camera_node/image", 10);

    paired_.escape.stamp = this->now();
    paired_.treasure.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), "CameraNode started");
}

CameraNode::PairedDetection CameraNode::get_paired_detection() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return paired_;
}

double CameraNode::get_escape_age_seconds(const rclcpp::Time& now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!paired_.escape.valid) return 1e9;
    return (now - paired_.escape.stamp).seconds();
}

double CameraNode::get_treasure_age_seconds(const rclcpp::Time& now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!paired_.treasure.valid) return 1e9;
    return (now - paired_.treasure.stamp).seconds();
}

void CameraNode::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
    cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame received");
        return;
    }

    last_frame_ = frame;

    last_detections_ = detector_.detect(frame);

    if (!last_detections_.empty()) {
        rclcpp::Time now = this->now();
        std::lock_guard<std::mutex> lock(mutex_);

        for (const auto& det : last_detections_) {
            std::string instr = getInstruction(det.id);

            if (isEscapeId(det.id)) {
                paired_.escape.id          = det.id;
                paired_.escape.instruction = instr;
                paired_.escape.stamp       = now;
                paired_.escape.valid       = true;
            }
            else if (isTreasureId(det.id)) {
                paired_.treasure.id          = det.id;
                paired_.treasure.instruction = instr;
                paired_.treasure.stamp       = now;
                paired_.treasure.valid       = true;
            }

            RCLCPP_INFO(this->get_logger(), "Marker ID: %d -> %s", det.id, instr.c_str());
        }
    }

    if (!last_detections_.empty()) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        for (auto& a : last_detections_) {
            corners.push_back(a.corners);
            ids.push_back(a.id);
        }
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_.publish(out_msg);
}

std::string CameraNode::getInstruction(int id) {
    switch (id) {
        case 0:  return "escape_straight";
        case 1:  return "escape_left";
        case 2:  return "escape_right";
        case 10: return "treasure_straight";
        case 11: return "treasure_left";
        case 12: return "treasure_right";
        default: return "unknown";
    }
}