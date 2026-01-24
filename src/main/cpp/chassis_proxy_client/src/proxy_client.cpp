#include "chassis_proxy_client/proxy_client.hpp"

#include <yaml-cpp/yaml.h>

#include "chassis_proxy_client/messages/obj_det/proxy_det.hpp"
#include "chassis_proxy_client/messages/tag_detection/tag_detection.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace proxy_client {

ProxyClientNode::ProxyClientNode() : basin::node_core::NodeCore("proxy_client") {
    params_ = std::make_unique<ProxyClientNodeParams>(this);

    // Always force an Unknown ID to the end of the class list
    params_->class_names.push_back("Unknown");

    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        params_->detections_topic, rclcpp::SensorDataQoS(), std::bind(&ProxyClientNode::detectionsCb, this, _1));
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        params_->camera_info_topic, rclcpp::SystemDefaultsQoS(), std::bind(&ProxyClientNode::camInfoCb, this, _1));

    tag_solution_sub_ = create_subscription<localization_msgs::msg::TagSolution>(
        params_->tag_solution_topic, rclcpp::SystemDefaultsQoS(), std::bind(&ProxyClientNode::tagSolutionCb, this, _1));

    log_metadata_pub_ = create_publisher<logging_msgs::msg::LogMetadata>("/log/metadata", rclcpp::SystemDefaultsQoS());

    // Setup the UDP server and register message types for receive
    udp_server_ = std::make_unique<UdpServer>(params_->client_port);
    udp_server_->registerType<MatchInfo>(std::bind(&ProxyClientNode::matchInfoCb, this, _1));
    udp_server_->registerType<AutoSnapshot>(std::bind(&ProxyClientNode::autosnapCb, this, _1));

    tryConnectUdpServer();

    RCLCPP_INFO(get_logger(), "Proxy client startup complete");
}

void ProxyClientNode::onTick() {
    // Called periodically by the base class tick timer
    try {
        udp_server_->processPackets();
    } catch (const std::runtime_error& e) {
        RCLCPP_WARN_STREAM(get_logger(), "Failed to handle proxy packet with error: " << e.what());
    }
}

void ProxyClientNode::shutdown() {
    RCLCPP_INFO(get_logger(), "Proxy client shutting down");
    // Cleanup resources if needed
    udp_client_.reset();
    udp_server_.reset();
}

void ProxyClientNode::tryConnectUdpServer() {
    try {
        // Just setup the local part of the client and configure the server address
        udp_client_ = std::make_unique<UdpClient>(params_->server_addr, params_->server_port);
    } catch (const std::system_error& e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Failed to connect to " << params_->server_addr << ":" << params_->server_port);
        RCLCPP_ERROR_STREAM(get_logger(), "Error: " << e.what());
    }
}

void ProxyClientNode::tagSolutionCb(localization_msgs::msg::TagSolution::ConstSharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), "Received Tag Solution with " << msg->detected_tags.size() << " detections");

    TagDetectionMsg proxy_msg;
    proxy_msg.sec = msg->header.stamp.sec;
    proxy_msg.nanosec = msg->header.stamp.nanosec;
}

void ProxyClientNode::matchInfoCb(MatchInfo msg) {
    // multiline
    RCLCPP_INFO_STREAM(get_logger(), "Received Match Info for event: " << msg.evt_name);

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "event_name" << YAML::Value << msg.evt_name;
    emitter << YAML::Key << "match_num" << YAML::Value << msg.match_num;
    emitter << YAML::Key << "match_type" << YAML::Value << msg.match_type;
    emitter << YAML::Key << "station" << YAML::Value << msg.station;
    emitter << YAML::EndMap;

    logging_msgs::msg::LogMetadata meta_msg;
    meta_msg.key_name = "match_info";
    meta_msg.timed_event = false;
    meta_msg.yaml_metadata = emitter.c_str();

    log_metadata_pub_->publish(meta_msg);
}

void ProxyClientNode::autosnapCb(AutoSnapshot msg) {
    // multiline
    RCLCPP_INFO_STREAM(get_logger(), "Received Snapshot Request: " << msg.evt_name);
}

ProxyVisionDetection loadDetection(const std_msgs::msg::Header& hdr, const vision_msgs::msg::Detection2D& msg,
                                   uint8_t class_id, sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info, int idx,
                                   int count) {
    ProxyVisionDetection proxy_msg;
    proxy_msg.sec = hdr.stamp.sec;
    proxy_msg.nanosec = hdr.stamp.nanosec;
    proxy_msg.detection_count = count;
    proxy_msg.detection_idx = idx;
    proxy_msg.class_id = class_id;

    // theta = atan((x - cx) / fx)
    proxy_msg.theta_x = (atan((msg.bbox.center.position.x - cam_info->k.at(2)) / cam_info->k.at(0)) / M_PI) * 180.0;
    proxy_msg.theta_y = (atan((msg.bbox.center.position.y - cam_info->k.at(5)) / cam_info->k.at(4)) / M_PI) * 180.0;

    return proxy_msg;
}

ProxyVisionDetection loadEmptyDetections(const std_msgs::msg::Header& hdr) {
    ProxyVisionDetection proxy_msg;
    proxy_msg.sec = hdr.stamp.sec;
    proxy_msg.nanosec = hdr.stamp.nanosec;
    proxy_msg.detection_count = 0;

    return proxy_msg;
}

void ProxyClientNode::camInfoCb(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { cam_info_ = msg; }

void ProxyClientNode::detectionsCb(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
    if (!cam_info_) {
        RCLCPP_WARN(get_logger(), "Unable to process detections. Waiting for camera info");
        return;
    }

    if (msg->detections.empty()) {
        ProxyVisionDetection proxy_msg = loadEmptyDetections(msg->header);
        udp_client_->sendMsg(proxy_msg);

    } else {
        // filter for greatest bbox area
        vision_msgs::msg::Detection2D best;
        double best_area = 0;
        for (const vision_msgs::msg::Detection2D& detection : msg->detections) {
            double area = detection.bbox.size_x * detection.bbox.size_y;

            if (area > best_area) {
                best = detection;
            }
        }

        // Try to find a class id match and force to unknown if unknown
        std::vector<std::string>::iterator it =
            std::find(params_->class_names.begin(), params_->class_names.end(), best.id);
        if (it == params_->class_names.end()) {
            it = (params_->class_names.end() - 1);
        }

        // now convert it to the packet
        uint8_t class_id = std::distance(params_->class_names.begin(), it);
        ProxyVisionDetection proxy_msg = loadDetection(msg->header, best, class_id, cam_info_, 0, 1);
        udp_client_->sendMsg(proxy_msg);
    }
}
}  // namespace proxy_client

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<proxy_client::ProxyClientNode> node = std::make_shared<proxy_client::ProxyClientNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
}
