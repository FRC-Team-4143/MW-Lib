#pragma once

#include <basin/node_core/node_core.hpp>
#include <basin/node_core/node_params.hpp>
#include <logging_msgs/msg/log_metadata.hpp>
#include <logging_msgs/srv/snapshot_request.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <localization_msgs/msg/tag_solution.hpp>

#include "chassis_proxy_client/messages/auto_snapshot/auto_snapshot.hpp"
#include "chassis_proxy_client/messages/match_info/match_info.hpp"
#include "chassis_proxy_client/udp_client.hpp"
#include "chassis_proxy_client/udp_server.hpp"

namespace proxy_client {

struct ProxyClientNodeParams : public basin::node_core::NodeParams {
    int server_port{ 5809 };                 ///< Server port running on the roborio to connect to
    int client_port{ 5809 };                 ///< Server port running on the local device for the roborio to send data
    std::string server_addr{ "127.0.0.1" };  ///< Server address for the roborio

    std::string odom_topic{ "/odometry/filtered" };          ///< odometry topic
    std::string module_topic{ "/chassis/modules" };          ///< swerve module data topic
    std::string detections_topic{ "/vision/detections" };    ///< vision detections topic
    std::string camera_info_topic{ "/vision/camera_info" };  ///< vision detections topic
    std::string tag_solution_topic{ "/vision/tag_solution" };  ///< tag solution topic

    std::vector<std::string> class_names{};

    ProxyClientNodeParams(rclcpp::Node* node) : basin::node_core::NodeParams(node) {
        BASIN_PARAM("server_port", server_port, node);
        BASIN_PARAM("server_addr", server_addr, node);
        BASIN_PARAM("client_port", client_port, node);

        BASIN_PARAM("odom_topic", odom_topic, node);
        BASIN_PARAM("module_topic", module_topic, node);

        BASIN_PARAM("detections_topic", detections_topic, node);
        BASIN_PARAM("camera_info_topic", camera_info_topic, node);
        BASIN_PARAM("tag_solution_topic", tag_solution_topic, node);
    }
};

class ProxyClientNode : public basin::node_core::NodeCore {
   public:
    ProxyClientNode();

    void onTick() override;

    void shutdown() override;

    void tryConnectUdpServer();

    void processRecievedPackets();

    void detectionsCb(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

    void camInfoCb(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    void tagSolutionCb(localization_msgs::msg::TagSolution::ConstSharedPtr msg);

    void matchInfoCb(MatchInfo msg);

    void autosnapCb(AutoSnapshot msg);

   private:
    std::unique_ptr<ProxyClientNodeParams> params_;

    std::unique_ptr<UdpClient> udp_client_;
    std::unique_ptr<UdpServer> udp_server_;

    sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;  ///< Camera info message received last

    rclcpp::TimerBase::SharedPtr recv_timer_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<localization_msgs::msg::TagSolution>::SharedPtr tag_solution_sub_;

    rclcpp::Publisher<logging_msgs::msg::LogMetadata>::SharedPtr log_metadata_pub_;
};
}  // namespace proxy_client
