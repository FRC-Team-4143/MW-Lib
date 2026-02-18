#pragma once

#include <cstdint>
#include <localization_msgs/msg/tag_solution.hpp>
#include <vector>

#include "chassis_proxy_client/messages/msg_common.hpp"

namespace proxy_client {
struct TagDetectionMsg {
    // Unique message identifier
    const uint8_t msg_id{ 15u };

    // Timestamp
    TimeStamp timestamp;

    // Tag ID
    std::vector<uint8_t> tag_ids;

    // Robot Pose
    double x_pos;
    double y_pos;
    double theta_pos;

    void loadFromMsg(const std::vector<uint8_t>& tag_ids, const geometry_msgs::msg::Pose& tag_pose_in_base,
                     const TimeStamp& stamp);

    static std::vector<uint8_t> serialize(const TagDetectionMsg& msg);

    static TagDetectionMsg deserialize(const std::vector<uint8_t>& data);
};
}  // namespace proxy_client
