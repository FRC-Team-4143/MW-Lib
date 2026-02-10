#pragma once

#include <cstdint>
#include <vector>

namespace proxy_client {
struct TagDetectionMsg {
    // Unique message identifier
    const uint8_t msg_id{ 15u };

    // Timestamp
    int32_t sec{ 0 };
    int32_t nanosec{ 0 };

    // Tag ID
    std::vector<uint8_t> tag_ids;

    // Robot Pose
    double x_pos;
    double y_pos;
    double theta_pos;

    static std::vector<uint8_t> serialize(const TagDetectionMsg& msg);

    static TagDetectionMsg deserialize(const std::vector<uint8_t>& data);
};
}  // namespace proxy_client
