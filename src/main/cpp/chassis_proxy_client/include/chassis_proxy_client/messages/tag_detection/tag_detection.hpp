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
    int32_t x_pos;
    int32_t y_pos;
    int32_t omega_pos;

    static std::vector<uint8_t> serialize(const TagDetectionMsg& msg);

    static TagDetectionMsg deserialize(const std::vector<uint8_t>& data);
};
}  // namespace proxy_client
