#pragma once

#include <cstdint>
#include <vector>

namespace proxy_client {

struct SyncRequestMsg {
    // Unique message identifier
    const uint8_t msg_id{ 60u };

    // Timestamp when request was sent (client)
    int32_t sec{ 0 };
    int32_t nanosec{ 0 };

    // Request identifier to match responses
    int32_t req_id{ 0 };

    static std::vector<uint8_t> serialize(const SyncRequestMsg& msg);

    static SyncRequestMsg deserialize(const std::vector<uint8_t>& data);
};

}  // namespace proxy_client
