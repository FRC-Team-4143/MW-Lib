#pragma once

#include <cstdint>
#include <vector>

namespace proxy_client {

struct SyncResponseMsg {
    // Unique message identifier
    const uint8_t msg_id{ 61u };

    // Request identifier echoed back
    int32_t req_id{ 0 };

    // Server timestamps: when request was received and when response sent
    int32_t server_recv_sec{ 0 };
    int32_t server_recv_nanosec{ 0 };
    int32_t server_send_sec{ 0 };
    int32_t server_send_nanosec{ 0 };

    static std::vector<uint8_t> serialize(const SyncResponseMsg& msg);

    static SyncResponseMsg deserialize(const std::vector<uint8_t>& data);
};

}  // namespace proxy_client
