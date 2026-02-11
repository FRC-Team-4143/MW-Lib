#include "chassis_proxy_client/messages/tag_detection/tag_detection.hpp"

#include "chassis_proxy_client/messages/msg_common.hpp"

namespace proxy_client {

std::vector<uint8_t> TagDetectionMsg::serialize(const TagDetectionMsg& msg) {
    BinOStream buffer;
    buffer << msg.msg_id;
    buffer << msg.timestamp;

    // pose
    buffer << msg.x_pos;
    buffer << msg.y_pos;
    buffer << msg.theta_pos;

    // tag ids
    int32_t tag_count = static_cast<int32_t>(msg.tag_ids.size());
    buffer << tag_count;
    for (const auto& tag_id : msg.tag_ids) {
        buffer << tag_id;
    }

    return buffer.serialize();
}

TagDetectionMsg TagDetectionMsg::deserialize(const std::vector<uint8_t>& data) {
    BinIStream buffer(data);
    TagDetectionMsg msg;

    if (data.empty() || data.at(0) != msg.msg_id) {
        throw SerializationError("msg id mismatch ");
    }

    buffer >> msg.msg_id;
    buffer >> msg.timestamp;

    // pose
    buffer >> msg.x_pos;
    buffer >> msg.y_pos;
    buffer >> msg.theta_pos;

    // tag ids
    int32_t tag_count = 0;
    buffer >> tag_count;
    for (int32_t i = 0; i < tag_count; ++i) {
        uint8_t tag_id;
        buffer >> tag_id;
        msg.tag_ids.push_back(tag_id);
    }

    return msg;
}

}  // namespace proxy_client
