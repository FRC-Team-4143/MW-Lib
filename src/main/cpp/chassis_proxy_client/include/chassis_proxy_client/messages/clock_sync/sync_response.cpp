#include "chassis_proxy_client/messages/clock_sync/sync_response.hpp"

#include "chassis_proxy_client/messages/msg_common.hpp"

namespace proxy_client {

std::vector<uint8_t> SyncResponseMsg::serialize(const SyncResponseMsg& msg) {
    BinOStream buffer;
    buffer << msg.msg_id;
    buffer << htonl(msg.req_id);
    buffer << htonl(msg.server_recv_sec);
    buffer << htonl(msg.server_recv_nanosec);
    buffer << htonl(msg.server_send_sec);
    buffer << htonl(msg.server_send_nanosec);

    return buffer.serialize();
}

SyncResponseMsg SyncResponseMsg::deserialize(const std::vector<uint8_t>& data) {
    SyncResponseMsg msg;

    if (data.empty() || data.at(0) != msg.msg_id) {
        throw SerializationError("msg id mismatch ");
    }

    BinIStream buffer(data);
    buffer >> msg.msg_id;
    buffer >> msg.req_id;
    msg.req_id = ntohl(msg.req_id);
    buffer >> msg.server_recv_sec;
    msg.server_recv_sec = ntohl(msg.server_recv_sec);
    buffer >> msg.server_recv_nanosec;
    msg.server_recv_nanosec = ntohl(msg.server_recv_nanosec);
    buffer >> msg.server_send_sec;
    msg.server_send_sec = ntohl(msg.server_send_sec);
    buffer >> msg.server_send_nanosec;
    msg.server_send_nanosec = ntohl(msg.server_send_nanosec);

    return msg;
}

}  // namespace proxy_client
