#pragma once

#include <netinet/in.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include "chassis_proxy_client/proxy_subscription.hpp"

namespace proxy_client {

using message_map_t = std::unordered_map<uint8_t, std::deque<std::vector<uint8_t>>>;
using sub_map_t = std::unordered_map<uint8_t, std::unique_ptr<ProxySubscriptionBase>>;

class UdpServer {
   public:
    UdpServer(int port);

    int processPackets();

    template <typename T>
    void registerType(std::function<void(T result)> callback) {
        sub_map_.emplace(T().msg_id, new ProxySubscription<T>(callback));
    }

   private:
    sub_map_t sub_map_;

    int socket_fd_ = -1;                  ///< file descriptor for the socket
    struct sockaddr_in server_addr_cfg_;  ///< Local bind address config
};
}  // namespace proxy_client
