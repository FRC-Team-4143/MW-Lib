#pragma once

#include <netinet/in.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace proxy_client {
class UdpClient {
   public:
    UdpClient(std::string_view server_hostname, int port);

    template <typename T>
    void sendMsg(const T& msg) {
        sendPacket(T::serialize(msg));
    }

   protected:
    void sendPacket(const std::vector<uint8_t>& byte_buffer);

   private:
    // Connection Info
    int server_port_{ 30000 };                 ///< Port on the server to send packets to
    std::string server_addr_{ "10.41.43.2" };  ///< hostname of the server to be sent packets

    int socket_fd_ = -1;  ///< file descriptor for the socket
    struct sockaddr_in server_addr_cfg_;
};
}  // namespace proxy_client
