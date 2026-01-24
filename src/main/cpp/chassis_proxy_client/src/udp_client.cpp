#include <arpa/inet.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chassis_proxy_client/udp_client.hpp>
#include <system_error>

namespace proxy_client {

UdpClient::UdpClient(std::string_view server_hostname, int port) {
    // Translate the hostname to an address
    server_addr_ = server_hostname;
    server_port_ = port;

    // open the socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ <= 0) {
        throw std::system_error(errno, std::system_category(), "Failed to open socket for communication");
    }

    // connect to the server
    memset(&server_addr_cfg_, 0, sizeof(server_addr_cfg_));
    server_addr_cfg_.sin_addr.s_addr = inet_addr(server_addr_.c_str());
    server_addr_cfg_.sin_port = htons(server_port_);
    server_addr_cfg_.sin_family = AF_INET;
}

void UdpClient::sendPacket(const std::vector<uint8_t> &byte_buffer) {
    // request to send datagram
    // no need to specify server address in sendto
    // connect stores the peers IP and port
    int err = sendto(socket_fd_, byte_buffer.data(), byte_buffer.size(), 0, (struct sockaddr *)&server_addr_cfg_,
                     sizeof(server_addr_cfg_));
    if (err < 0) {
        throw std::system_error(errno, std::system_category(), "Failed to send message to server");
    }
}

}  // namespace proxy_client
