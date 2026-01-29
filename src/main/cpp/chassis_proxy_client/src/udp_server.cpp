#include "chassis_proxy_client/udp_server.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <system_error>
#include <iostream>

namespace proxy_client {
UdpServer::UdpServer(int port) {
    // open the socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ <= 0) {
        throw std::system_error(errno, std::system_category(), "Failed to open socket for communication");
    }

    memset(&server_addr_cfg_, 0, sizeof(server_addr_cfg_));
    server_addr_cfg_.sin_addr.s_addr = INADDR_ANY;
    server_addr_cfg_.sin_port = htons(port);
    server_addr_cfg_.sin_family = AF_INET;

    // bind the local port
    if (bind(socket_fd_, (const struct sockaddr *)&server_addr_cfg_, sizeof(server_addr_cfg_)) > 0) {
        throw std::system_error(errno, std::system_category(), "Failed to bind local socket for communication");
    }

    // make socket non blocking if blocking
    int socket_flags = fcntl(socket_fd_, F_GETFL);
    if (!(socket_flags & O_NONBLOCK)) {
        if (fcntl(socket_fd_, F_SETFL, socket_flags | O_NONBLOCK) < 0) {
            throw std::system_error(errno, std::system_category(), "unable to set nonblocking socket");
        }
    }
}

int UdpServer::processPackets() {
    // get the data
    std::vector<uint8_t> data;
    data.resize(1024);

    struct sockaddr_in cliaddr;
    memset(&cliaddr, 0, sizeof(cliaddr));
    unsigned int cli_len = sizeof(cliaddr);

    int received = recvfrom(socket_fd_, data.data(), data.size(), MSG_WAITALL, (struct sockaddr *)&cliaddr, &cli_len);

    // return 0 if timeout
    if (received < 0 && errno == EWOULDBLOCK) {
        return 0;
    } else if (received < 0) {
        throw std::system_error(errno, std::system_category(), "socket read failed");
    }

    // Pair down the size of the vector to what we received
    data.resize(received);

    // fire the correct callback
    sub_map_t::iterator it = sub_map_.find(data.at(0));
    if(it != sub_map_.end()){
        it->second->invokeCallback(data);
    }

    return 1;
}

}  // namespace proxy_client
