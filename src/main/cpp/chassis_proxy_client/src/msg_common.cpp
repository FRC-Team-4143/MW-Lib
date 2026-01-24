#include "chassis_proxy_client/messages/msg_common.hpp"

namespace proxy_client {

SerializationError::SerializationError(const std::string& msg) { message += msg; }

const char* SerializationError::what() const throw() { return message.c_str(); }

BinIStream::BinIStream(const std::vector<uint8_t>& data) {
    iss.write(reinterpret_cast<char*>(const_cast<uint8_t*>(data.data())), data.size());
}

BinIStream& BinIStream::operator>>(double& v) {
    int64_t tmp;
    iss.read((char*)&tmp, sizeof(tmp));
    tmp = be64toh(tmp);
    v = tmp / FIXED_DIV;
    return *this;
}

BinIStream& BinIStream::operator>>(std::string& v){
    uint16_t str_size;
    (*this) >> str_size;
    str_size = ntohs(str_size);
    char c;
    for(uint16_t i = 0; i < str_size; i++){
        (*this) >> c;
        v += c;
    }
    return *this;
}

BinOStream::BinOStream() : oss() {}

BinOStream& BinOStream::operator<<(const double & v) {
    int64_t tmp = htobe64(static_cast<int64_t>(v * FIXED_DIV));
    oss.write((char*)&tmp, sizeof(tmp));
    return *this;
}

BinOStream& BinOStream::operator<<(const std::string& v){
    uint16_t str_size = htons(v.size());
    oss.write((char*)&str_size, sizeof(uint16_t));
    for(char c : v){
        oss.write((char*)&c, sizeof(char));
    }
    return *this;
}

std::string BinOStream::str() const { return oss.str(); }

std::vector<uint8_t> BinOStream::serialize() const {
    std::vector<uint8_t> vec;
    for (uint8_t c : str()) {
        vec.push_back(c);
    }

    return vec;
}

}  // namespace proxy_client
