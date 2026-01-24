#pragma once

#include <arpa/inet.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#define FIXED_DIV 1.0e6

namespace proxy_client {

class SerializationError : public std::exception {
   protected:
    std::string message = "Failed to deserialize type: ";

   public:
    SerializationError(const std::string& msg);

    // Override the what() method to return our message
    const char* what() const throw();
};

class BinIStream {
   public:
    BinIStream(const std::vector<uint8_t>& data);

    template <typename T, typename std::enable_if<std::is_fundamental<T>::value, bool>::type = true>
    BinIStream& operator>>(T& v) {
        iss.read((char*)&v, sizeof(T));
        return *this;
    }

    template <typename T,
              typename std::enable_if<std::is_fundamental<typename T::value_type>::value, bool>::type = true>
    BinIStream& operator>>(T& v) {
        iss.read((char*)v.data(), v.size() * sizeof(typename T::value_type));
        return *this;
    }

    BinIStream& operator>>(double& v);

    BinIStream& operator>>(std::string& v);

   protected:
    std::stringstream iss;
};

class BinOStream {
   public:
    BinOStream();

    template <typename T, typename std::enable_if<std::is_fundamental<T>::value, bool>::type = true>
    BinOStream& operator<<(const T& v) {
        oss.write((char*)&v, sizeof(T));
        return *this;
    }

    template <typename T,
              typename std::enable_if<std::is_fundamental<typename T::value_type>::value, bool>::type = true>
    BinOStream& operator<<(const T& v) {
        oss.write((char*)v.data(), v.size() * sizeof(typename T::value_type));
        return *this;
    }

    BinOStream& operator<<(const double& v);

    BinOStream& operator<<(const std::string& v);

    std::vector<uint8_t> serialize() const;

   protected:
    std::string str() const;

    std::ostringstream oss;
};

}  // namespace proxy_client
