#pragma once

#include <netinet/in.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chassis_msgs/msg/module_states.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vision_msgs/msg/detection2_d.hpp>

#define FIXED_PT_DIV 1000.0

uint64_t htonll(uint64_t value) {
// Check the endianness of the system
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    // Little-endian: Perform byte-swapping
    return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
#else
    // Big-endian: No need to swap
    return value;
#endif
}

namespace proxy_client {

// force tight packing
#pragma pack(push, 1)

struct ProxyOdomMsg {
    // Unique message identifier
    const uint8_t msg_id{ 30u };

    // Timestamp
    int32_t sec{ 0 };
    int32_t nanosec{ 0 };

    // Position data
    int32_t x_pos;
    int32_t y_pos;
    int32_t theta_pos;

    // Position variances for certainty estimation
    int32_t x_var;
    int32_t y_var;
    int32_t theta_var;

    // Velocity data
    int32_t x_dot;
    int32_t y_dot;
    int32_t theta_dot;
};

struct ProxyModuleStatesMsg {
    // Unique message identifier
    const uint8_t msg_id{ 2u };

    // Timestamp
    int32_t sec{ 0 };
    int32_t nanosec{ 0 };

    // Module 1 data
    int32_t ang_pos_1;
    int32_t lin_vel_1;

    // Module 2 data
    int32_t ang_pos_2;
    int32_t lin_vel_2;

    // Module 3 data
    int32_t ang_pos_3;
    int32_t lin_vel_3;

    // Module 4 data
    int32_t ang_pos_4;
    int32_t lin_vel_4;
};

struct ProxyVisionDetection {
    // Unique message identifier
    const uint8_t msg_id{ 10u };

    // Timestamp
    int32_t sec{ 0 };
    int32_t nanosec{ 0 };

    int32_t detection_count;  ///< Number of detections to expect in total
    int32_t detection_idx;    ///< Index of the detection

    uint8_t class_id;  ///< Class identifier #

    int64_t theta_x;  ///< Angle from camera center on x axis
    int64_t theta_y;  ///< Angle from camera center on y axis
};

#pragma pack(pop)

template <typename T>
std::vector<uint8_t> serialize(T* data_struct) {
    std::vector<uint8_t> data(sizeof(T), 0u);
    std::memcpy(data.data(), data_struct, sizeof(T));
    return data;
};

ProxyOdomMsg loadOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    ProxyOdomMsg proxy_msg;
    proxy_msg.sec = htonl(msg->header.stamp.sec);
    proxy_msg.nanosec = htonl(msg->header.stamp.nanosec);

    // convert quaternion position to RPY
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // convert the position
    proxy_msg.x_pos = htonl(static_cast<int32_t>(msg->pose.pose.position.x * FIXED_PT_DIV));
    proxy_msg.y_pos = htonl(static_cast<int32_t>(msg->pose.pose.position.y * FIXED_PT_DIV));
    proxy_msg.theta_pos = htonl(static_cast<int32_t>(yaw * FIXED_PT_DIV));

    // convert the variance data
    proxy_msg.x_var = htonl(static_cast<int32_t>(msg->pose.covariance.at(0) * FIXED_PT_DIV));
    proxy_msg.y_var = htonl(static_cast<int32_t>(msg->pose.covariance.at(7) * FIXED_PT_DIV));
    proxy_msg.theta_var = htonl(static_cast<int32_t>(msg->pose.covariance.at(14) * FIXED_PT_DIV));

    proxy_msg.x_dot = htonl(static_cast<int32_t>(msg->twist.twist.linear.x * FIXED_PT_DIV));
    proxy_msg.y_dot = htonl(static_cast<int32_t>(msg->twist.twist.linear.y * FIXED_PT_DIV));
    proxy_msg.theta_dot = htonl(static_cast<int32_t>(msg->twist.twist.angular.z * FIXED_PT_DIV));

    return proxy_msg;
}

ProxyModuleStatesMsg loadModules(chassis_msgs::msg::ModuleStates::ConstSharedPtr msg) {
    ProxyModuleStatesMsg proxy_msg;
    proxy_msg.sec = htonl(msg->header.stamp.sec);
    proxy_msg.nanosec = htonl(msg->header.stamp.nanosec);

    // load module 1
    proxy_msg.ang_pos_1 = htonl(static_cast<int32_t>(msg->observed_states.at(0).angle_rad * FIXED_PT_DIV));
    proxy_msg.lin_vel_1 = htonl(static_cast<int32_t>(msg->observed_states.at(0).lin_vel * FIXED_PT_DIV));

    // load module 2
    proxy_msg.ang_pos_2 = htonl(static_cast<int32_t>(msg->observed_states.at(1).angle_rad * FIXED_PT_DIV));
    proxy_msg.lin_vel_2 = htonl(static_cast<int32_t>(msg->observed_states.at(1).lin_vel * FIXED_PT_DIV));

    // load module 3
    proxy_msg.ang_pos_3 = htonl(static_cast<int32_t>(msg->observed_states.at(2).angle_rad * FIXED_PT_DIV));
    proxy_msg.lin_vel_3 = htonl(static_cast<int32_t>(msg->observed_states.at(2).lin_vel * FIXED_PT_DIV));

    // load module 4
    proxy_msg.ang_pos_4 = htonl(static_cast<int32_t>(msg->observed_states.at(3).angle_rad * FIXED_PT_DIV));
    proxy_msg.lin_vel_4 = htonl(static_cast<int32_t>(msg->observed_states.at(3).lin_vel * FIXED_PT_DIV));

    return proxy_msg;
}

ProxyVisionDetection loadDetection(const std_msgs::msg::Header& hdr, const vision_msgs::msg::Detection2D& msg,
                                   uint8_t class_id, sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info, int idx,
                                   int count) {
    ProxyVisionDetection proxy_msg;
    proxy_msg.sec = htonl(hdr.stamp.sec);
    proxy_msg.nanosec = htonl(hdr.stamp.nanosec);
    proxy_msg.detection_count = htonl(count);
    proxy_msg.detection_idx = htonl(idx);
    proxy_msg.class_id = htons(class_id);

    // theta = atan((x - cx) / fx)
    double theta_x = (atan((msg.bbox.center.position.x - cam_info->k.at(2)) / cam_info->k.at(0)) / M_PI) * 180.0;
    double theta_y = (atan((msg.bbox.center.position.y - cam_info->k.at(5)) / cam_info->k.at(4)) / M_PI) * 180.0;

    proxy_msg.theta_x = htonll(static_cast<int64_t>(theta_x * 10e6));
    proxy_msg.theta_y = htonll(static_cast<int64_t>(theta_y * 10e6));

    return proxy_msg;
}

ProxyVisionDetection loadEmptyDetections(const std_msgs::msg::Header& hdr) {
    ProxyVisionDetection proxy_msg;
    proxy_msg.sec = htonl(hdr.stamp.sec);
    proxy_msg.nanosec = htonl(hdr.stamp.nanosec);
    proxy_msg.detection_count = htonl(0);

    return proxy_msg;
}

}  // namespace proxy_client
