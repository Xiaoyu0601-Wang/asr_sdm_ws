#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace vk {
namespace img_type_conversion
{
int sensorMsgsEncodingToOpenCVType(const std::string& encoding);

std::string openCVTypeToSensorMsgsEncoding(const int opencv_type);
} // namespace img_type_conversion
} // namespace vk
