/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 * Modified for ROS 2
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline
bool hasParam(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  return node->has_parameter(name);
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& defaultValue)
{
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(v).c_str());
    return v;
  }
  else
    RCLCPP_WARN(node->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), std::to_string(defaultValue).c_str());
  return defaultValue;
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(v).c_str());
    return v;
  }
  else
    RCLCPP_ERROR(node->get_logger(), "Cannot find value for parameter: %s", name.c_str());
  return T();
}

template<typename T>
T param(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& defaultValue,
        const bool silent=false)
{
  if(node->has_parameter(name))
  {
    T v;
    node->get_parameter_or(name, v, defaultValue);
    if (!silent)
    {
      RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(v).c_str());
    }
    return v;
  }
  if (!silent)
  {
    RCLCPP_WARN(node->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), std::to_string(defaultValue).c_str());
  }
  return defaultValue;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
