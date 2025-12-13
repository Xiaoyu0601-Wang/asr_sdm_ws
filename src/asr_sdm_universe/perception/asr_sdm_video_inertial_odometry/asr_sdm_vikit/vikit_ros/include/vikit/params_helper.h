/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 * Ported to ROS2
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline
bool hasParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  return node->has_parameter(name);
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name, const T& defaultValue)
{
  if(!node->has_parameter(name)) {
    node->declare_parameter(name, defaultValue);
  }
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}

template<typename T>
T getParam(rclcpp::Node::SharedPtr node, const std::string& name)
{
  if(!node->has_parameter(name)) {
    node->declare_parameter<T>(name);
  }
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: " << name);
  }
  return T();
}

// Overloads for raw node pointer
inline
bool hasParam(rclcpp::Node* node, const std::string& name)
{
  return node->has_parameter(name);
}

template<typename T>
T getParam(rclcpp::Node* node, const std::string& name, const T& defaultValue)
{
  if(!node->has_parameter(name)) {
    node->declare_parameter(name, defaultValue);
  }
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  }
  return defaultValue;
}

template<typename T>
T getParam(rclcpp::Node* node, const std::string& name)
{
  if(!node->has_parameter(name)) {
    node->declare_parameter<T>(name);
  }
  T v;
  if(node->get_parameter(name, v))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: " << name);
  }
  return T();
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
