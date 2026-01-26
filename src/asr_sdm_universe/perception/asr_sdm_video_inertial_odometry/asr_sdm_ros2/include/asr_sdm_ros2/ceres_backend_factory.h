#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <asr_sdm_vio/ceres_backend_interface.hpp>

namespace svo {

namespace ceres_backend_factory {

std::shared_ptr<CeresBackendInterface> makeBackend(
    const rclcpp::Node::SharedPtr& node, const CameraBundlePtr& camera_bundle);

} // namespace ceres_backend_factory
} // namespace svo
