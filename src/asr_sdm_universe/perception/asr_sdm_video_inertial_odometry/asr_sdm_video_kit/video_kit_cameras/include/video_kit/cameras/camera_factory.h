#pragma once

#include <string>
#include <video_kit/cameras/camera_geometry_base.h>

namespace vk {
namespace cameras {
namespace factory {

CameraGeometryBase::Ptr makePinholeCamera(
    const Eigen::VectorXd& intrinsics, uint32_t width, uint32_t height);

CameraGeometryBase::Ptr loadFromYAML(
    const std::string& filename,
    const std::string& cam_name);

} // namespace factory
} // namespace cameras
} // namespace vk
