#pragma once
#include <asr_sdm_vio/ceres_backend_interface.hpp>

#include <memory>

namespace vio_ros2_backend
{

inline std::shared_ptr<svo::CeresBackendInterface> makeCeresBackend(
  const std::shared_ptr<svo::ImuHandler> & imu_handler)
{
  using namespace svo;
  CeresBackendInterfaceOptions iface_opts;  // defaults
  CeresBackendOptions opt_opts;             // defaults
  auto be = std::allocate_shared<CeresBackendInterface>(Eigen::aligned_allocator<CeresBackendInterface>{}, iface_opts, opt_opts, nullptr);
  if (imu_handler) {
    be->setImu(imu_handler);
  }
  // Optional: run optimization loop thread
  be->startThread();
  return be;
}

}  // namespace vio_ros2_backend
