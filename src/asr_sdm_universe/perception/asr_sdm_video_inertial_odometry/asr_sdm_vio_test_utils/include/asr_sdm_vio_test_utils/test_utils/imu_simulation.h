#pragma once

#include <asr_sdm_vio_common/common/types.h>
#include <asr_sdm_vio_common/common/imu_calibration.h>

namespace svo {
namespace simulation {

void simulateBiases(
    const size_t n_measurements,
    const double dt,
    const ImuCalibration& imu_calib,
    const Eigen::Vector3d& bias_gyr_init,
    const Eigen::Vector3d& bias_acc_init,
    Eigen::Matrix3Xd* bias_gyr,
    Eigen::Matrix3Xd* bias_acc);


} // namespace simulation
} // namespace svo
