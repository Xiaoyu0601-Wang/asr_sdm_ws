#ifndef SVO_IMU_TYPES_H_
#define SVO_IMU_TYPES_H_

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <svo/global.h>

namespace svo
{

/// IMU measurement: gyroscope + accelerometer data at a specific timestamp.
struct ImuMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp_ = 0.0;               // Unix timestamp in seconds
  Eigen::Vector3d angular_velocity_;       // Gyroscope (rad/s)
  Eigen::Vector3d linear_acceleration_;   // Accelerometer (m/s^2)

  ImuMeasurement() = default;
  ImuMeasurement(double t, const Eigen::Vector3d& omega, const Eigen::Vector3d& acc)
    : timestamp_(t), angular_velocity_(omega), linear_acceleration_(acc) {}
};

typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>> ImuMeasurements;

/// IMU noise and calibration parameters.
struct ImuCalibration
{
  double delay_imu_cam = 0.0;
  double max_imu_delta_t = 0.1;
  double saturation_accel_max = 200.0;
  double saturation_omega_max = 20.0;
  double gyro_noise_density = 1.0e-5;
  double acc_noise_density = 1.0e-4;
  double gyro_bias_random_walk_sigma = 1.0e-6;
  double acc_bias_random_walk_sigma = 1.0e-5;
  double gravity_magnitude = 9.81;
  double imu_rate = 200.0;
};

/// IMU initial state (biases, velocity).
struct ImuInitialization
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  double velocity_sigma = 0.1;
  double omega_bias_sigma = 0.01;
  double acc_bias_sigma = 0.1;
};

/// Temporal status of IMU measurements.
enum class IMUTemporalStatus { kStationary, kMoving, kUnkown };

/// Options for the IMU handler.
struct IMUHandlerOptions
{
  bool temporal_stationary_check = false;
  double temporal_window_length_sec_ = 0.5;
  double stationary_acc_sigma_thresh_ = 0.0;
  double stationary_gyr_sigma_thresh_ = 0.0;
};

/// Preintegrated IMU measurement between two camera timestamps.
class PreintegratedImuMeasurement
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegratedImuMeasurement() = default;
  PreintegratedImuMeasurement(const Eigen::Vector3d& omega_bias,
                             const Eigen::Vector3d& acc_bias)
    : omega_bias_(omega_bias), acc_bias_(acc_bias) {}

  void addMeasurement(const ImuMeasurement& m);
  void addMeasurements(const ImuMeasurements& ms);

  Eigen::Vector3d omega_bias_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d delta_t_ij_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_v_ij_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond delta_R_ij_ = Eigen::Quaterniond::Identity();
  double dt_sum_ = 0.0;
  bool last_imu_measurement_set_ = false;
  ImuMeasurement last_imu_measurement;
};

/// Handles IMU data buffering, bias correction, preintegration, and rotation priors.
class ImuHandler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuHandler(const ImuCalibration& imu_calib,
              const ImuInitialization& imu_init,
              const IMUHandlerOptions& options);
  ~ImuHandler();

  bool addImuMeasurement(const ImuMeasurement& m);
  bool getMeasurements(double old_cam_timestamp, double new_cam_timestamp,
                       bool delete_old_measurements, ImuMeasurements& measurements);
  bool getClosestMeasurement(double timestamp, ImuMeasurement& measurement) const;
  bool getRelativeRotationPrior(double old_cam_timestamp, double new_cam_timestamp,
                                bool delete_old_measurements,
                                Eigen::Quaterniond& R_oldimu_newimu);
  bool getInitialAttitude(double timestamp, Eigen::Quaterniond& R_imu_world) const;
  IMUTemporalStatus checkTemporalStatus(const double time_sec);
  void reset();

  const ImuCalibration& imu_calib_;
  const ImuInitialization& imu_init_;
  mutable Eigen::Vector3d acc_bias_;
  mutable Eigen::Vector3d omega_bias_;

private:
  const IMUHandlerOptions options_;
  mutable std::mutex measurements_mut_;
  ImuMeasurements measurements_;
  ImuMeasurements temporal_imu_window_;
  using ulock_t = std::unique_lock<std::mutex>;
};

typedef std::shared_ptr<ImuHandler> ImuHandlerPtr;

}  // namespace svo

#endif  // SVO_IMU_TYPES_H_
