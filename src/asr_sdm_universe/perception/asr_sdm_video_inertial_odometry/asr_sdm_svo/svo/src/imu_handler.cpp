#include <numeric>
#include <svo/global.h>
#include <svo/imu_types.h>
#include <vikit/math_utils.h>

namespace
{

double stdVec(const std::vector<double>& v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / v.size());
}

}  // namespace

namespace svo
{

void PreintegratedImuMeasurement::addMeasurement(const ImuMeasurement& m)
{
  if (last_imu_measurement_set_)
  {
    const double dt = m.timestamp_ - last_imu_measurement.timestamp_;
    if (dt <= 0.0 || dt > 0.5) return;  // Reject non-positive or unreasonably large dt

    const Eigen::Vector3d a = last_imu_measurement.linear_acceleration_ - acc_bias_;
    const Eigen::Vector3d w = last_imu_measurement.angular_velocity_ - omega_bias_;

    // Saturate angular velocity and acceleration to physical limits
    const double omega_norm = w.norm();
    const double acc_norm = a.norm();
    Eigen::Vector3d w_corr = w;
    Eigen::Vector3d a_corr = a;
    if (omega_norm > saturation_omega_max_)
    {
      w_corr = w.normalized() * saturation_omega_max_;
    }
    if (acc_norm > saturation_accel_max_)
    {
      a_corr = a.normalized() * saturation_accel_max_;
    }

    // Second-order integration:
    delta_t_ij_ += delta_v_ij_ * dt + (delta_R_ij_.toRotationMatrix() * a_corr) * dt * dt * 0.5;
    delta_v_ij_ += delta_R_ij_.toRotationMatrix() * a_corr * dt;
    const double wn = w_corr.norm();
    const Eigen::Quaterniond R_incr = Eigen::Quaterniond(
        Eigen::AngleAxisd(wn * dt, wn > 1e-8 ? w_corr.normalized() : Eigen::Vector3d::UnitX()));
    delta_R_ij_ = delta_R_ij_ * R_incr;
    dt_sum_ += dt;
  }
  last_imu_measurement_set_ = true;
  last_imu_measurement = m;
}

void PreintegratedImuMeasurement::addMeasurements(const ImuMeasurements& ms)
{
  for (const ImuMeasurement& m : ms)
    addMeasurement(m);
}

ImuHandler::ImuHandler(
    const ImuCalibration& imu_calib,
    const ImuInitialization& imu_init,
    const IMUHandlerOptions& options)
  : options_(options),
    imu_calib_(imu_calib),
    imu_init_(imu_init),
    acc_bias_(imu_init.acc_bias),
    omega_bias_(imu_init.omega_bias)
{}

ImuHandler::~ImuHandler()
{}

bool ImuHandler::getMeasurements(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    const bool delete_old_measurements,
    ImuMeasurements& measurements)
{
  if (new_cam_timestamp <= old_cam_timestamp)
    return false;

  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurements available.");
    return false;
  }

  // Subtract camera delay to get corrected timestamps.
  const double t1 = old_cam_timestamp - imu_calib_.delay_imu_cam;
  const double t2 = new_cam_timestamp - imu_calib_.delay_imu_cam;

  ImuMeasurements::iterator it1 = measurements_.end();
  ImuMeasurements::iterator it2 = measurements_.end();
  bool it2_set = false;

  for (ImuMeasurements::iterator it = measurements_.begin();
       it != measurements_.end(); ++it)
  {
    if (!it2_set && it->timestamp_ < t2)
    {
      it2 = it;
      it2_set = true;
    }
    if (it->timestamp_ <= t1)
    {
      it1 = it;
      break;
    }
  }

  if (!it2_set || it1 == measurements_.end())
  {
    if (measurements_.empty())
    {
      SVO_WARN_STREAM("ImuHandler: IMU buffer is EMPTY, cannot find any measurements for t1="
          << t1 << " t2=" << t2);
    }
    else
    {
      SVO_WARN_STREAM("ImuHandler: Not enough IMU measurements for t1=" << t1
          << " t2=" << t2 << ". Buffer size=" << measurements_.size()
          << ". Buffer front(ts)=" << measurements_.front().timestamp_
          << " Buffer back(ts)=" << measurements_.back().timestamp_);
    }
    return false;
  }

  // it1 = last measurement <= t1 (older)
  // it2 = last measurement < t2 (newer)
  // Require at least 2 measurements to integrate
  if (it2 == it1 || std::distance(it2, it1) < 1)
  {
    SVO_WARN_STREAM("ImuHandler: Not enough IMU measurements for the time interval.");
    return false;
  }

  if (t2 - it2->timestamp_ > imu_calib_.max_imu_delta_t)
  {
    SVO_WARN_STREAM("ImuHandler: Newest IMU measurement is too old. Gap: "
                    << (t2 - it2->timestamp_) * 1000.0 << " ms.");
    return false;
  }

  // Copy measurements from it2 (newer) to it1 (older).
  ++it1;
  measurements.insert(measurements.begin(), it2, it1);
  measurements.back().timestamp_ = t1;

  if (delete_old_measurements)
    measurements_.erase(it1, measurements_.end());

  return true;
}

bool ImuHandler::getClosestMeasurement(
    const double timestamp,
    ImuMeasurement& measurement) const
{
  ulock_t lock(measurements_mut_);
  if (measurements_.empty())
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurements available.");
    return false;
  }

  double dt_best = std::numeric_limits<double>::max();
  double img_ts_corrected = timestamp - imu_calib_.delay_imu_cam;
  for (const ImuMeasurement& m : measurements_)
  {
    const double dt = std::abs(m.timestamp_ - img_ts_corrected);
    if (dt < dt_best)
    {
      dt_best = dt;
      measurement = m;
    }
  }

  // Use 10x max_imu_delta_t as the initial-attitude lookup threshold
  // (much looser than the preintegration threshold, since we just need one nearby measurement)
  const double max_lookup_dt = std::max(imu_calib_.max_imu_delta_t * 10.0, 1.0);
  if (dt_best > max_lookup_dt)
  {
    SVO_WARN_STREAM("ImuHandler: No IMU measurement found within threshold. "
                    "Closest: " << dt_best * 1000.0 << " ms > " << max_lookup_dt * 1000.0 << " ms.");
    return false;
  }
  return true;
}

bool ImuHandler::getRelativeRotationPrior(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    bool delete_old_measurements,
    Eigen::Quaterniond& R_oldimu_newimu)
{
  ImuMeasurements measurements;
  if (!getMeasurements(old_cam_timestamp, new_cam_timestamp,
                       delete_old_measurements, measurements))
    return false;

  // Integrate angular velocity from t1 to t2 with bias correction and saturation.
  R_oldimu_newimu.setIdentity();
  ImuMeasurements::reverse_iterator it = measurements.rbegin();
  ImuMeasurements::reverse_iterator it_plus = measurements.rbegin();
  ++it_plus;

  for (; it != measurements.rend(); ++it, ++it_plus)
  {
    double dt = 0.0;
    if (it_plus == measurements.rend())
      dt = new_cam_timestamp - imu_calib_.delay_imu_cam - it->timestamp_;
    else
      dt = it_plus->timestamp_ - it->timestamp_;

    if (dt <= 0.0 || dt > 0.5)
    {
      SVO_WARN_STREAM("ImuHandler: Rejected dt=" << dt << " in rotation prior integration.");
      continue;
    }

    const Eigen::Vector3d omega_raw = it->angular_velocity_ - omega_bias_;
    // Apply gyroscope saturation limit
    Eigen::Vector3d omega_corrected = omega_raw;
    const double omega_norm = omega_corrected.norm();
    if (omega_norm > imu_calib_.saturation_omega_max)
    {
      omega_corrected = omega_corrected.normalized() * imu_calib_.saturation_omega_max;
      SVO_WARN_STREAM("ImuHandler: Gyro saturation detected. omega_norm="
                      << omega_norm << " > " << imu_calib_.saturation_omega_max);
    }

    const double theta = omega_corrected.norm() * dt;
    if (theta > 1e-8)
    {
      Eigen::Quaterniond R_incr(Eigen::AngleAxisd(
          theta, omega_corrected.normalized()));
      R_oldimu_newimu = R_oldimu_newimu * R_incr;
    }
  }
  return true;
}

bool ImuHandler::addImuMeasurement(const ImuMeasurement& m)
{
  ulock_t lock(measurements_mut_);
  measurements_.push_front(m);
  if (options_.temporal_stationary_check)
    temporal_imu_window_.push_front(m);
  return true;
}

bool ImuHandler::getInitialAttitude(
    double timestamp,
    Eigen::Quaterniond& R_imu_world) const
{
  ImuMeasurement m;
  if (!getClosestMeasurement(timestamp, m))
  {
    SVO_WARN_STREAM("ImuHandler: Could not get initial attitude. No IMU measurements.");
    return false;
  }

  // Align world Z-axis with measured gravity direction.
  const Eigen::Vector3d& g = m.linear_acceleration_;
  const Eigen::Vector3d z = g.normalized();

  Eigen::Vector3d p(1, 0, 0);
  Eigen::Vector3d p_alt(0, 1, 0);
  if (std::fabs(z.dot(p)) > std::fabs(z.dot(p_alt)))
    p = p_alt;

  Eigen::Vector3d y = z.cross(p);
  y.normalize();
  const Eigen::Vector3d x = y.cross(z);

  Eigen::Matrix3d C_imu_world;
  C_imu_world.col(0) = x;
  C_imu_world.col(1) = y;
  C_imu_world.col(2) = z;

  R_imu_world = Eigen::Quaterniond(C_imu_world);
  SVO_DEBUG_STREAM("ImuHandler: Initial attitude from gravity. g=" << g.transpose());
  return true;
}

void ImuHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
  temporal_imu_window_.clear();
}

IMUTemporalStatus ImuHandler::checkTemporalStatus(const double time_sec)
{
  if (!options_.temporal_stationary_check)
  {
    SVO_WARN_STREAM("ImuHandler: Stationary check is disabled.");
    return IMUTemporalStatus::kMoving;
  }

  IMUTemporalStatus res = IMUTemporalStatus::kMoving;

  ulock_t lock(measurements_mut_);
  if (temporal_imu_window_.empty() ||
      temporal_imu_window_.front().timestamp_ < time_sec)
    return IMUTemporalStatus::kUnkown;

  // Find indices: start = first measurement older than time_sec, end = oldest in window
  ssize_t start_idx = -1;
  ssize_t end_idx = -1;
  for (ssize_t idx = 0; idx < static_cast<ssize_t>(temporal_imu_window_.size()); ++idx)
  {
    if (temporal_imu_window_[static_cast<size_t>(idx)].timestamp_ >= time_sec)
    {
      end_idx = idx;
      break;
    }
  }
  if (end_idx == -1)
    return IMUTemporalStatus::kUnkown;

  // Search for start of a full temporal window
  const double window_end_ts = temporal_imu_window_[static_cast<size_t>(end_idx)].timestamp_;
  for (ssize_t idx = end_idx - 1; idx >= 0; --idx)
  {
    if (window_end_ts - temporal_imu_window_[static_cast<size_t>(idx)].timestamp_ >
        options_.temporal_window_length_sec_)
    {
      start_idx = idx + 1;
      break;
    }
  }
  if (start_idx == -1 || end_idx < start_idx)
    return IMUTemporalStatus::kUnkown;

  const size_t n = static_cast<size_t>(end_idx - start_idx + 1);
  std::vector<double> gyr_x(n), gyr_y(n), gyr_z(n), acc_x(n), acc_y(n), acc_z(n);
  for (ssize_t midx = start_idx; midx <= end_idx; ++midx)
  {
    const ImuMeasurement& m = temporal_imu_window_[static_cast<size_t>(midx)];
    const size_t off = static_cast<size_t>(midx - start_idx);
    gyr_x[off] = m.angular_velocity_.x();
    gyr_y[off] = m.angular_velocity_.y();
    gyr_z[off] = m.angular_velocity_.z();
    acc_x[off] = m.linear_acceleration_.x();
    acc_y[off] = m.linear_acceleration_.y();
    acc_z[off] = m.linear_acceleration_.z();
  }

  const double sqrt_dt = std::sqrt(1.0 / imu_calib_.imu_rate);
  std::array<double, 3> gyr_std = {stdVec(gyr_x) * sqrt_dt,
                                     stdVec(gyr_y) * sqrt_dt,
                                     stdVec(gyr_z) * sqrt_dt};
  std::array<double, 3> acc_std = {stdVec(acc_x) * sqrt_dt,
                                     stdVec(acc_y) * sqrt_dt,
                                     stdVec(acc_z) * sqrt_dt};

  bool stationary = true;
  for (size_t idx = 0; idx < 3; ++idx)
  {
    stationary &= (gyr_std[idx] < options_.stationary_gyr_sigma_thresh_);
    stationary &= (acc_std[idx] < options_.stationary_acc_sigma_thresh_);
  }

  // Prune: keep only measurements newer than the start of the window
  temporal_imu_window_.erase(
      temporal_imu_window_.begin(), temporal_imu_window_.begin() + static_cast<ssize_t>(start_idx));

  return stationary ? IMUTemporalStatus::kStationary : IMUTemporalStatus::kMoving;
}

}  // namespace svo
