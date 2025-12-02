#pragma once

#include <ceres/ceres.h>
#include <ceres/version.h>
#include "asr_sdm_vio/ceres_compat.hpp"
#include "asr_sdm_vio/ceres_backend/local_parameterization_additional_interfaces.hpp"

namespace svo {
namespace ceres_backend {

// Adapter to wrap a ceres::LocalParameterization into a ceres::Manifold
class LocalParameterizationManifoldAdapter : public ceres::Manifold {
 public:
  LocalParameterizationManifoldAdapter(const ceres::LocalParameterization* local,
                                       const LocalParamizationAdditionalInterfaces* iface)
      : local_(local), iface_(iface) {}

  ~LocalParameterizationManifoldAdapter() override = default;

  int AmbientSize() const override { return local_->GlobalSize(); }
  int TangentSize() const override { return local_->LocalSize(); }

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
    return local_->Plus(x, delta, x_plus_delta);
  }

  bool Minus(const double* y, const double* x, double* y_minus_x) const override {
    return iface_->Minus(x, y, y_minus_x);
  }

  bool PlusJacobian(const double* x, double* jacobian) const override {
    return local_->ComputeJacobian(x, jacobian);
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    return iface_->ComputeLiftJacobian(x, jacobian);
  }

 private:
  const ceres::LocalParameterization* local_;
  const LocalParamizationAdditionalInterfaces* iface_;
};

}  // namespace ceres_backend
}  // namespace svo

