// Compatibility header for Ceres Solver API changes
// Handles transition from LocalParameterization to Manifold API (Ceres 2.1+)

#pragma once

#include <ceres/ceres.h>
#include <ceres/version.h>

// For Ceres 2.1+, LocalParameterization has been removed and replaced with Manifold
// We provide compatibility definitions here to allow old code to compile
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1

namespace ceres {

// Compatibility base class for LocalParameterization
// This allows old code that inherits from LocalParameterization to compile
// with newer versions of Ceres that use Manifold instead
class LocalParameterization {
 public:
  virtual ~LocalParameterization() = default;
  
  // These are the key methods that LocalParameterization had
  virtual bool Plus(const double* x, const double* delta, 
                    double* x_plus_delta) const = 0;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0;
  virtual int GlobalSize() const = 0;
  virtual int LocalSize() const = 0;
};

}  // namespace ceres

#endif  // CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1

