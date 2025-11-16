#pragma once

#include <vector>
#include <memory>

#include <svo/common/types.h>
#include <svo/common/camera.h> // For CameraBundlePtr
#include <svo/direct/feature_detection.h> // For AbstractDetector
#include <svo/direct/feature_detection_types.h>
#include <asr_sdm_vio_tracker/tracker/feature_tracking_types.h>

namespace svo {

class FeatureTracker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureTracker(
      const FeatureTrackerOptions& options,
      const DetectorOptions& detector_options,
      const CameraBundlePtr& cams);

  virtual ~FeatureTracker() = default;

  void trackAndDetect(const FrameBundlePtr& nframe_kp1);

  const FeatureTracks& getActiveTracks(size_t frame_index) const;

  size_t getTotalActiveTracks() const;

  void getNumTrackedAndDisparityPerFrame(
      double pivot_ratio,
      std::vector<size_t>* num_tracked,
      std::vector<double>* disparity) const;

  FrameBundlePtr getOldestFrameInTrack(size_t frame_index) const;

  void reset();

public:
  size_t trackFrameBundle(const FrameBundlePtr& nframe_kp1);

  size_t initializeNewTracks(const FrameBundlePtr& nframe);

  void resetActiveTracks();

private:

  void resetTerminatedTracks();

  FeatureTrackerOptions options_;
  size_t bundle_size_;
  std::vector<FeatureTracks> active_tracks_;
  std::vector<FeatureTracks> terminated_tracks_;
  std::vector<AbstractDetector::Ptr> detectors_;
};

} // namespace svo
