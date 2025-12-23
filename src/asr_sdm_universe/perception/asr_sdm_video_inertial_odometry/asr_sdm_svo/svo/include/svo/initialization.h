#ifndef SVO_INITIALIZATION_H
#define SVO_INITIALIZATION_H

#include <svo/global.h>

namespace svo
{

class FrameHandlerMono;

/// Bootstrapping the map from the first two views.
namespace initialization
{

enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };

/// Tracks features using Lucas-Kanade tracker and then estimates a homography.
class KltHomographyInit
{
  friend class svo::FrameHandlerMono;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr frame_ref_;
  KltHomographyInit() {};
  ~KltHomographyInit() {};
  InitResult addFirstFrame(FramePtr frame_ref);
  InitResult addSecondFrame(FramePtr frame_ref);
  void reset();

protected:
  std::vector<cv::Point2f> px_ref_;  //!< keypoints to be tracked in reference frame.
  std::vector<cv::Point2f> px_cur_;  //!< tracked keypoints in current frame.
  std::vector<Vector3d>
    f_ref_;  //!< bearing vectors corresponding to the keypoints in the reference image.
  std::vector<Vector3d>
    f_cur_;  //!< bearing vectors corresponding to the keypoints in the current image.
  std::vector<double> disparities_;   //!< disparity between first and second frame.
  std::vector<int> inliers_;          //!< inliers after the geometric check (e.g., Homography).
  std::vector<Vector3d> xyz_in_cur_;  //!< 3D points computed during the geometric check.
  Sophus::SE3d T_cur_from_ref_;       //!< computed transformation between the first two frames.
};

/// Detect Fast corners in the image.
void detectFeatures(
  FramePtr frame, std::vector<cv::Point2f> & px_vec, std::vector<Vector3d> & f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
void trackKlt(
  FramePtr frame_ref, FramePtr frame_cur, std::vector<cv::Point2f> & px_ref,
  std::vector<cv::Point2f> & px_cur, std::vector<Vector3d> & f_ref, std::vector<Vector3d> & f_cur,
  std::vector<double> & disparities);

void computeHomography(
  const std::vector<Vector3d> & f_ref, const std::vector<Vector3d> & f_cur, double focal_length,
  double reprojection_threshold, std::vector<int> & inliers, std::vector<Vector3d> & xyz_in_cur,
  Sophus::SE3d & T_cur_from_ref);

}  // namespace initialization
}  // namespace svo

#endif  // SVO_INITIALIZATION_H
