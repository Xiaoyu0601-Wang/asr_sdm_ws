#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/StdVector>

#include <vikit/math_utils.h>

namespace vk
{

using namespace Eigen;
using namespace std;

struct HomographyDecomposition
{
  Vector3d t;
  Matrix3d R;
  double d;
  Vector3d n;

  // Resolved  Composition
  Sophus::SE3d T;  //!< second from first
  int score;
};

class Homography
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Homography(
    const vector<Vector2d, aligned_allocator<Vector2d> > & _fts1,
    const vector<Vector2d, aligned_allocator<Vector2d> > & _fts2, double _error_multiplier2,
    double _thresh_in_px);

  void calcFromPlaneParams(const Vector3d & normal, const Vector3d & point_on_plane);

  void calcFromMatches();

  size_t computeMatchesInliers();

  bool computeSE3fromMatches();

  bool decompose();

  void findBestDecomposition();

  double thresh;
  double error_multiplier2;
  const vector<Vector2d, aligned_allocator<Vector2d> > &
    fts_c1;  //!< Features on first image on unit plane
  const vector<Vector2d, aligned_allocator<Vector2d> > &
    fts_c2;  //!< Features on second image on unit plane
  vector<bool> inliers;
  Sophus::SE3d T_c2_from_c1;  //!< Relative translation and rotation of two images
  Matrix3d H_c2_from_c1;      //!< Homography
  vector<HomographyDecomposition> decompositions;
};

// --------------------------------------------------------------------------------------
// Compatibility layer for SVO (rpg_svo_pro_open) style API.
//
// rpg_svo_pro_open expects:
// - vk::Homography to be a simple struct with members: t_cur_ref, R_cur_ref, n_cur, score
// - vk::estimateHomography(...) returning that struct
//
// This workspace contains a different Homography class implementation. To avoid rewriting
// SVO code, we provide a small adapter type and function.
// --------------------------------------------------------------------------------------

using BearingVector = Eigen::Vector3d;
using Bearings = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor>;

struct HomographyResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d t_cur_ref;
  Eigen::Matrix3d R_cur_ref;
  Eigen::Vector3d n_cur;
  double score;

  HomographyResult()
  : t_cur_ref(Eigen::Vector3d::Zero()),
    R_cur_ref(Eigen::Matrix3d::Identity()),
    n_cur(Eigen::Vector3d::Zero()),
    score(0.0)
  {}
};

/// Estimates Homography from corresponding feature bearing vectors.
/// Score of returned homography is set to the number of inliers.
HomographyResult estimateHomography(
  const Bearings& f_cur,
  const Bearings& f_ref,
  const double focal_length,
  const double reproj_error_thresh,
  const size_t min_num_inliers);

// Backward-compat alias: SVO code expects vk::Homography with these fields.
// This repository already has a class named vk::Homography; to avoid breaking it,
// SVO code should use vk::HomographyResult.

} /* end namespace vk */

#endif /* HOMOGRAPHY_H_ */
