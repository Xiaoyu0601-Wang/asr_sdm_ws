#include <glog/logging.h>

#include <vikit/homography.h>
#include <vikit/math_utils.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

namespace vk {

HomographyResult estimateHomography(
  const Bearings& f_cur,
  const Bearings& f_ref,
  const double focal_length,
  const double reproj_error_thresh,
  const size_t min_num_inliers)
{
  CHECK_EQ(f_cur.cols(), f_ref.cols());
  const size_t N = static_cast<size_t>(f_cur.cols());
  const double thresh = reproj_error_thresh / focal_length;

  // compute homography using RANSAC
  std::vector<cv::Point2f> ref_pts(N), cur_pts(N);
  for (size_t i = 0; i < N; ++i) {
    const Eigen::Vector2d uv_ref(vk::project2(f_ref.col(static_cast<int>(i))));
    const Eigen::Vector2d uv_cur(vk::project2(f_cur.col(static_cast<int>(i))));
    ref_pts[i] = cv::Point2f(static_cast<float>(uv_ref[0]), static_cast<float>(uv_ref[1]));
    cur_pts[i] = cv::Point2f(static_cast<float>(uv_cur[0]), static_cast<float>(uv_cur[1]));
  }

  const cv::Mat H = cv::findHomography(ref_pts, cur_pts, cv::RANSAC, thresh);
  if (H.empty()) {
    return HomographyResult();
  }

  Eigen::Matrix3d H_cur_ref;
  H_cur_ref << H.at<double>(0, 0), H.at<double>(0, 1), H.at<double>(0, 2),
    H.at<double>(1, 0), H.at<double>(1, 1), H.at<double>(1, 2),
    H.at<double>(2, 0), H.at<double>(2, 1), H.at<double>(2, 2);

  // compute number of inliers
  std::vector<bool> inliers(N);
  size_t n_inliers = 0;
  for (size_t i = 0; i < N; ++i) {
    const Eigen::Vector2d uv_cur = vk::project2(H_cur_ref * f_ref.col(static_cast<int>(i)));
    const Eigen::Vector2d e = vk::project2(f_cur.col(static_cast<int>(i))) - uv_cur;
    inliers[i] = (e.norm() < thresh);
    n_inliers += inliers[i];
  }

  VLOG(100) << "Homography has " << n_inliers << " inliers";
  if (n_inliers < min_num_inliers) {
    return HomographyResult();
  }

  // decompose homography
  const cv::Matx33d K(1, 0, 0, 0, 1, 0, 0, 0, 1);
  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  std::vector<cv::Mat> normals;
  cv::decomposeHomographyMat(H, K, rotations, translations, normals);
  CHECK_EQ(rotations.size(), 4u);

  std::vector<HomographyResult> decomp;
  decomp.reserve(4);
  for (size_t i = 0; i < 4; ++i) {
    HomographyResult d;

    const cv::Mat& t = translations[i];
    d.t_cur_ref = Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));

    const cv::Mat& n = normals[i];
    d.n_cur = Eigen::Vector3d(n.at<double>(0), n.at<double>(1), n.at<double>(2));

    const cv::Mat& R = rotations[i];
    d.R_cur_ref << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    decomp.push_back(d);
  }

  // check plane in front of camera
  for (HomographyResult& D : decomp) {
    D.score = 0.0;
    for (size_t i = 0; i < N; ++i) {
      if (!inliers[i]) {
        continue;
      }
      const double test = f_cur.col(static_cast<int>(i)).dot(D.n_cur);
      if (test > 0.0) {
        D.score += 1.0;
      }
    }
  }

  std::sort(
    decomp.begin(), decomp.end(),
    [&](const HomographyResult& lhs, const HomographyResult& rhs) { return lhs.score > rhs.score; });

  decomp.resize(2);

  if (decomp[1].score / decomp[0].score < 0.9) {
    decomp.erase(decomp.begin() + 1);
  } else {
    const double thresh_squared = thresh * thresh * 4.0;
    for (HomographyResult& D : decomp) {
      D.score = 0.0;
      const Eigen::Matrix3d E_cur_ref = D.R_cur_ref * vk::sqew(D.t_cur_ref);
      for (size_t i = 0; i < N; ++i) {
        const double d = vk::sampsonusError(vk::project2(f_cur.col(static_cast<int>(i))), E_cur_ref, vk::project2(f_ref.col(static_cast<int>(i))));
        D.score += std::min(d, thresh_squared);
      }
    }

    if (decomp[0].score < decomp[1].score) {
      decomp.erase(decomp.begin() + 1);
    } else {
      decomp.erase(decomp.begin());
    }
  }

  decomp[0].score = static_cast<double>(n_inliers);
  return decomp[0];
}

} // namespace vk

