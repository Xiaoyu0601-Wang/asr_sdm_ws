
#ifndef SVO_FEATURE_ALIGNMENT_H_
#define SVO_FEATURE_ALIGNMENT_H_

#include <svo/global.h>

namespace svo
{


namespace feature_alignment
{

bool align1D(
  const cv::Mat & cur_img,
  const Vector2f & dir,  // direction in which the patch is allowed to move
  uint8_t * ref_patch_with_border, uint8_t * ref_patch, const int n_iter,
  Vector2d & cur_px_estimate, double & h_inv);

bool align2D(
  const cv::Mat & cur_img, uint8_t * ref_patch_with_border, uint8_t * ref_patch, const int n_iter,
  Vector2d & cur_px_estimate, bool no_simd = false);

bool align2D_SSE2(
  const cv::Mat & cur_img, uint8_t * ref_patch_with_border, uint8_t * ref_patch, const int n_iter,
  Vector2d & cur_px_estimate);

bool align2D_NEON(
  const cv::Mat & cur_img, uint8_t * ref_patch_with_border, uint8_t * ref_patch, const int n_iter,
  Vector2d & cur_px_estimate);

}  
}  

#endif  
