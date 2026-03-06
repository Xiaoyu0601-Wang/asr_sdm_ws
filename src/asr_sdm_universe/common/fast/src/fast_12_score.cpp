#include <fast/fast.h>
#include <fast/corner_12.h>
#include <fast/faster_corner_utilities.h>

namespace fast {

static inline int score_corner_12(const fast_byte* p, int stride, int threshold)
{
  int lo = threshold;
  int hi = 255;
  int best = threshold - 1;

  while (lo <= hi) {
    const int mid = (lo + hi) >> 1;
    if (is_corner_12<Greater>(p, stride, mid) || is_corner_12<Less>(p, stride, mid)) {
      best = mid;
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  return best;
}

void fast_corner_score_12(
  const fast_byte* img,
  const int img_stride,
  const std::vector<fast_xy>& corners,
  const int threshold,
  std::vector<int>& scores)
{
  scores.resize(corners.size());
  for (size_t i = 0; i < corners.size(); ++i) {
    const fast_byte* p = img + corners[i].y * img_stride + corners[i].x;
    scores[i] = score_corner_12(p, img_stride, threshold);
  }
}

} // namespace fast
