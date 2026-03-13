#include <fast/fast.h>
#include <fast/corner_7.h>
#include <fast/faster_corner_utilities.h>

namespace fast {

void fast_corner_detect_7(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners)
{
  const int w = widthStep;
  for (int y = 3; y < imgHeight - 3; ++y) {
    for (int x = 3; x < imgWidth - 3; ++x) {
      const unsigned char* p = img + y * w + x;
      if (is_corner_7<Greater>(p, w, barrier) || is_corner_7<Less>(p, w, barrier)) {
        corners.push_back(fast_xy(x, y));
      }
    }
  }
}

} // namespace fast
