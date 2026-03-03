#include <fast/fast.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace {

using fast::fast_xy;
using fast::fast_byte;

struct XYLess {
  bool operator()(const fast_xy& a, const fast_xy& b) const {
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  }
};

inline bool is_positive(int val, int center, int barrier) {
  return val > center + barrier;
}

inline bool is_negative(int val, int center, int barrier) {
  return val < center - barrier;
}

bool is_corner_segment(const std::vector<fast_byte>& img, int w, int x, int y, int barrier, int num_for_corner) {
  static const int dx[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
  static const int dy[16] = {3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2, 3};

  const int center = img[y * w + x];
  int consecutive = 0;
  int first_cons = 0;

  for (int i = 0; i < 16; ++i) {
    const int val = img[(y + dy[i]) * w + (x + dx[i])];
    if (is_positive(val, center, barrier)) {
      ++consecutive;
      if (consecutive == num_for_corner) return true;
    } else {
      if (consecutive == i) first_cons = i;
      consecutive = 0;
    }
  }
  if (first_cons + consecutive >= num_for_corner) return true;

  consecutive = 0;
  first_cons = 0;
  for (int i = 0; i < 16; ++i) {
    const int val = img[(y + dy[i]) * w + (x + dx[i])];
    if (is_negative(val, center, barrier)) {
      ++consecutive;
      if (consecutive == num_for_corner) return true;
    } else {
      if (consecutive == i) first_cons = i;
      consecutive = 0;
    }
  }
  return first_cons + consecutive >= num_for_corner;
}

std::set<fast_xy, XYLess> segment_detect(const std::vector<fast_byte>& img, int w, int h, int barrier, int num_for_corner) {
  std::set<fast_xy, XYLess> out;
  for (int y = 3; y < h - 3; ++y) {
    for (int x = 3; x < w - 3; ++x) {
      if (is_corner_segment(img, w, x, y, barrier, num_for_corner)) {
        out.insert(fast_xy(static_cast<short>(x), static_cast<short>(y)));
      }
    }
  }
  return out;
}

std::set<fast_xy, XYLess> fast_detect(const std::vector<fast_byte>& img, int w, int h, int barrier, int type) {
  std::vector<fast_xy> corners;
  if (type == 11) {
    fast::fast_corner_detect_11(img.data(), w, h, w, static_cast<short>(barrier), corners);
  } else {
    fast::fast_corner_detect_12(img.data(), w, h, w, static_cast<short>(barrier), corners);
  }
  return std::set<fast_xy, XYLess>(corners.begin(), corners.end());
}

bool validate_type(int type, int rounds) {
  std::mt19937 rng(42 + type);
  std::uniform_int_distribution<int> size_dist(16, 96);
  std::uniform_int_distribution<int> pixel_dist(0, 255);
  std::uniform_int_distribution<int> thr_dist(0, 255);

  for (int r = 0; r < rounds; ++r) {
    const int w = size_dist(rng);
    const int h = size_dist(rng);
    std::vector<fast_byte> img(static_cast<size_t>(w * h));
    for (auto& p : img) p = static_cast<fast_byte>(pixel_dist(rng));

    const int barrier = thr_dist(rng);
    const auto ref = segment_detect(img, w, h, barrier, type);
    const auto got = fast_detect(img, w, h, barrier, type);

    const bool same_size = (ref.size() == got.size());
    const bool same_points = same_size && std::equal(
        ref.begin(), ref.end(), got.begin(),
        [](const fast_xy& a, const fast_xy& b) { return a.x == b.x && a.y == b.y; });

    if (!same_points) {
      std::cerr << "FAST" << type << " mismatch at round " << r
                << " size=" << w << "x" << h
                << " barrier=" << barrier
                << " ref=" << ref.size() << " got=" << got.size() << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  const bool ok11 = validate_type(11, 200);
  const bool ok12 = validate_type(12, 200);

  if (!ok11 || !ok12) {
    std::cerr << "FAST-11/12 validation failed." << std::endl;
    return 1;
  }

  std::cout << "FAST-11/12 validation passed." << std::endl;
  return 0;
}
