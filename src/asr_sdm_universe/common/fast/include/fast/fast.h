#ifndef FAST_H
#define FAST_H

#include <vector>

namespace fast
{

using ::std::vector;

struct fast_xy
{
  short x, y;
  fast_xy(short x_, short y_) : x(x_), y(y_) {}
};

typedef unsigned char fast_byte;

/// plain C++ version of the corner 7
void fast_corner_detect_7(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// plain C++ version of the corner 8
void fast_corner_detect_8(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// plain C++ version of the corner 9
void fast_corner_detect_9(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// NEON optimized version of the corner 9
void fast_corner_detect_9_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// plain C++ version of the corner 10
void fast_corner_detect_10(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners); 

/// SSE2 optimized version of the corner 10
void fast_corner_detect_10_sse2(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// plain C++ version of the corner 11
void fast_corner_detect_11(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// plain C++ version of the corner 12
void fast_corner_detect_12(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners);

/// corner score 10
void fast_corner_score_10(const fast_byte* img, const int img_stride, const vector<fast_xy>& corners, const int threshold, vector<int>& scores);

/// corner score 12
void fast_corner_score_12(const fast_byte* img, const int img_stride, const vector<fast_xy>& corners, const int threshold, vector<int>& scores);

/// Nonmax Suppression on a 3x3 Window
void fast_nonmax_3x3(const vector<fast_xy>& corners, const vector<int>& scores, vector<int>& nonmax_corners);

} // namespace fast

#endif
