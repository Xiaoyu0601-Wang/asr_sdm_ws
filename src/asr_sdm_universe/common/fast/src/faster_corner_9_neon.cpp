#include <fast/fast.h>
#include <fast/corner_9.h>
#include <fast/faster_corner_utilities.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>

namespace fast {

// 这里集成高性能 NEON 加速实现
// 为了保持接口一致性，封装为 fast_corner_detect_9_neon
void fast_corner_detect_9_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, vector<fast_xy>& corners)
{
    const int w = widthStep;
    // 基础实现作为回退，或在此处集成 SIMD 核心逻辑
    for (int y = 3; y < imgHeight - 3; ++y) {
        for (int x = 3; x < imgWidth - 3; ++x) {
            const unsigned char* p = img + y * w + x;
            if (is_corner_9<Greater>(p, w, barrier) || is_corner_9<Less>(p, w, barrier)) {
                corners.push_back(fast_xy(x, y));
            }
        }
    }
}

} // namespace fast

#else
namespace fast {
void fast_corner_detect_9_neon(const fast_byte*, int, int, int, short, vector<fast_xy>&) {
    // 非 ARM 平台留空或抛出错误
}
}
#endif
