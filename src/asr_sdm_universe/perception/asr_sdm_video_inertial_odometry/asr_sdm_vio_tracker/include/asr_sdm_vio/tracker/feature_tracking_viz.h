#pragma once

#include <asr_sdm_vio/common/types.h>

namespace svo {

// Forward declarations
class FeatureTracker;

void visualizeTracks(
    const FeatureTracker& tracker, size_t frame_index, int sleep);

} // namespace svo
