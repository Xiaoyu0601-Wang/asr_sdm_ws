#pragma once

// geometry
/*
#include <video_kit/cameras/camera_geometry_base.h>
#include <video_kit/cameras/camera_geometry.h>
*/

// distortion models
/*
#include <video_kit/cameras/no_distortion.h>
#include <video_kit/cameras/atan_distortion.h>
#include <video_kit/cameras/equidistant_distortion.h>
#include <video_kit/cameras/radial_tangential_distortion.h>

// projections
#include <video_kit/cameras/pinhole_projection.h>
*/

namespace vk {
namespace cameras {

template <typename ProjectionType>
class CameraGeometry;

template <typename DistrortionType>
class PinholeProjection;

class AtanDistortion;
class EquidistantDistortion;
class NoDistortion;
class RadialTangentialDistortion;

typedef CameraGeometry<PinholeProjection<NoDistortion>> PinholeGeometry;
typedef CameraGeometry<PinholeProjection<AtanDistortion>> PinholeAtanGeometry;
typedef CameraGeometry<PinholeProjection<EquidistantDistortion>>
    PinholeEquidistantGeometry;
typedef CameraGeometry<PinholeProjection<RadialTangentialDistortion>>
    PinholeRadTanGeometry;
class OmniGeometry;
} // namespace cameras
} // namespace vk
