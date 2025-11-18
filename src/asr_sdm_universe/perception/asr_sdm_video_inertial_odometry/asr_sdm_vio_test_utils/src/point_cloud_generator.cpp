#include <asr_sdm_vio/test_utils/point_cloud_generator.h>
#include <asr_sdm_vio/test_utils/test_utils.h>
#include <video_kit/sample.h>
#include <fstream>
#include <random>
#include <algorithm>

namespace svo {
namespace test_utils {

using namespace Eigen;

std::vector<Vector3d> UniformRandomPCG::generatePointCloud(size_t num_points)
{
  std::vector<Vector3d> point_cloud;
  for(size_t i=0;i<num_points;++i)
  {
    Vector3d point = svo::test_utils::generateRandomPoint(max_depth_, min_depth_);
    point_cloud.push_back(point);
  }
  return point_cloud;
}

std::vector<Vector3d> GaussianRandomPCG::generatePointCloud(size_t num_points)
{
  std::vector<Vector3d> point_cloud;
  for(size_t i=0;i<num_points;++i)
  {
    Vector3d point;
    point.setRandom();
    point.normalize();
    const double rnd_depth = mean_depth_ + vk::Sample::gaussian(std_depth_);
    const double depth = rnd_depth >= 0.0 ? rnd_depth : 0.0;
    point_cloud.push_back(depth * point);
  }
  return point_cloud;
}

FromFilePCG::FromFilePCG(const std::string& filename)
  : PCG()
{
  std::ifstream ss;
  ss.open(filename);
  if(!ss.is_open())
    LOG(FATAL) << "Could not open point cloud file: " << filename;

  double x, y, z;
  while(ss >> x >> y >> z) {
    points_.push_back(Vector3d(x, y, z));
  }
}

std::vector<Vector3d> FromFilePCG::generatePointCloud(size_t /*num_points*/)
{
  static std::random_device rd;
  static std::mt19937 g(rd());
  std::shuffle(points_.begin(), points_.end(), g);
  return points_;
}

}
}
