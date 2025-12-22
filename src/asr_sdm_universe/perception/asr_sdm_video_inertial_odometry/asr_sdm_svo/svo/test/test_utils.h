#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <string.h>

#include <cstdlib>  // for getenv
#ifdef SVO_USE_ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace svo
{
namespace test_utils
{

std::string getDatasetDir()
{
  const char * env_dir = std::getenv("SVO_DATASET_DIR");
#ifdef SVO_USE_ROS
  std::string dataset_dir;
  try {
    dataset_dir = ament_index_cpp::get_package_share_directory("svo") + "/test/data";
  } catch (...) {
    dataset_dir = "/tmp/svo_test_data";
  }
  if (env_dir != NULL) dataset_dir = std::string(env_dir);
  return dataset_dir;
#else
  return std::string(env_dir);
#endif
}

std::string getTraceDir()
{
#ifdef SVO_USE_ROS
  return "/tmp";
#else
  return "/tmp";
#endif
}

}  // namespace test_utils
}  // namespace svo

#endif  // TEST_UTILS_H_
