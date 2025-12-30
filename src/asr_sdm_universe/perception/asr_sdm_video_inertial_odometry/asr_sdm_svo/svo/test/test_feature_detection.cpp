#include "test_utils.h"

#include <string.h>
#include <svo/config.h>
#include <svo/depth_filter.h>
#include <svo/feature.h>
#include <svo/feature_detection.h>
#include <svo/frame.h>
#include <svo/global.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/timer.h>
#include <vikit/vision.h>

namespace
{

using namespace Eigen;
using namespace std;

void testCornerDetector()
{
  std::string img_name(
    svo::test_utils::getDatasetDir() + "/sin2_tex2_h1_v8_d/img/frame_000002_0.png");
  printf("Loading image '%s'\n", img_name.c_str());
  cv::Mat img(cv::imread(img_name, 0));
  assert(img.type() == CV_8UC1 && !img.empty());

  vk::AbstractCamera * cam =
    new vk::ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);
  svo::FramePtr frame(new svo::Frame(cam, img, 0.0));

  // Corner detection
  vk::Timer t;
  svo::Features fts;
  svo::feature_detection::FastDetector fast_detector(
    img.cols, img.rows, svo::Config::gridSize(), svo::Config::nPyrLevels());
  for (int i = 0; i < 100; ++i) {
    fast_detector.detect(frame.get(), frame->img_pyr_, svo::Config::triangMinCornerScore(), fts);
  }
  printf(
    "Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n",
    t.stop() * 10, fts.size());
  printf(
    "Note, in this case, feature detection also contains the cam2world projection of the "
    "feature.\n");
  // Visualization (commented out for headless environments)
  // cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
  // cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
  // std::for_each(fts.begin(), fts.end(), [&](svo::Feature * i) {
  //   cv::circle(
  //     img_rgb, cv::Point2f(i->px[0], i->px[1]), 4 * (i->level + 1), cv::Scalar(0, 255, 0), 1);
  // });
  // cv::imshow("ref_img", img_rgb);
  // cv::waitKey(0);

  std::for_each(fts.begin(), fts.end(), [&](svo::Feature * i) { delete i; });
}

}  // namespace

int main(int argc, char ** argv)
{
  testCornerDetector();
  return 0;
}
