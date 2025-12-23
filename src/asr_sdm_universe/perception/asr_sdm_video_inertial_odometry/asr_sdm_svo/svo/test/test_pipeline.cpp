#include "test_utils.h"

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/math_utils.h>
#include <vikit/pinhole_camera.h>
#include <vikit/vision.h>

#include <iostream>
#include <string>
#include <vector>

namespace svo
{

class BenchmarkNode
{
  vk::AbstractCamera * cam_;
  svo::FrameHandlerMono * vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
  for (int img_id = 2; img_id < 188; ++img_id) {
    // load image
    std::stringstream ss;
    ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_" << std::setw(6)
       << std::setfill('0') << img_id << "_0.png";
    if (img_id == 2) std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01 * img_id);

    // display tracking quality
    if (vo_->lastFrame() != NULL) {
      std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                << "#Features: " << vo_->lastNumObservations() << " \t"
                << "Proc. Time: " << vo_->lastProcessingTime() * 1000 << "ms \n";

      // access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}

}  // namespace svo

int main(int argc, char ** argv)
{
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}
