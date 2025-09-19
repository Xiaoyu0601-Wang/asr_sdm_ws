#include "asr_sdm_video_enhancement/asr_sdm_video_enhancement.hpp"

namespace asr_sdm_video_enhancement
{

VideoEnhancementNode::VideoEnhancementNode(const rclcpp::NodeOptions & node_options)
: Node("asr_sdm_video_enhancement", node_options)
{
  // Parameter
  airlight = static_cast<int>(declare_parameter("airlight", 255));
  scale = static_cast<double>(declare_parameter("scale", 0.5));

  // Subscriber
  sub_image_ = this->create_subscription<Image>(
  "~/input/image", 1, std::bind(&VideoEnhancementNode::onImageCallback, this, _1));

  // Publisher
  pub_image_ = this->create_publisher<Image>("~/output/image", 10);
}

// Callback
void VideoEnhancementNode::onImageCallback(const Image::SharedPtr msg)
{
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // Calculate FPS
    auto now = std::chrono::steady_clock::now();
    if (frame_count_ > 0) {
        double elapsed = std::chrono::duration<double>(now - last_frame_time_).count();
        double fps_now = 1.0 / elapsed;
        avg_fps_ = (avg_fps_ * (frame_count_ - 1) + fps_now) / frame_count_;
        std::string text = "FPS: " + std::to_string(fps_now).substr(0,5) +
                           " Avg: " + std::to_string(avg_fps_).substr(0,5);
        putText(cv_ptr->image, text, cv::Point(20,40), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0,255,0), 2);
    }
    last_frame_time_ = now;
    frame_count_++;

    resize(cv_ptr->image, cv_ptr->image, cv::Size(), scale, scale, cv::INTER_LINEAR);
    Mat processed = processFrame(cv_ptr->image, airlight);
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", processed).toImageMsg();
    pub_image_->publish(*out_msg);

  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

// Function
double VideoEnhancementNode::avePixel(const Mat & src)
{
  double minVal, maxVal;
  minMaxLoc(src, &minVal, &maxVal);
  return (minVal + maxVal) / 2.0;
}

Mat VideoEnhancementNode::calcYchannel(const Mat & src)
{
  Mat yuv;
  CV_Assert(!src.empty() && src.type() == CV_8UC3);
  cvtColor(src, yuv, COLOR_BGR2YUV);
  std::vector<Mat> planes;
  split(yuv, planes);
  return planes[0];
}

Mat VideoEnhancementNode::calcTransmission(const Mat & src, const Mat & Mmed, int airlight)
{
  CV_Assert(!src.empty() && !Mmed.empty());

  double m = avePixel(src) / 255.0;  // Use m to determin the haze is more or less
  double p = 1.3;                    // Set this value by experiment
  double q = 1 + (m - 0.5);  // Value q is decided by value m, if m is big and q will be bigger to
                             // remove more haze. <- Auto-tunning parameter
  double k = min(m * p * q, 0.95);

  Mat transmission = 255 * (1 - k * Mmed / airlight);
  gammaCorrection(transmission, transmission, 1.3f - static_cast<float>(m));
  return transmission;
}

Mat VideoEnhancementNode::dehazing(const Mat & src, const Mat & transmission, int airlight)
{
  CV_Assert(!src.empty() && !transmission.empty());

  Mat dst(src.size(), CV_8UC3);
  const double tmin = 0.1;

  for (int i = 0; i < src.rows; i++) {
    const Vec3b * rowSrc = src.ptr<Vec3b>(i);
    const uchar * rowT = transmission.ptr<uchar>(i);
    Vec3b * rowDst = dst.ptr<Vec3b>(i);

    for (int j = 0; j < src.cols; j++) {
      double tVal = max(rowT[j] / 255.0, tmin);
      for (int c = 0; c < 3; c++) {
        double val = (rowSrc[j][c] - airlight) / tVal + airlight;
        rowDst[j][c] = saturate_cast<uchar>(val);
      }
    }
  }
  return dst;
}

Mat VideoEnhancementNode::processFrame(const Mat & frame, int airlight)
{
  if (frame.empty()) return frame;

  Mat yChannel, yChannelMedian, transmission, dehazed;
  yChannel = calcYchannel(frame);
  medianBlur(yChannel, yChannelMedian, 5);
  transmission = calcTransmission(frame, yChannelMedian, airlight);
  dehazed = dehazing(frame, transmission, airlight);
  return dehazed;
}

void VideoEnhancementNode::gammaCorrection(const Mat & src, Mat & dst, float gamma)
{
  CV_Assert(!src.empty());

  uchar lut[256];
  for (int i = 0; i < 256; i++) {
    lut[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0f);
  }

  dst = src.clone();
  for (int i = 0; i < src.rows; i++) {
    uchar * row = dst.ptr<uchar>(i);
    for (int j = 0; j < src.cols * src.channels(); j++) {
      row[j] = lut[row[j]];
    }
  }
}

} // namespace asr_sdm_video_enhancement

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_video_enhancement::VideoEnhancementNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

