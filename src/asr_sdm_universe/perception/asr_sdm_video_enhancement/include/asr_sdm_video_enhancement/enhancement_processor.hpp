#ifndef ASR_SDM_VIDEO_ENHANCEMENT__ENHANCEMENT_PROCESSOR_HPP_
#define ASR_SDM_VIDEO_ENHANCEMENT__ENHANCEMENT_PROCESSOR_HPP_

#include <opencv2/opencv.hpp>

#include <string>

namespace asr_sdm_video_enhancement
{

enum class EnhancementQualityProfile
{
  Default,
  OfflineGtLike,
};

struct EnhancementConfig
{
  int manual_airlight = 255;
  double scale = 1.0;
  bool use_adaptive_airlight = true;
  double airlight_percentile = 0.99;
  double transmission_floor = 0.25;
  double transmission_gain = 0.50;
  double gamma_blend = 0.30;
  bool enable_white_balance = true;
  bool enable_clahe = true;
  double white_balance_blend = 0.35;
  double clahe_blend = 0.30;
  double original_blend = 0.20;
  double clahe_clip_limit = 1.4;
  int clahe_tile_size = 8;
  bool enable_pre_denoise = false;
  double pre_denoise_luma_strength = 5.0;
  double pre_denoise_color_strength = 5.0;
  int pre_denoise_template_window = 7;
  int pre_denoise_search_window = 21;
};

EnhancementQualityProfile parseEnhancementQualityProfile(const std::string & profile_name);
std::string toString(EnhancementQualityProfile profile);
EnhancementConfig makeEnhancementConfig(EnhancementQualityProfile profile);

class EnhancementProcessor
{
public:
  cv::Mat process(const cv::Mat & frame, const EnhancementConfig & config) const;
  cv::Mat process(const cv::Mat & frame, int airlight, double scale) const;

private:
  double avePixel(const cv::Mat & src) const;
  cv::Mat calcYchannel(const cv::Mat & src) const;
  int estimateAirlight(const cv::Mat & y_channel, double percentile) const;
  cv::Mat applyPreDenoise(const cv::Mat & src, const EnhancementConfig & config) const;
  cv::Mat calcTransmission(
    const cv::Mat & src, const cv::Mat & median_y, int airlight, double transmission_gain,
    double gamma_blend) const;
  cv::Mat dehazing(
    const cv::Mat & src, const cv::Mat & transmission, int airlight,
    double transmission_floor) const;
  void gammaCorrection(const cv::Mat & src, cv::Mat & dst, float gamma) const;
  cv::Mat applyWhiteBalance(const cv::Mat & src) const;
  cv::Mat applyClahe(const cv::Mat & src, double clip_limit, int tile_size) const;
  cv::Mat blendImages(const cv::Mat & base, const cv::Mat & adjusted, double blend_ratio) const;
};

}  // namespace asr_sdm_video_enhancement

#endif  // ASR_SDM_VIDEO_ENHANCEMENT__ENHANCEMENT_PROCESSOR_HPP_
