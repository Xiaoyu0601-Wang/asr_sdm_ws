#include "asr_sdm_video_enhancement/enhancement_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace asr_sdm_video_enhancement
{

EnhancementQualityProfile parseEnhancementQualityProfile(const std::string & profile_name)
{
  if (profile_name == "default") {
    return EnhancementQualityProfile::Default;
  }
  if (profile_name == "offline_gt_like") {
    return EnhancementQualityProfile::OfflineGtLike;
  }
  throw std::invalid_argument(
          "unsupported quality profile: " + profile_name +
          " (expected 'default' or 'offline_gt_like')");
}

std::string toString(EnhancementQualityProfile profile)
{
  switch (profile) {
    case EnhancementQualityProfile::Default:
      return "default";
    case EnhancementQualityProfile::OfflineGtLike:
      return "offline_gt_like";
  }
  return "default";
}

EnhancementConfig makeEnhancementConfig(EnhancementQualityProfile profile)
{
  EnhancementConfig config;
  if (profile == EnhancementQualityProfile::OfflineGtLike) {
    config.scale = 1.0;
    config.use_adaptive_airlight = true;
    config.airlight_percentile = 0.99;
    config.transmission_gain = 0.80;
    config.transmission_floor = 0.15;
    config.gamma_blend = 0.55;
    config.enable_white_balance = true;
    config.enable_clahe = true;
    config.white_balance_blend = 0.50;
    config.clahe_blend = 0.45;
    config.original_blend = 0.05;
    config.clahe_clip_limit = 2.2;
    config.clahe_tile_size = 8;
    config.enable_pre_denoise = true;
    config.pre_denoise_luma_strength = 5.0;
    config.pre_denoise_color_strength = 5.0;
    config.pre_denoise_template_window = 7;
    config.pre_denoise_search_window = 21;
  }
  return config;
}

cv::Mat EnhancementProcessor::process(const cv::Mat & frame, const EnhancementConfig & config) const
{
  if (frame.empty()) {
    return frame;
  }

  EnhancementConfig safe_config = config;
  safe_config.manual_airlight = std::clamp(safe_config.manual_airlight, 1, 255);
  safe_config.scale = std::max(0.01, safe_config.scale);
  safe_config.airlight_percentile = std::clamp(safe_config.airlight_percentile, 0.50, 0.999);
  safe_config.transmission_floor = std::clamp(safe_config.transmission_floor, 0.05, 0.95);
  safe_config.transmission_gain = std::clamp(safe_config.transmission_gain, 0.05, 2.0);
  safe_config.gamma_blend = std::clamp(safe_config.gamma_blend, 0.0, 1.0);
  safe_config.white_balance_blend = std::clamp(safe_config.white_balance_blend, 0.0, 1.0);
  safe_config.clahe_blend = std::clamp(safe_config.clahe_blend, 0.0, 1.0);
  safe_config.original_blend = std::clamp(safe_config.original_blend, 0.0, 1.0);
  safe_config.clahe_clip_limit = std::max(0.1, safe_config.clahe_clip_limit);
  safe_config.clahe_tile_size = std::max(1, safe_config.clahe_tile_size);
  safe_config.pre_denoise_luma_strength = std::max(0.0, safe_config.pre_denoise_luma_strength);
  safe_config.pre_denoise_color_strength = std::max(0.0, safe_config.pre_denoise_color_strength);
  safe_config.pre_denoise_template_window = std::max(1, safe_config.pre_denoise_template_window);
  safe_config.pre_denoise_search_window = std::max(1, safe_config.pre_denoise_search_window);

  cv::Mat working_frame;
  if (std::abs(safe_config.scale - 1.0) < 1e-6) {
    working_frame = frame;
  } else {
    cv::resize(
      frame, working_frame, cv::Size(), safe_config.scale, safe_config.scale,
      cv::INTER_LINEAR);
  }

  if (safe_config.enable_pre_denoise) {
    working_frame = applyPreDenoise(working_frame, safe_config);
  }

  cv::Mat y_channel = calcYchannel(working_frame);
  cv::Mat y_channel_median;
  cv::medianBlur(y_channel, y_channel_median, 5);

  const int airlight = safe_config.use_adaptive_airlight ?
    estimateAirlight(y_channel, safe_config.airlight_percentile) :
    safe_config.manual_airlight;

  cv::Mat transmission = calcTransmission(
    working_frame, y_channel_median, airlight, safe_config.transmission_gain,
    safe_config.gamma_blend);
  cv::Mat enhanced = dehazing(
    working_frame, transmission, airlight, safe_config.transmission_floor);

  if (safe_config.enable_white_balance && safe_config.white_balance_blend > 0.0) {
    enhanced = blendImages(
      enhanced, applyWhiteBalance(enhanced), safe_config.white_balance_blend);
  }
  if (safe_config.enable_clahe && safe_config.clahe_blend > 0.0) {
    enhanced = blendImages(
      enhanced,
      applyClahe(enhanced, safe_config.clahe_clip_limit, safe_config.clahe_tile_size),
      safe_config.clahe_blend);
  }
  if (safe_config.original_blend > 0.0) {
    enhanced = blendImages(enhanced, working_frame, safe_config.original_blend);
  }
  return enhanced;
}

cv::Mat EnhancementProcessor::process(const cv::Mat & frame, int airlight, double scale) const
{
  EnhancementConfig config = makeEnhancementConfig(EnhancementQualityProfile::Default);
  config.manual_airlight = airlight;
  config.scale = scale;
  config.use_adaptive_airlight = false;
  return process(frame, config);
}

double EnhancementProcessor::avePixel(const cv::Mat & src) const
{
  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(src, &min_val, &max_val);
  return (min_val + max_val) / 2.0;
}

cv::Mat EnhancementProcessor::calcYchannel(const cv::Mat & src) const
{
  cv::Mat yuv;
  CV_Assert(!src.empty() && src.type() == CV_8UC3);
  cv::cvtColor(src, yuv, cv::COLOR_BGR2YUV);
  cv::Mat y_channel;
  cv::extractChannel(yuv, y_channel, 0);
  return y_channel;
}

int EnhancementProcessor::estimateAirlight(const cv::Mat & y_channel, double percentile) const
{
  CV_Assert(!y_channel.empty() && y_channel.type() == CV_8UC1);

  std::array<int, 256> histogram{};
  for (int row = 0; row < y_channel.rows; ++row) {
    const auto * row_ptr = y_channel.ptr<uchar>(row);
    for (int col = 0; col < y_channel.cols; ++col) {
      histogram[row_ptr[col]]++;
    }
  }

  const std::size_t total = y_channel.total();
  const double target = percentile * static_cast<double>(total - 1);
  std::size_t cumulative = 0;
  for (std::size_t value = 0; value < histogram.size(); ++value) {
    cumulative += static_cast<std::size_t>(histogram[value]);
    if (static_cast<double>(cumulative - 1) >= target) {
      return std::clamp(static_cast<int>(value), 160, 255);
    }
  }
  return 255;
}

cv::Mat EnhancementProcessor::applyPreDenoise(
  const cv::Mat & src, const EnhancementConfig & config) const
{
  CV_Assert(!src.empty() && src.type() == CV_8UC3);

  cv::Mat denoised;
  cv::fastNlMeansDenoisingColored(
    src, denoised,
    static_cast<float>(config.pre_denoise_luma_strength),
    static_cast<float>(config.pre_denoise_color_strength),
    config.pre_denoise_template_window,
    config.pre_denoise_search_window);
  return denoised;
}

cv::Mat EnhancementProcessor::calcTransmission(
  const cv::Mat & src, const cv::Mat & median_y, int airlight, double transmission_gain,
  double gamma_blend) const
{
  CV_Assert(!src.empty() && !median_y.empty());

  const double m = avePixel(src) / 255.0;
  const double p = 1.3;
  const double q = 1 + (m - 0.5);
  const double k = std::min(m * p * q * transmission_gain, 0.95);

  cv::Mat transmission_float;
  median_y.convertTo(
    transmission_float, CV_32F,
    -255.0 * k / static_cast<double>(std::max(airlight, 1)), 255.0);
  cv::Mat transmission;
  transmission_float.convertTo(transmission, CV_8U);
  const float raw_gamma = 1.3f - static_cast<float>(m);
  const float gamma = std::clamp(
    1.0f + static_cast<float>(gamma_blend) * (raw_gamma - 1.0f), 0.8f, 1.3f);
  gammaCorrection(transmission, transmission, gamma);
  return transmission;
}

cv::Mat EnhancementProcessor::dehazing(
  const cv::Mat & src, const cv::Mat & transmission, int airlight,
  double transmission_floor) const
{
  CV_Assert(!src.empty() && !transmission.empty());

  cv::Mat transmission_float;
  transmission.convertTo(transmission_float, CV_32F, 1.0 / 255.0);
  cv::max(transmission_float, static_cast<float>(transmission_floor), transmission_float);

  cv::Mat src_float;
  src.convertTo(src_float, CV_32FC3);

  std::vector<cv::Mat> channels;
  cv::split(src_float, channels);
  for (auto & channel : channels) {
    channel = (channel - static_cast<float>(airlight)) / transmission_float +
      static_cast<float>(airlight);
  }

  cv::Mat dst_float;
  cv::merge(channels, dst_float);

  cv::Mat dst;
  dst_float.convertTo(dst, CV_8UC3);
  return dst;
}

void EnhancementProcessor::gammaCorrection(
  const cv::Mat & src, cv::Mat & dst, float gamma) const
{
  CV_Assert(!src.empty());

  cv::Mat lut(1, 256, CV_8U);
  auto * lut_data = lut.ptr<uchar>();
  for (int i = 0; i < 256; ++i) {
    lut_data[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, gamma) * 255.0f);
  }

  cv::LUT(src, lut, dst);
}

cv::Mat EnhancementProcessor::applyWhiteBalance(const cv::Mat & src) const
{
  CV_Assert(!src.empty() && src.type() == CV_8UC3);

  cv::Mat src_float;
  src.convertTo(src_float, CV_32FC3);

  std::vector<cv::Mat> channels;
  cv::split(src_float, channels);

  const cv::Scalar b_mean = cv::mean(channels[0]);
  const cv::Scalar g_mean = cv::mean(channels[1]);
  const cv::Scalar r_mean = cv::mean(channels[2]);
  const double gray_mean = (b_mean[0] + g_mean[0] + r_mean[0]) / 3.0;

  const std::array<double, 3> gains = {
    std::clamp(gray_mean / std::max(b_mean[0], 1.0), 0.7, 1.3),
    std::clamp(gray_mean / std::max(g_mean[0], 1.0), 0.7, 1.3),
    std::clamp(gray_mean / std::max(r_mean[0], 1.0), 0.7, 1.3)};

  for (std::size_t index = 0; index < channels.size(); ++index) {
    channels[index] *= gains[index];
  }

  cv::Mat balanced_float;
  cv::merge(channels, balanced_float);
  cv::Mat balanced;
  balanced_float.convertTo(balanced, CV_8UC3);
  return balanced;
}

cv::Mat EnhancementProcessor::applyClahe(const cv::Mat & src, double clip_limit, int tile_size) const
{
  CV_Assert(!src.empty() && src.type() == CV_8UC3);

  cv::Mat lab;
  cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);

  auto clahe = cv::createCLAHE(clip_limit, cv::Size(tile_size, tile_size));
  clahe->apply(channels[0], channels[0]);

  cv::merge(channels, lab);
  cv::Mat enhanced;
  cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
  return enhanced;
}

cv::Mat EnhancementProcessor::blendImages(
  const cv::Mat & base, const cv::Mat & adjusted, double blend_ratio) const
{
  CV_Assert(!base.empty() && !adjusted.empty());
  CV_Assert(base.size() == adjusted.size());
  CV_Assert(base.type() == adjusted.type());

  cv::Mat blended;
  cv::addWeighted(
    base, 1.0 - blend_ratio, adjusted, blend_ratio, 0.0, blended);
  return blended;
}

}  // namespace asr_sdm_video_enhancement
