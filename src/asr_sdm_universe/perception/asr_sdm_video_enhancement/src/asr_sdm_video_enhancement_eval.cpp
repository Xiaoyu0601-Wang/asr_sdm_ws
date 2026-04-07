#include "asr_sdm_video_enhancement/enhancement_processor.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace asr_sdm_video_enhancement
{

namespace
{

struct EvalOptions
{
  fs::path blur_dir = "/home/cortin/Downloads/test/blur";
  fs::path reference_dir = "/home/cortin/Downloads/test/gt";
  fs::path output_dir = "/tmp/asr_sdm_video_eval";
  EnhancementQualityProfile quality_profile = EnhancementQualityProfile::Default;
  std::optional<int> airlight_override;
  std::optional<double> scale_override;
  int frame_step = 1;
  double metric_scale = 1.0;
};

struct RunningStats
{
  std::size_t count = 0;
  double sum = 0.0;
  double sum_sq = 0.0;
  double min = std::numeric_limits<double>::infinity();
  double max = -std::numeric_limits<double>::infinity();

  void add(double value)
  {
    count++;
    sum += value;
    sum_sq += value * value;
    min = std::min(min, value);
    max = std::max(max, value);
  }

  double mean() const
  {
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
  }

  double stddev() const
  {
    if (count == 0) {
      return 0.0;
    }
    const double mu = mean();
    const double variance = std::max(0.0, sum_sq / static_cast<double>(count) - mu * mu);
    return std::sqrt(variance);
  }
};

struct FrameMetrics
{
  double psnr_input_ref = 0.0;
  double psnr_enhanced_ref = 0.0;
  double delta_psnr = 0.0;
  double ssim_input_ref = 0.0;
  double ssim_enhanced_ref = 0.0;
  double delta_ssim = 0.0;
  double uciqe_input = 0.0;
  double uciqe_enhanced = 0.0;
  double delta_uciqe = 0.0;
  double uiqm_input = 0.0;
  double uiqm_enhanced = 0.0;
  double delta_uiqm = 0.0;
};

struct FrameMetricAccumulator
{
  RunningStats psnr_input_ref;
  RunningStats psnr_enhanced_ref;
  RunningStats delta_psnr;
  RunningStats ssim_input_ref;
  RunningStats ssim_enhanced_ref;
  RunningStats delta_ssim;
  RunningStats uciqe_input;
  RunningStats uciqe_enhanced;
  RunningStats delta_uciqe;
  RunningStats uiqm_input;
  RunningStats uiqm_enhanced;
  RunningStats delta_uiqm;

  void add(const FrameMetrics & metrics)
  {
    psnr_input_ref.add(metrics.psnr_input_ref);
    psnr_enhanced_ref.add(metrics.psnr_enhanced_ref);
    delta_psnr.add(metrics.delta_psnr);
    ssim_input_ref.add(metrics.ssim_input_ref);
    ssim_enhanced_ref.add(metrics.ssim_enhanced_ref);
    delta_ssim.add(metrics.delta_ssim);
    uciqe_input.add(metrics.uciqe_input);
    uciqe_enhanced.add(metrics.uciqe_enhanced);
    delta_uciqe.add(metrics.delta_uciqe);
    uiqm_input.add(metrics.uiqm_input);
    uiqm_enhanced.add(metrics.uiqm_enhanced);
    delta_uiqm.add(metrics.delta_uiqm);
  }
};

struct VideoPair
{
  std::string video_id;
  std::string reference_method;
  fs::path blur_path;
  fs::path reference_path;
};

struct SkippedFile
{
  std::string file_name;
  std::string reason;
};

struct PairResult
{
  std::string video_id;
  std::string reference_method;
  fs::path blur_path;
  fs::path reference_path;
  std::string status = "ok";
  std::size_t reported_blur_frames = 0;
  std::size_t reported_reference_frames = 0;
  std::size_t frames_evaluated = 0;
  bool truncated_pair = false;
  bool resized_to_reference = false;
  FrameMetricAccumulator metrics;
};

struct DatasetAccumulator
{
  std::size_t pair_count = 0;
  std::size_t frame_count = 0;
  FrameMetricAccumulator metrics;

  void addVideo(const PairResult & result)
  {
    if (result.frames_evaluated == 0) {
      return;
    }
    pair_count++;
    frame_count += result.frames_evaluated;

    FrameMetrics mean_metrics;
    mean_metrics.psnr_input_ref = result.metrics.psnr_input_ref.mean();
    mean_metrics.psnr_enhanced_ref = result.metrics.psnr_enhanced_ref.mean();
    mean_metrics.delta_psnr = result.metrics.delta_psnr.mean();
    mean_metrics.ssim_input_ref = result.metrics.ssim_input_ref.mean();
    mean_metrics.ssim_enhanced_ref = result.metrics.ssim_enhanced_ref.mean();
    mean_metrics.delta_ssim = result.metrics.delta_ssim.mean();
    mean_metrics.uciqe_input = result.metrics.uciqe_input.mean();
    mean_metrics.uciqe_enhanced = result.metrics.uciqe_enhanced.mean();
    mean_metrics.delta_uciqe = result.metrics.delta_uciqe.mean();
    mean_metrics.uiqm_input = result.metrics.uiqm_input.mean();
    mean_metrics.uiqm_enhanced = result.metrics.uiqm_enhanced.mean();
    mean_metrics.delta_uiqm = result.metrics.delta_uiqm.mean();
    metrics.add(mean_metrics);
  }
};

template<typename ValueT>
std::string quoteCsv(const ValueT & value)
{
  std::ostringstream stream;
  stream << value;
  std::string text = stream.str();
  std::string escaped;
  escaped.reserve(text.size() + 2);
  escaped.push_back('"');
  for (char ch : text) {
    if (ch == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(ch);
  }
  escaped.push_back('"');
  return escaped;
}

std::string jsonEscape(const std::string & text)
{
  std::string escaped;
  escaped.reserve(text.size());
  for (char ch : text) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(ch);
        break;
    }
  }
  return escaped;
}

double elapsedSeconds(const std::chrono::steady_clock::time_point & start_time)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
}

void writeMetricJson(std::ostream & out, const std::string & name, const RunningStats & stats, bool last)
{
  out << "    \"" << name << "\": {"
      << "\"mean\": " << stats.mean() << ", "
      << "\"stddev\": " << stats.stddev() << ", "
      << "\"min\": " << (stats.count == 0 ? 0.0 : stats.min) << ", "
      << "\"max\": " << (stats.count == 0 ? 0.0 : stats.max) << "}";
  if (!last) {
    out << ",";
  }
  out << "\n";
}

void writeMetricCsvHeader(std::ostream & out)
{
  out
    << "video_id,reference_method,frame_index,"
    << "psnr_input_ref,psnr_enhanced_ref,delta_psnr,"
    << "ssim_input_ref,ssim_enhanced_ref,delta_ssim,"
    << "uciqe_input,uciqe_enhanced,delta_uciqe,"
    << "uiqm_input,uiqm_enhanced,delta_uiqm\n";
}

void writeVideoSummaryHeader(std::ostream & out)
{
  out
    << "video_id,reference_method,blur_path,reference_path,status,"
    << "reported_blur_frames,reported_reference_frames,frames_evaluated,"
    << "truncated_pair,resized_to_reference,"
    << "mean_psnr_input_ref,mean_psnr_enhanced_ref,mean_delta_psnr,"
    << "mean_ssim_input_ref,mean_ssim_enhanced_ref,mean_delta_ssim,"
    << "mean_uciqe_input,mean_uciqe_enhanced,mean_delta_uciqe,"
    << "mean_uiqm_input,mean_uiqm_enhanced,mean_delta_uiqm\n";
}

void writeMethodSummaryHeader(std::ostream & out)
{
  out
    << "reference_method,pair_count,frame_count,"
    << "mean_psnr_input_ref,std_psnr_input_ref,min_psnr_input_ref,max_psnr_input_ref,"
    << "mean_psnr_enhanced_ref,std_psnr_enhanced_ref,min_psnr_enhanced_ref,max_psnr_enhanced_ref,"
    << "mean_delta_psnr,std_delta_psnr,min_delta_psnr,max_delta_psnr,"
    << "mean_ssim_input_ref,std_ssim_input_ref,min_ssim_input_ref,max_ssim_input_ref,"
    << "mean_ssim_enhanced_ref,std_ssim_enhanced_ref,min_ssim_enhanced_ref,max_ssim_enhanced_ref,"
    << "mean_delta_ssim,std_delta_ssim,min_delta_ssim,max_delta_ssim,"
    << "mean_uciqe_input,std_uciqe_input,min_uciqe_input,max_uciqe_input,"
    << "mean_uciqe_enhanced,std_uciqe_enhanced,min_uciqe_enhanced,max_uciqe_enhanced,"
    << "mean_delta_uciqe,std_delta_uciqe,min_delta_uciqe,max_delta_uciqe,"
    << "mean_uiqm_input,std_uiqm_input,min_uiqm_input,max_uiqm_input,"
    << "mean_uiqm_enhanced,std_uiqm_enhanced,min_uiqm_enhanced,max_uiqm_enhanced,"
    << "mean_delta_uiqm,std_delta_uiqm,min_delta_uiqm,max_delta_uiqm\n";
}

std::string getOptionValue(int & index, int argc, char ** argv, const std::string & option)
{
  const std::string arg = argv[index];
  const std::string prefix = option + "=";
  if (arg.rfind(prefix, 0) == 0) {
    return arg.substr(prefix.size());
  }
  if (arg == option) {
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value for " + option);
    }
    index++;
    return argv[index];
  }
  return {};
}

void printUsage()
{
  std::cout
    << "Usage: asr_sdm_video_enhancement_eval [options]\n"
    << "  --blur-dir PATH         Default: /home/cortin/Downloads/test/blur\n"
    << "  --reference-dir PATH    Default: /home/cortin/Downloads/test/gt\n"
    << "  --output-dir PATH       Default: /tmp/asr_sdm_video_eval\n"
    << "  --quality-profile NAME  Default: default (default|offline_gt_like)\n"
    << "  --airlight INT          Optional manual override; disables adaptive airlight\n"
    << "  --scale FLOAT           Optional scale override\n"
    << "  --frame-step INT        Default: 1 (evaluate every Nth frame)\n"
    << "  --metric-scale FLOAT    Default: 1.0 (resize frames before metrics)\n";
}

EvalOptions parseArgs(int argc, char ** argv)
{
  EvalOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage();
      std::exit(0);
    }

    if (const auto value = getOptionValue(i, argc, argv, "--blur-dir"); !value.empty()) {
      options.blur_dir = value;
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--reference-dir"); !value.empty()) {
      options.reference_dir = value;
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--output-dir"); !value.empty()) {
      options.output_dir = value;
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--quality-profile"); !value.empty()) {
      options.quality_profile = parseEnhancementQualityProfile(value);
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--airlight"); !value.empty()) {
      options.airlight_override = std::stoi(value);
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--scale"); !value.empty()) {
      options.scale_override = std::stod(value);
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--frame-step"); !value.empty()) {
      options.frame_step = std::stoi(value);
      continue;
    }
    if (const auto value = getOptionValue(i, argc, argv, "--metric-scale"); !value.empty()) {
      options.metric_scale = std::stod(value);
      continue;
    }

    throw std::runtime_error("unknown argument: " + arg);
  }

  if (options.scale_override.has_value() && *options.scale_override <= 0.0) {
    throw std::runtime_error("--scale must be > 0");
  }
  if (options.frame_step <= 0) {
    throw std::runtime_error("--frame-step must be > 0");
  }
  if (options.metric_scale <= 0.0 || options.metric_scale > 1.0) {
    throw std::runtime_error("--metric-scale must be in (0, 1]");
  }
  return options;
}

EnhancementConfig buildEnhancementConfig(const EvalOptions & options)
{
  EnhancementConfig config = makeEnhancementConfig(options.quality_profile);
  if (options.airlight_override.has_value()) {
    config.manual_airlight = *options.airlight_override;
    config.use_adaptive_airlight = false;
  }
  if (options.scale_override.has_value()) {
    config.scale = *options.scale_override;
  }
  return config;
}

std::optional<std::string> parseBlurId(const fs::path & file_path)
{
  static const std::regex pattern(R"(^(cv_[0-9]+)\.mp4$)", std::regex::icase);
  std::smatch match;
  const std::string name = file_path.filename().string();
  if (!std::regex_match(name, match, pattern)) {
    return std::nullopt;
  }
  return match[1].str();
}

std::optional<std::pair<std::string, std::string>> parseReferenceInfo(const fs::path & file_path)
{
  static const std::regex pattern(R"(^(cv_[0-9]+)_(.+)\.mp4$)", std::regex::icase);
  std::smatch match;
  const std::string name = file_path.filename().string();
  if (!std::regex_match(name, match, pattern)) {
    return std::nullopt;
  }
  return std::make_pair(match[1].str(), match[2].str());
}

std::vector<VideoPair> discoverPairs(
  const fs::path & blur_dir, const fs::path & reference_dir,
  std::vector<SkippedFile> & skipped_blur_files)
{
  if (!fs::exists(blur_dir) || !fs::is_directory(blur_dir)) {
    throw std::runtime_error("blur directory does not exist: " + blur_dir.string());
  }
  if (!fs::exists(reference_dir) || !fs::is_directory(reference_dir)) {
    throw std::runtime_error("reference directory does not exist: " + reference_dir.string());
  }

  std::map<std::string, fs::path> blur_files;
  for (const auto & entry : fs::directory_iterator(blur_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    auto id = parseBlurId(entry.path());
    if (!id.has_value()) {
      skipped_blur_files.push_back({entry.path().filename().string(), "filename_not_supported"});
      continue;
    }
    const auto inserted = blur_files.emplace(*id, entry.path());
    if (!inserted.second) {
      throw std::runtime_error("duplicate blur video id found: " + *id);
    }
  }

  std::map<std::string, std::pair<fs::path, std::string>> reference_files;
  for (const auto & entry : fs::directory_iterator(reference_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    auto reference_info = parseReferenceInfo(entry.path());
    if (!reference_info.has_value()) {
      continue;
    }
    const auto inserted = reference_files.emplace(
      reference_info->first, std::make_pair(entry.path(), reference_info->second));
    if (!inserted.second) {
      throw std::runtime_error("duplicate reference video id found: " + reference_info->first);
    }
  }

  std::vector<VideoPair> pairs;
  for (const auto & [video_id, blur_path] : blur_files) {
    const auto reference_it = reference_files.find(video_id);
    if (reference_it == reference_files.end()) {
      skipped_blur_files.push_back({blur_path.filename().string(), "missing_reference"});
      continue;
    }
    pairs.push_back({
      video_id,
      reference_it->second.second,
      blur_path,
      reference_it->second.first});
  }

  std::sort(
    pairs.begin(), pairs.end(),
    [](const VideoPair & lhs, const VideoPair & rhs) {return lhs.video_id < rhs.video_id;});
  return pairs;
}

cv::Mat ensureBgr8(const cv::Mat & frame)
{
  if (frame.empty()) {
    return frame;
  }
  if (frame.type() == CV_8UC3) {
    return frame;
  }
  if (frame.channels() == 1) {
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  cv::Mat converted;
  frame.convertTo(converted, CV_8UC3);
  return converted;
}

double computePsnr(const cv::Mat & lhs, const cv::Mat & rhs)
{
  cv::Mat lhs_8u = ensureBgr8(lhs);
  cv::Mat rhs_8u = ensureBgr8(rhs);
  cv::Mat diff;
  cv::absdiff(lhs_8u, rhs_8u, diff);
  diff.convertTo(diff, CV_32F);
  diff = diff.mul(diff);
  const cv::Scalar sum_sq = cv::sum(diff);
  const double sse = sum_sq[0] + sum_sq[1] + sum_sq[2];
  const double mse = sse / static_cast<double>(lhs_8u.total() * lhs_8u.channels());
  if (mse <= 1e-12) {
    return 100.0;
  }
  return 10.0 * std::log10((255.0 * 255.0) / mse);
}

double computeSsim(const cv::Mat & lhs, const cv::Mat & rhs)
{
  const cv::Mat lhs_8u = ensureBgr8(lhs);
  const cv::Mat rhs_8u = ensureBgr8(rhs);

  cv::Mat img1;
  cv::Mat img2;
  lhs_8u.convertTo(img1, CV_32F);
  rhs_8u.convertTo(img2, CV_32F);

  cv::Mat mu1;
  cv::Mat mu2;
  cv::GaussianBlur(img1, mu1, cv::Size(11, 11), 1.5);
  cv::GaussianBlur(img2, mu2, cv::Size(11, 11), 1.5);

  const cv::Mat mu1_sq = mu1.mul(mu1);
  const cv::Mat mu2_sq = mu2.mul(mu2);
  const cv::Mat mu1_mu2 = mu1.mul(mu2);

  cv::Mat sigma1_sq;
  cv::Mat sigma2_sq;
  cv::Mat sigma12;
  cv::GaussianBlur(img1.mul(img1), sigma1_sq, cv::Size(11, 11), 1.5);
  sigma1_sq -= mu1_sq;
  cv::GaussianBlur(img2.mul(img2), sigma2_sq, cv::Size(11, 11), 1.5);
  sigma2_sq -= mu2_sq;
  cv::GaussianBlur(img1.mul(img2), sigma12, cv::Size(11, 11), 1.5);
  sigma12 -= mu1_mu2;

  constexpr double k1 = 0.01 * 255.0;
  constexpr double k2 = 0.03 * 255.0;
  constexpr double c1 = k1 * k1;
  constexpr double c2 = k2 * k2;

  cv::Mat numerator = (2.0 * mu1_mu2 + c1).mul(2.0 * sigma12 + c2);
  cv::Mat denominator = (mu1_sq + mu2_sq + c1).mul(sigma1_sq + sigma2_sq + c2);
  cv::Mat ssim_map;
  cv::divide(numerator, denominator, ssim_map);

  const cv::Scalar mssim = cv::mean(ssim_map);
  return (mssim[0] + mssim[1] + mssim[2]) / 3.0;
}

double percentile8uNormalized(const cv::Mat & channel, double quantile)
{
  std::array<int, 256> histogram{};
  const int rows = channel.rows;
  const int cols = channel.cols;
  for (int y = 0; y < rows; ++y) {
    const auto * row_ptr = channel.ptr<uchar>(y);
    for (int x = 0; x < cols; ++x) {
      histogram[row_ptr[x]]++;
    }
  }

  const std::size_t total = channel.total();
  const double target = quantile * static_cast<double>(total - 1);
  std::size_t cumulative = 0;
  for (std::size_t value = 0; value < histogram.size(); ++value) {
    cumulative += static_cast<std::size_t>(histogram[value]);
    if (static_cast<double>(cumulative - 1) >= target) {
      return static_cast<double>(value) / 255.0;
    }
  }
  return 1.0;
}

double computeUciqe(const cv::Mat & frame)
{
  const cv::Mat bgr = ensureBgr8(frame);
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> channels;
  cv::split(lab, channels);

  cv::Mat l_norm;
  channels[0].convertTo(l_norm, CV_32F, 1.0 / 255.0);

  cv::Mat a_float;
  cv::Mat b_float;
  channels[1].convertTo(a_float, CV_32F, 1.0 / 255.0, -128.0 / 255.0);
  channels[2].convertTo(b_float, CV_32F, 1.0 / 255.0, -128.0 / 255.0);

  cv::Mat chroma;
  cv::magnitude(a_float, b_float, chroma);
  const double max_chroma = std::sqrt(2.0 * std::pow(128.0 / 255.0, 2));
  chroma /= max_chroma;

  cv::Scalar chroma_mean;
  cv::Scalar chroma_std;
  cv::meanStdDev(chroma, chroma_mean, chroma_std);

  cv::Mat saturation_denominator;
  cv::sqrt(chroma.mul(chroma) + l_norm.mul(l_norm) + 1e-12, saturation_denominator);
  cv::Mat saturation;
  cv::divide(chroma, saturation_denominator, saturation);
  const double mean_saturation = cv::mean(saturation)[0];

  const double luminance_contrast =
    percentile8uNormalized(channels[0], 0.99) - percentile8uNormalized(channels[0], 0.01);

  return 0.4680 * chroma_std[0] + 0.2745 * luminance_contrast + 0.2576 * mean_saturation;
}

struct TrimmedStats
{
  double mean = 0.0;
  double stddev = 0.0;
};

template<std::size_t BinCount>
TrimmedStats computeTrimmedStatsFromHistogram(
  const std::array<int, BinCount> & histogram,
  int offset,
  double scale,
  std::size_t total_count,
  double trim_ratio)
{
  if (total_count == 0) {
    return {};
  }

  std::size_t trim_each = static_cast<std::size_t>(std::floor(trim_ratio * total_count));
  if (trim_each * 2 >= total_count) {
    trim_each = 0;
  }

  std::size_t remaining_lower_trim = trim_each;
  std::size_t remaining_upper_trim = trim_each;
  double weighted_sum = 0.0;
  double weighted_sq_sum = 0.0;
  std::size_t kept_count = 0;

  for (std::size_t i = 0; i < BinCount; ++i) {
    int count = histogram[i];
    if (count == 0) {
      continue;
    }

    if (remaining_lower_trim > 0) {
      const auto trimmed = std::min<std::size_t>(remaining_lower_trim, static_cast<std::size_t>(count));
      count -= static_cast<int>(trimmed);
      remaining_lower_trim -= trimmed;
    }

    if (count <= 0) {
      continue;
    }

    std::size_t upper_index = BinCount - 1 - i;
    if (upper_index == i) {
      if (remaining_upper_trim > 0) {
        const auto trimmed = std::min<std::size_t>(
          remaining_upper_trim, static_cast<std::size_t>(count));
        count -= static_cast<int>(trimmed);
        remaining_upper_trim -= trimmed;
      }
    }

    if (count <= 0) {
      continue;
    }

    const double value = (static_cast<int>(i) - offset) * scale;
    weighted_sum += value * static_cast<double>(count);
    weighted_sq_sum += value * value * static_cast<double>(count);
    kept_count += static_cast<std::size_t>(count);
  }

  if (kept_count == 0) {
    return {};
  }

  const double mean = weighted_sum / static_cast<double>(kept_count);
  const double variance =
    std::max(0.0, weighted_sq_sum / static_cast<double>(kept_count) - mean * mean);
  return {mean, std::sqrt(variance)};
}

double computeUicm(const cv::Mat & frame)
{
  const cv::Mat bgr = ensureBgr8(frame);
  std::array<int, 511> rg_hist{};
  std::array<int, 1021> yb_hist{};

  for (int y = 0; y < bgr.rows; ++y) {
    const auto * row_ptr = bgr.ptr<cv::Vec3b>(y);
    for (int x = 0; x < bgr.cols; ++x) {
      const int b = row_ptr[x][0];
      const int g = row_ptr[x][1];
      const int r = row_ptr[x][2];
      const int rg = r - g;
      const int yb_twice = r + g - 2 * b;
      rg_hist[static_cast<std::size_t>(rg + 255)]++;
      yb_hist[static_cast<std::size_t>(yb_twice + 510)]++;
    }
  }

  const std::size_t pixel_count = bgr.total();
  const TrimmedStats rg_stats =
    computeTrimmedStatsFromHistogram(rg_hist, 255, 1.0, pixel_count, 0.1);
  const TrimmedStats yb_stats =
    computeTrimmedStatsFromHistogram(yb_hist, 510, 0.5, pixel_count, 0.1);

  const double chroma_bias =
    std::sqrt(rg_stats.mean * rg_stats.mean + yb_stats.mean * yb_stats.mean);
  const double chroma_spread =
    std::sqrt(rg_stats.stddev * rg_stats.stddev + yb_stats.stddev * yb_stats.stddev);
  return -0.0268 * chroma_bias + 0.1586 * chroma_spread;
}

double computeEme(const cv::Mat & single_channel, int block_size)
{
  cv::Mat channel_float;
  single_channel.convertTo(channel_float, CV_32F, 1.0 / 255.0);

  const int block_rows = std::max(1, channel_float.rows / block_size);
  const int block_cols = std::max(1, channel_float.cols / block_size);
  const int height_step = std::max(1, channel_float.rows / block_rows);
  const int width_step = std::max(1, channel_float.cols / block_cols);

  double sum = 0.0;
  int used_blocks = 0;
  for (int row = 0; row < channel_float.rows; row += height_step) {
    for (int col = 0; col < channel_float.cols; col += width_step) {
      const int h = std::min(height_step, channel_float.rows - row);
      const int w = std::min(width_step, channel_float.cols - col);
      const cv::Mat block = channel_float(cv::Rect(col, row, w, h));
      double min_value = 0.0;
      double max_value = 0.0;
      cv::minMaxLoc(block, &min_value, &max_value);
      max_value = std::max(max_value, 1e-6);
      min_value = std::max(min_value, 1e-6);
      if (max_value <= min_value) {
        continue;
      }
      sum += std::log(max_value / min_value);
      used_blocks++;
    }
  }

  if (used_blocks == 0) {
    return 0.0;
  }
  return 2.0 * sum / static_cast<double>(used_blocks);
}

double computeUism(const cv::Mat & frame)
{
  const cv::Mat bgr = ensureBgr8(frame);
  std::vector<cv::Mat> channels;
  cv::split(bgr, channels);

  const std::array<double, 3> weights = {0.1140, 0.5870, 0.2990};
  double uism = 0.0;

  for (std::size_t i = 0; i < channels.size(); ++i) {
    cv::Mat channel_float;
    channels[i].convertTo(channel_float, CV_32F, 1.0 / 255.0);

    cv::Mat grad_x;
    cv::Mat grad_y;
    cv::Sobel(channel_float, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(channel_float, grad_y, CV_32F, 0, 1, 3);

    cv::Mat gradient_magnitude;
    cv::magnitude(grad_x, grad_y, gradient_magnitude);
    cv::Mat sharpness;
    cv::multiply(gradient_magnitude, channel_float, sharpness);

    cv::Mat sharpness_8u;
    sharpness.convertTo(sharpness_8u, CV_8U, 255.0);
    uism += weights[i] * computeEme(sharpness_8u, 8);
  }

  return uism;
}

double computeUiconm(const cv::Mat & frame)
{
  const cv::Mat bgr = ensureBgr8(frame);
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  cv::Mat gray_float;
  gray.convertTo(gray_float, CV_32F, 1.0 / 255.0);

  constexpr int block_size = 8;
  const int block_rows = std::max(1, gray_float.rows / block_size);
  const int block_cols = std::max(1, gray_float.cols / block_size);
  const int height_step = std::max(1, gray_float.rows / block_rows);
  const int width_step = std::max(1, gray_float.cols / block_cols);

  double sum = 0.0;
  int used_blocks = 0;
  for (int row = 0; row < gray_float.rows; row += height_step) {
    for (int col = 0; col < gray_float.cols; col += width_step) {
      const int h = std::min(height_step, gray_float.rows - row);
      const int w = std::min(width_step, gray_float.cols - col);
      const cv::Mat block = gray_float(cv::Rect(col, row, w, h));
      double min_value = 0.0;
      double max_value = 0.0;
      cv::minMaxLoc(block, &min_value, &max_value);
      const double ratio = (max_value - min_value) / std::max(max_value + min_value, 1e-6);
      if (ratio <= 1e-6) {
        continue;
      }
      sum += ratio * std::log(ratio);
      used_blocks++;
    }
  }

  if (used_blocks == 0) {
    return 0.0;
  }
  return -sum / static_cast<double>(used_blocks);
}

double computeUiqm(const cv::Mat & frame)
{
  const double uicm = computeUicm(frame);
  const double uism = computeUism(frame);
  const double uiconm = computeUiconm(frame);
  return 0.0282 * uicm + 0.2953 * uism + 3.5753 * uiconm;
}

FrameMetrics computeFrameMetrics(const cv::Mat & blur_frame, const cv::Mat & enhanced_frame, const cv::Mat & reference_frame)
{
  FrameMetrics metrics;
  metrics.psnr_input_ref = computePsnr(blur_frame, reference_frame);
  metrics.psnr_enhanced_ref = computePsnr(enhanced_frame, reference_frame);
  metrics.delta_psnr = metrics.psnr_enhanced_ref - metrics.psnr_input_ref;

  metrics.ssim_input_ref = computeSsim(blur_frame, reference_frame);
  metrics.ssim_enhanced_ref = computeSsim(enhanced_frame, reference_frame);
  metrics.delta_ssim = metrics.ssim_enhanced_ref - metrics.ssim_input_ref;

  metrics.uciqe_input = computeUciqe(blur_frame);
  metrics.uciqe_enhanced = computeUciqe(enhanced_frame);
  metrics.delta_uciqe = metrics.uciqe_enhanced - metrics.uciqe_input;

  metrics.uiqm_input = computeUiqm(blur_frame);
  metrics.uiqm_enhanced = computeUiqm(enhanced_frame);
  metrics.delta_uiqm = metrics.uiqm_enhanced - metrics.uiqm_input;
  return metrics;
}

PairResult evaluatePair(
  const VideoPair & pair, const EvalOptions & options, const EnhancementProcessor & processor,
  const EnhancementConfig & config, std::ostream & pair_metrics_csv)
{
  PairResult result;
  result.video_id = pair.video_id;
  result.reference_method = pair.reference_method;
  result.blur_path = pair.blur_path;
  result.reference_path = pair.reference_path;

  cv::VideoCapture blur_capture(pair.blur_path.string());
  cv::VideoCapture reference_capture(pair.reference_path.string());
  if (!blur_capture.isOpened()) {
    result.status = "blur_open_failed";
    return result;
  }
  if (!reference_capture.isOpened()) {
    result.status = "reference_open_failed";
    return result;
  }

  result.reported_blur_frames =
    static_cast<std::size_t>(std::max(0.0, blur_capture.get(cv::CAP_PROP_FRAME_COUNT)));
  result.reported_reference_frames =
    static_cast<std::size_t>(std::max(0.0, reference_capture.get(cv::CAP_PROP_FRAME_COUNT)));

  const auto pair_start_time = std::chrono::steady_clock::now();
  constexpr std::size_t progress_log_interval = 25;

  std::size_t source_frame_index = 0;
  std::size_t evaluated_frame_count = 0;
  bool blur_ok = false;
  bool reference_ok = false;
  do {
    cv::Mat blur_frame;
    cv::Mat reference_frame;
    blur_ok = blur_capture.read(blur_frame);
    reference_ok = reference_capture.read(reference_frame);
    if (!blur_ok || !reference_ok) {
      break;
    }

    blur_frame = ensureBgr8(blur_frame);
    reference_frame = ensureBgr8(reference_frame);

    if ((source_frame_index % static_cast<std::size_t>(options.frame_step)) != 0U) {
      source_frame_index++;
      continue;
    }

    cv::Mat enhanced_frame = processor.process(blur_frame, config);
    cv::Mat blur_for_metrics = blur_frame;
    cv::Mat enhanced_for_metrics = enhanced_frame;

    if (blur_for_metrics.size() != reference_frame.size()) {
      cv::resize(
        blur_for_metrics, blur_for_metrics, reference_frame.size(), 0.0, 0.0, cv::INTER_LINEAR);
      result.resized_to_reference = true;
    }
    if (enhanced_for_metrics.size() != reference_frame.size()) {
      cv::resize(
        enhanced_for_metrics, enhanced_for_metrics, reference_frame.size(), 0.0, 0.0,
        cv::INTER_LINEAR);
      result.resized_to_reference = true;
    }

    if (options.metric_scale < 1.0) {
      const cv::Size metric_size(
        std::max(1, static_cast<int>(std::lround(reference_frame.cols * options.metric_scale))),
        std::max(1, static_cast<int>(std::lround(reference_frame.rows * options.metric_scale))));
      if (metric_size != reference_frame.size()) {
        cv::resize(
          blur_for_metrics, blur_for_metrics, metric_size, 0.0, 0.0, cv::INTER_LINEAR);
        cv::resize(
          enhanced_for_metrics, enhanced_for_metrics, metric_size, 0.0, 0.0,
          cv::INTER_LINEAR);
        cv::resize(
          reference_frame, reference_frame, metric_size, 0.0, 0.0, cv::INTER_LINEAR);
      }
    }

    const FrameMetrics metrics =
      computeFrameMetrics(blur_for_metrics, enhanced_for_metrics, reference_frame);
    result.metrics.add(metrics);

    pair_metrics_csv
      << quoteCsv(result.video_id) << ','
      << quoteCsv(result.reference_method) << ','
      << source_frame_index << ','
      << std::fixed << std::setprecision(6)
      << metrics.psnr_input_ref << ','
      << metrics.psnr_enhanced_ref << ','
      << metrics.delta_psnr << ','
      << metrics.ssim_input_ref << ','
      << metrics.ssim_enhanced_ref << ','
      << metrics.delta_ssim << ','
      << metrics.uciqe_input << ','
      << metrics.uciqe_enhanced << ','
      << metrics.delta_uciqe << ','
      << metrics.uiqm_input << ','
      << metrics.uiqm_enhanced << ','
      << metrics.delta_uiqm << '\n';

    evaluated_frame_count++;
    if ((evaluated_frame_count % progress_log_interval) == 0U) {
      pair_metrics_csv.flush();
      std::cout << "    " << result.video_id
                << ": evaluated " << evaluated_frame_count
                << " sampled frames, latest source frame " << source_frame_index
                << ", elapsed " << std::fixed << std::setprecision(1)
                << elapsedSeconds(pair_start_time) << "s\n";
    }

    source_frame_index++;
  } while (true);

  pair_metrics_csv.flush();
  result.frames_evaluated = evaluated_frame_count;
  result.truncated_pair = blur_ok != reference_ok;
  if (result.frames_evaluated == 0) {
    result.status = "no_frames_evaluated";
  } else if (result.truncated_pair) {
    result.status = "truncated_pair";
  }
  return result;
}

void writeVideoSummaryRow(std::ostream & out, const PairResult & result)
{
  out
    << quoteCsv(result.video_id) << ','
    << quoteCsv(result.reference_method) << ','
    << quoteCsv(result.blur_path.string()) << ','
    << quoteCsv(result.reference_path.string()) << ','
    << quoteCsv(result.status) << ','
    << result.reported_blur_frames << ','
    << result.reported_reference_frames << ','
    << result.frames_evaluated << ','
    << (result.truncated_pair ? "true" : "false") << ','
    << (result.resized_to_reference ? "true" : "false") << ','
    << std::fixed << std::setprecision(6)
    << result.metrics.psnr_input_ref.mean() << ','
    << result.metrics.psnr_enhanced_ref.mean() << ','
    << result.metrics.delta_psnr.mean() << ','
    << result.metrics.ssim_input_ref.mean() << ','
    << result.metrics.ssim_enhanced_ref.mean() << ','
    << result.metrics.delta_ssim.mean() << ','
    << result.metrics.uciqe_input.mean() << ','
    << result.metrics.uciqe_enhanced.mean() << ','
    << result.metrics.delta_uciqe.mean() << ','
    << result.metrics.uiqm_input.mean() << ','
    << result.metrics.uiqm_enhanced.mean() << ','
    << result.metrics.delta_uiqm.mean() << '\n';
}

void writeMethodSummaryRow(
  std::ostream & out, const std::string & method, const DatasetAccumulator & dataset)
{
  out
    << quoteCsv(method) << ','
    << dataset.pair_count << ','
    << dataset.frame_count << ','
    << std::fixed << std::setprecision(6)
    << dataset.metrics.psnr_input_ref.mean() << ','
    << dataset.metrics.psnr_input_ref.stddev() << ','
    << (dataset.metrics.psnr_input_ref.count == 0 ? 0.0 : dataset.metrics.psnr_input_ref.min) << ','
    << (dataset.metrics.psnr_input_ref.count == 0 ? 0.0 : dataset.metrics.psnr_input_ref.max) << ','
    << dataset.metrics.psnr_enhanced_ref.mean() << ','
    << dataset.metrics.psnr_enhanced_ref.stddev() << ','
    << (dataset.metrics.psnr_enhanced_ref.count == 0 ? 0.0 : dataset.metrics.psnr_enhanced_ref.min) << ','
    << (dataset.metrics.psnr_enhanced_ref.count == 0 ? 0.0 : dataset.metrics.psnr_enhanced_ref.max) << ','
    << dataset.metrics.delta_psnr.mean() << ','
    << dataset.metrics.delta_psnr.stddev() << ','
    << (dataset.metrics.delta_psnr.count == 0 ? 0.0 : dataset.metrics.delta_psnr.min) << ','
    << (dataset.metrics.delta_psnr.count == 0 ? 0.0 : dataset.metrics.delta_psnr.max) << ','
    << dataset.metrics.ssim_input_ref.mean() << ','
    << dataset.metrics.ssim_input_ref.stddev() << ','
    << (dataset.metrics.ssim_input_ref.count == 0 ? 0.0 : dataset.metrics.ssim_input_ref.min) << ','
    << (dataset.metrics.ssim_input_ref.count == 0 ? 0.0 : dataset.metrics.ssim_input_ref.max) << ','
    << dataset.metrics.ssim_enhanced_ref.mean() << ','
    << dataset.metrics.ssim_enhanced_ref.stddev() << ','
    << (dataset.metrics.ssim_enhanced_ref.count == 0 ? 0.0 : dataset.metrics.ssim_enhanced_ref.min) << ','
    << (dataset.metrics.ssim_enhanced_ref.count == 0 ? 0.0 : dataset.metrics.ssim_enhanced_ref.max) << ','
    << dataset.metrics.delta_ssim.mean() << ','
    << dataset.metrics.delta_ssim.stddev() << ','
    << (dataset.metrics.delta_ssim.count == 0 ? 0.0 : dataset.metrics.delta_ssim.min) << ','
    << (dataset.metrics.delta_ssim.count == 0 ? 0.0 : dataset.metrics.delta_ssim.max) << ','
    << dataset.metrics.uciqe_input.mean() << ','
    << dataset.metrics.uciqe_input.stddev() << ','
    << (dataset.metrics.uciqe_input.count == 0 ? 0.0 : dataset.metrics.uciqe_input.min) << ','
    << (dataset.metrics.uciqe_input.count == 0 ? 0.0 : dataset.metrics.uciqe_input.max) << ','
    << dataset.metrics.uciqe_enhanced.mean() << ','
    << dataset.metrics.uciqe_enhanced.stddev() << ','
    << (dataset.metrics.uciqe_enhanced.count == 0 ? 0.0 : dataset.metrics.uciqe_enhanced.min) << ','
    << (dataset.metrics.uciqe_enhanced.count == 0 ? 0.0 : dataset.metrics.uciqe_enhanced.max) << ','
    << dataset.metrics.delta_uciqe.mean() << ','
    << dataset.metrics.delta_uciqe.stddev() << ','
    << (dataset.metrics.delta_uciqe.count == 0 ? 0.0 : dataset.metrics.delta_uciqe.min) << ','
    << (dataset.metrics.delta_uciqe.count == 0 ? 0.0 : dataset.metrics.delta_uciqe.max) << ','
    << dataset.metrics.uiqm_input.mean() << ','
    << dataset.metrics.uiqm_input.stddev() << ','
    << (dataset.metrics.uiqm_input.count == 0 ? 0.0 : dataset.metrics.uiqm_input.min) << ','
    << (dataset.metrics.uiqm_input.count == 0 ? 0.0 : dataset.metrics.uiqm_input.max) << ','
    << dataset.metrics.uiqm_enhanced.mean() << ','
    << dataset.metrics.uiqm_enhanced.stddev() << ','
    << (dataset.metrics.uiqm_enhanced.count == 0 ? 0.0 : dataset.metrics.uiqm_enhanced.min) << ','
    << (dataset.metrics.uiqm_enhanced.count == 0 ? 0.0 : dataset.metrics.uiqm_enhanced.max) << ','
    << dataset.metrics.delta_uiqm.mean() << ','
    << dataset.metrics.delta_uiqm.stddev() << ','
    << (dataset.metrics.delta_uiqm.count == 0 ? 0.0 : dataset.metrics.delta_uiqm.min) << ','
    << (dataset.metrics.delta_uiqm.count == 0 ? 0.0 : dataset.metrics.delta_uiqm.max) << '\n';
}

void writeOverallSummaryJson(
  const fs::path & output_path, const EvalOptions & options, const std::vector<SkippedFile> & skipped_blur_files,
  const std::vector<PairResult> & results, const DatasetAccumulator & overall,
  const std::map<std::string, DatasetAccumulator> & method_summaries)
{
  std::ofstream out(output_path);
  if (!out.is_open()) {
    throw std::runtime_error("failed to open summary json: " + output_path.string());
  }

  out << "{\n";
  out << "  \"blur_dir\": \"" << jsonEscape(options.blur_dir.string()) << "\",\n";
  out << "  \"reference_dir\": \"" << jsonEscape(options.reference_dir.string()) << "\",\n";
  out << "  \"output_dir\": \"" << jsonEscape(options.output_dir.string()) << "\",\n";
  out << "  \"parameters\": {\n";
  out << "    \"quality_profile\": \"" << jsonEscape(toString(options.quality_profile)) << "\",\n";
  if (options.airlight_override.has_value()) {
    out << "    \"airlight_override\": " << *options.airlight_override << ",\n";
  } else {
    out << "    \"airlight_override\": null,\n";
  }
  if (options.scale_override.has_value()) {
    out << "    \"scale_override\": " << *options.scale_override << ",\n";
  } else {
    out << "    \"scale_override\": null,\n";
  }
  out << "    \"frame_step\": " << options.frame_step << ",\n";
  out << "    \"metric_scale\": " << options.metric_scale << "\n";
  out << "  },\n";
  out << "  \"matched_pairs\": " << results.size() << ",\n";
  out << "  \"skipped_blur_files\": [\n";
  for (std::size_t i = 0; i < skipped_blur_files.size(); ++i) {
    const auto & skipped = skipped_blur_files[i];
    out << "    {\"file_name\": \"" << jsonEscape(skipped.file_name)
        << "\", \"reason\": \"" << jsonEscape(skipped.reason) << "\"}";
    if (i + 1 != skipped_blur_files.size()) {
      out << ",";
    }
    out << "\n";
  }
  out << "  ],\n";
  out << "  \"overall\": {\n";
  out << "    \"pair_count\": " << overall.pair_count << ",\n";
  out << "    \"frame_count\": " << overall.frame_count << ",\n";
  writeMetricJson(out, "psnr_input_ref", overall.metrics.psnr_input_ref, false);
  writeMetricJson(out, "psnr_enhanced_ref", overall.metrics.psnr_enhanced_ref, false);
  writeMetricJson(out, "delta_psnr", overall.metrics.delta_psnr, false);
  writeMetricJson(out, "ssim_input_ref", overall.metrics.ssim_input_ref, false);
  writeMetricJson(out, "ssim_enhanced_ref", overall.metrics.ssim_enhanced_ref, false);
  writeMetricJson(out, "delta_ssim", overall.metrics.delta_ssim, false);
  writeMetricJson(out, "uciqe_input", overall.metrics.uciqe_input, false);
  writeMetricJson(out, "uciqe_enhanced", overall.metrics.uciqe_enhanced, false);
  writeMetricJson(out, "delta_uciqe", overall.metrics.delta_uciqe, false);
  writeMetricJson(out, "uiqm_input", overall.metrics.uiqm_input, false);
  writeMetricJson(out, "uiqm_enhanced", overall.metrics.uiqm_enhanced, false);
  writeMetricJson(out, "delta_uiqm", overall.metrics.delta_uiqm, true);
  out << "  },\n";
  out << "  \"methods\": {\n";
  std::size_t method_index = 0;
  for (const auto & [method, summary] : method_summaries) {
    out << "    \"" << jsonEscape(method) << "\": {\n";
    out << "      \"pair_count\": " << summary.pair_count << ",\n";
    out << "      \"frame_count\": " << summary.frame_count << ",\n";
    writeMetricJson(out, "psnr_input_ref", summary.metrics.psnr_input_ref, false);
    writeMetricJson(out, "psnr_enhanced_ref", summary.metrics.psnr_enhanced_ref, false);
    writeMetricJson(out, "delta_psnr", summary.metrics.delta_psnr, false);
    writeMetricJson(out, "ssim_input_ref", summary.metrics.ssim_input_ref, false);
    writeMetricJson(out, "ssim_enhanced_ref", summary.metrics.ssim_enhanced_ref, false);
    writeMetricJson(out, "delta_ssim", summary.metrics.delta_ssim, false);
    writeMetricJson(out, "uciqe_input", summary.metrics.uciqe_input, false);
    writeMetricJson(out, "uciqe_enhanced", summary.metrics.uciqe_enhanced, false);
    writeMetricJson(out, "delta_uciqe", summary.metrics.delta_uciqe, false);
    writeMetricJson(out, "uiqm_input", summary.metrics.uiqm_input, false);
    writeMetricJson(out, "uiqm_enhanced", summary.metrics.uiqm_enhanced, false);
    writeMetricJson(out, "delta_uiqm", summary.metrics.delta_uiqm, true);
    out << "    }";
    if (++method_index != method_summaries.size()) {
      out << ",";
    }
    out << "\n";
  }
  out << "  }\n";
  out << "}\n";
}

}  // namespace

int runEvaluation(int argc, char ** argv)
{
  const EvalOptions options = parseArgs(argc, argv);
  std::vector<SkippedFile> skipped_blur_files;
  const std::vector<VideoPair> pairs =
    discoverPairs(options.blur_dir, options.reference_dir, skipped_blur_files);
  if (pairs.empty()) {
    throw std::runtime_error("no matched blur/reference video pairs found");
  }

  fs::create_directories(options.output_dir);

  const fs::path pair_metrics_path = options.output_dir / "pair_metrics.csv";
  const fs::path video_summary_path = options.output_dir / "video_summary.csv";
  const fs::path method_summary_path = options.output_dir / "method_summary.csv";
  const fs::path overall_summary_path = options.output_dir / "overall_summary.json";

  std::ofstream pair_metrics_csv(pair_metrics_path);
  std::ofstream video_summary_csv(video_summary_path);
  std::ofstream method_summary_csv(method_summary_path);
  if (!pair_metrics_csv.is_open() || !video_summary_csv.is_open() || !method_summary_csv.is_open()) {
    throw std::runtime_error("failed to open one or more output files under " + options.output_dir.string());
  }

  writeMetricCsvHeader(pair_metrics_csv);
  writeVideoSummaryHeader(video_summary_csv);
  writeMethodSummaryHeader(method_summary_csv);
  pair_metrics_csv.flush();
  video_summary_csv.flush();
  method_summary_csv.flush();

  EnhancementProcessor processor;
  const EnhancementConfig config = buildEnhancementConfig(options);
  std::vector<PairResult> results;
  results.reserve(pairs.size());
  DatasetAccumulator overall_summary;
  std::map<std::string, DatasetAccumulator> method_summaries;
  std::vector<std::string> failed_pairs;
  const auto overall_start_time = std::chrono::steady_clock::now();

  std::cout << "Matched pairs discovered: " << pairs.size() << "\n"
            << "Quality profile: " << toString(options.quality_profile) << "\n"
            << "Frame step: " << options.frame_step << "\n"
            << "Metric scale: " << options.metric_scale << "\n"
            << "Output dir: " << options.output_dir << "\n";

  for (std::size_t pair_index = 0; pair_index < pairs.size(); ++pair_index) {
    const auto & pair = pairs[pair_index];
    const auto pair_start_time = std::chrono::steady_clock::now();
    std::cout << "[" << (pair_index + 1) << "/" << pairs.size() << "] "
              << pair.video_id << " (" << pair.reference_method << ")\n";
    PairResult result = evaluatePair(pair, options, processor, config, pair_metrics_csv);
    if (result.frames_evaluated == 0) {
      failed_pairs.push_back(pair.video_id + ":" + result.status);
    } else {
      overall_summary.addVideo(result);
      method_summaries[result.reference_method].addVideo(result);
    }
    writeVideoSummaryRow(video_summary_csv, result);
    video_summary_csv.flush();
    results.push_back(std::move(result));

    std::cout << "  -> status=" << results.back().status
              << ", sampled_frames=" << results.back().frames_evaluated
              << ", elapsed=" << std::fixed << std::setprecision(1)
              << elapsedSeconds(pair_start_time) << "s"
              << ", total_elapsed=" << elapsedSeconds(overall_start_time) << "s\n";
  }

  for (const auto & [method, summary] : method_summaries) {
    writeMethodSummaryRow(method_summary_csv, method, summary);
  }
  method_summary_csv.flush();

  writeOverallSummaryJson(
    overall_summary_path, options, skipped_blur_files, results, overall_summary, method_summaries);

  std::cout << std::fixed << std::setprecision(6)
            << "Matched pairs: " << pairs.size() << "\n"
            << "Skipped blur files: " << skipped_blur_files.size() << "\n"
            << "Successful pairs: " << overall_summary.pair_count << "\n"
            << "Failed pairs: " << failed_pairs.size() << "\n"
            << "Mean PSNR(input,ref): " << overall_summary.metrics.psnr_input_ref.mean() << "\n"
            << "Mean PSNR(enhanced,ref): " << overall_summary.metrics.psnr_enhanced_ref.mean() << "\n"
            << "Mean delta PSNR: " << overall_summary.metrics.delta_psnr.mean() << "\n"
            << "Mean SSIM(input,ref): " << overall_summary.metrics.ssim_input_ref.mean() << "\n"
            << "Mean SSIM(enhanced,ref): " << overall_summary.metrics.ssim_enhanced_ref.mean() << "\n"
            << "Mean delta SSIM: " << overall_summary.metrics.delta_ssim.mean() << "\n"
            << "Mean UCIQE(input): " << overall_summary.metrics.uciqe_input.mean() << "\n"
            << "Mean UCIQE(enhanced): " << overall_summary.metrics.uciqe_enhanced.mean() << "\n"
            << "Mean delta UCIQE: " << overall_summary.metrics.delta_uciqe.mean() << "\n"
            << "Mean UIQM(input): " << overall_summary.metrics.uiqm_input.mean() << "\n"
            << "Mean UIQM(enhanced): " << overall_summary.metrics.uiqm_enhanced.mean() << "\n"
            << "Mean delta UIQM: " << overall_summary.metrics.delta_uiqm.mean() << "\n"
            << "Outputs written to: " << options.output_dir << "\n";

  if (!failed_pairs.empty()) {
    std::cerr << "Failed pairs:\n";
    for (const auto & item : failed_pairs) {
      std::cerr << "  - " << item << "\n";
    }
    return 1;
  }

  return 0;
}

}  // namespace asr_sdm_video_enhancement

int main(int argc, char ** argv)
{
  try {
    return asr_sdm_video_enhancement::runEvaluation(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Evaluation failed: " << error.what() << std::endl;
    return 1;
  }
}
