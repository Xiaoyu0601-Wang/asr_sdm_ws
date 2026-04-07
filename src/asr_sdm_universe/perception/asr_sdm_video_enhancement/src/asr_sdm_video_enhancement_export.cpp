#include "asr_sdm_video_enhancement/enhancement_processor.hpp"

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace fs = std::filesystem;

namespace asr_sdm_video_enhancement
{

namespace
{

struct ExportOptions
{
  fs::path blur_dir = "/home/cortin/Downloads/test/blur";
  fs::path reference_dir = "/home/cortin/Downloads/test/gt";
  fs::path output_dir = "/home/cortin/Downloads/test/output";
  EnhancementQualityProfile quality_profile = EnhancementQualityProfile::Default;
  std::optional<int> airlight_override;
  std::optional<double> scale_override;
};

struct VideoPair
{
  std::string video_id;
  fs::path blur_path;
  fs::path reference_path;
};

struct SkippedFile
{
  std::string video_id;
  fs::path blur_path;
  std::string reason;
};

struct ExportRecord
{
  std::string video_id;
  fs::path blur_path;
  fs::path reference_path;
  fs::path output_path;
  std::string status = "ok";
  std::size_t frames_written = 0;
  double fps = 0.0;
  int width = 0;
  int height = 0;
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

void writeSummaryHeader(std::ostream & out)
{
  out << "video_id,blur_path,reference_path,output_path,status,frames_written,fps,width,height\n";
}

void writeSummaryRow(std::ostream & out, const ExportRecord & record)
{
  out << quoteCsv(record.video_id) << ','
      << quoteCsv(record.blur_path.string()) << ','
      << quoteCsv(record.reference_path.string()) << ','
      << quoteCsv(record.output_path.string()) << ','
      << quoteCsv(record.status) << ','
      << record.frames_written << ','
      << std::fixed << std::setprecision(6) << record.fps << ','
      << record.width << ','
      << record.height << '\n';
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
    << "Usage: asr_sdm_video_enhancement_export [options]\n"
    << "  --blur-dir PATH         Default: /home/cortin/Downloads/test/blur\n"
    << "  --reference-dir PATH    Default: /home/cortin/Downloads/test/gt\n"
    << "  --output-dir PATH       Default: /home/cortin/Downloads/test/output\n"
    << "  --quality-profile NAME  Default: default (default|offline_gt_like)\n"
    << "  --airlight INT          Optional manual override; disables adaptive airlight\n"
    << "  --scale FLOAT           Optional scale override\n";
}

ExportOptions parseArgs(int argc, char ** argv)
{
  ExportOptions options;
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

    throw std::runtime_error("unknown argument: " + arg);
  }

  if (options.scale_override.has_value() && *options.scale_override <= 0.0) {
    throw std::runtime_error("--scale must be > 0");
  }
  return options;
}

EnhancementConfig buildEnhancementConfig(const ExportOptions & options)
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

std::optional<std::string> parseReferenceId(const fs::path & file_path)
{
  static const std::regex pattern(R"(^(cv_[0-9]+)_(.+)\.mp4$)", std::regex::icase);
  std::smatch match;
  const std::string name = file_path.filename().string();
  if (!std::regex_match(name, match, pattern)) {
    return std::nullopt;
  }
  return match[1].str();
}

std::vector<VideoPair> discoverPairs(
  const fs::path & blur_dir, const fs::path & reference_dir, std::vector<SkippedFile> & skipped_blur_files)
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

    const auto blur_id = parseBlurId(entry.path());
    if (!blur_id.has_value()) {
      skipped_blur_files.push_back({"", entry.path(), "filename_not_supported"});
      continue;
    }

    const auto inserted = blur_files.emplace(*blur_id, entry.path());
    if (!inserted.second) {
      throw std::runtime_error("duplicate blur video id found: " + *blur_id);
    }
  }

  std::map<std::string, fs::path> reference_files;
  for (const auto & entry : fs::directory_iterator(reference_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto reference_id = parseReferenceId(entry.path());
    if (!reference_id.has_value()) {
      continue;
    }

    const auto inserted = reference_files.emplace(*reference_id, entry.path());
    if (!inserted.second) {
      throw std::runtime_error("duplicate reference video id found: " + *reference_id);
    }
  }

  std::vector<VideoPair> pairs;
  for (const auto & [video_id, blur_path] : blur_files) {
    const auto reference_it = reference_files.find(video_id);
    if (reference_it == reference_files.end()) {
      skipped_blur_files.push_back({video_id, blur_path, "missing_reference"});
      continue;
    }

    pairs.push_back({video_id, blur_path, reference_it->second});
  }

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

ExportRecord exportPair(
  const VideoPair & pair, const ExportOptions & options, const EnhancementProcessor & processor,
  const EnhancementConfig & config)
{
  ExportRecord record;
  record.video_id = pair.video_id;
  record.blur_path = pair.blur_path;
  record.reference_path = pair.reference_path;
  record.output_path = options.output_dir / pair.blur_path.filename();
  const fs::path temp_output_path =
    options.output_dir / (pair.blur_path.stem().string() + ".partial.mp4");

  cv::VideoCapture blur_capture(pair.blur_path.string());
  if (!blur_capture.isOpened()) {
    record.status = "blur_open_failed";
    return record;
  }

  double fps = blur_capture.get(cv::CAP_PROP_FPS);
  bool used_fallback_fps = false;
  if (fps <= 0.0) {
    fps = 30.0;
    used_fallback_fps = true;
  }
  record.fps = fps;

  cv::VideoWriter writer;
  cv::Size original_size;
  std::size_t frames_written = 0;

  std::error_code remove_error;
  fs::remove(record.output_path, remove_error);
  fs::remove(temp_output_path, remove_error);

  while (true) {
    cv::Mat blur_frame;
    if (!blur_capture.read(blur_frame)) {
      break;
    }

    blur_frame = ensureBgr8(blur_frame);
    if (blur_frame.empty()) {
      continue;
    }

    if (original_size.width == 0 || original_size.height == 0) {
      original_size = blur_frame.size();
      record.width = original_size.width;
      record.height = original_size.height;

      writer.open(
        temp_output_path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
        original_size, true);
      if (!writer.isOpened()) {
        record.status = "writer_open_failed";
        return record;
      }
    }

    cv::Mat enhanced_frame = processor.process(blur_frame, config);
    enhanced_frame = ensureBgr8(enhanced_frame);
    if (enhanced_frame.empty()) {
      record.status = "empty_processed_frame";
      return record;
    }

    if (enhanced_frame.size() != original_size) {
      cv::resize(
        enhanced_frame, enhanced_frame, original_size, 0.0, 0.0, cv::INTER_LINEAR);
    }

    writer.write(enhanced_frame);
    frames_written++;
  }

  record.frames_written = frames_written;
  if (frames_written == 0) {
    writer.release();
    fs::remove(temp_output_path, remove_error);
    record.status = "no_frames_written";
  } else if (used_fallback_fps) {
    writer.release();
    fs::rename(temp_output_path, record.output_path, remove_error);
    if (remove_error) {
      fs::remove(temp_output_path, remove_error);
      record.status = "rename_failed";
      return record;
    }
    record.status = "ok_fallback_fps";
  } else {
    writer.release();
    fs::rename(temp_output_path, record.output_path, remove_error);
    if (remove_error) {
      fs::remove(temp_output_path, remove_error);
      record.status = "rename_failed";
      return record;
    }
  }

  return record;
}

int runExport(int argc, char ** argv)
{
  const ExportOptions options = parseArgs(argc, argv);
  std::vector<SkippedFile> skipped_blur_files;
  std::vector<VideoPair> pairs = discoverPairs(options.blur_dir, options.reference_dir, skipped_blur_files);
  if (pairs.empty()) {
    throw std::runtime_error("no matched blur/reference video pairs found");
  }

  fs::create_directories(options.output_dir);

  const fs::path summary_path = options.output_dir / "export_summary.csv";
  std::ofstream summary_csv(summary_path);
  if (!summary_csv.is_open()) {
    throw std::runtime_error("failed to open export summary: " + summary_path.string());
  }
  writeSummaryHeader(summary_csv);
  summary_csv.flush();

  for (const auto & skipped : skipped_blur_files) {
    ExportRecord record;
    record.video_id = skipped.video_id;
    record.blur_path = skipped.blur_path;
    record.status = skipped.reason;
    writeSummaryRow(summary_csv, record);
    summary_csv.flush();
  }

  EnhancementProcessor processor;
  const EnhancementConfig config = buildEnhancementConfig(options);
  std::size_t success_count = 0;
  std::size_t failed_count = 0;

  for (const auto & pair : pairs) {
    ExportRecord record = exportPair(pair, options, processor, config);
    if (record.status == "ok" || record.status == "ok_fallback_fps") {
      success_count++;
    } else {
      failed_count++;
    }
    writeSummaryRow(summary_csv, record);
    summary_csv.flush();
  }

  std::cout << "Matched pairs: " << pairs.size() << "\n"
            << "Skipped blur files: " << skipped_blur_files.size() << "\n"
            << "Successful exports: " << success_count << "\n"
            << "Failed exports: " << failed_count << "\n"
            << "Quality profile: " << toString(options.quality_profile) << "\n"
            << "Outputs written to: " << options.output_dir << "\n"
            << "Summary written to: " << summary_path << "\n";

  return failed_count == 0 ? 0 : 1;
}

}  // namespace

}  // namespace asr_sdm_video_enhancement

int main(int argc, char ** argv)
{
  try {
    return asr_sdm_video_enhancement::runExport(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Export failed: " << error.what() << std::endl;
    return 1;
  }
}
