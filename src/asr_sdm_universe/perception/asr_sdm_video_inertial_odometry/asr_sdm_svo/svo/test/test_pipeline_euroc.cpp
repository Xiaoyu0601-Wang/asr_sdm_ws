#include "test_utils.h"

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

namespace svo
{

class EuRoCBenchmarkNode
{
  vk::AbstractCamera * cam_;
  svo::FrameHandlerMono * vo_;

public:
  EuRoCBenchmarkNode();
  ~EuRoCBenchmarkNode();
  void runFromEuRoCDataset(const std::string& dataset_path);
};

EuRoCBenchmarkNode::EuRoCBenchmarkNode()
{
  // EuRoC camera parameters (cam0)
  // From: https://github.com/ethz-asl/kalibr/wiki/yaml-formats
  cam_ = new vk::PinholeCamera(752, 480, 458.654, 457.296, 367.215, 248.375);
  
  // Configure SVO for EuRoC - OPTIMIZED PARAMETERS FOR STABILITY
  svo::Config::gridSize() = 30;                    // Even larger grid for more features
  svo::Config::nPyrLevels() = 4;
  svo::Config::kltMaxLevel() = 4;
  svo::Config::kltMinLevel() = 2;
  
  // Feature detection - More aggressive
  svo::Config::triangMinCornerScore() = 10.0;      // Much lower threshold for more features
  svo::Config::maxFts() = 300;                     // Significantly increase max features
  
  // Tracking quality - More tolerant
  svo::Config::qualityMinFts() = 40;               // Lower minimum (was 50)
  svo::Config::qualityMaxFtsDrop() = 60;           // Allow more drops (was 40)
  
  // Reprojection - More tolerant
  svo::Config::reprojThresh() = 4.0;               // Looser threshold (was 3.0)
  
  // Keyframe selection - More frequent
  svo::Config::kfSelectMinDist() = 0.05;           // Even more frequent keyframes
  svo::Config::maxNKfs() = 20;                     // Allow more keyframes in map
  
  // Map scale
  svo::Config::mapScale() = 3.0;                   // Adjust map scale
  
  std::cout << "SVO Configuration:" << std::endl;
  std::cout << "  Grid size: " << svo::Config::gridSize() << std::endl;
  std::cout << "  Max features: " << svo::Config::maxFts() << std::endl;
  std::cout << "  Min corner score: " << svo::Config::triangMinCornerScore() << std::endl;
  std::cout << "  Quality min features: " << svo::Config::qualityMinFts() << std::endl;
  std::cout << "  Quality max drop: " << svo::Config::qualityMaxFtsDrop() << std::endl;
  std::cout << "  Reproj threshold: " << svo::Config::reprojThresh() << std::endl;
  std::cout << "  KF min dist: " << svo::Config::kfSelectMinDist() << std::endl;
  std::cout << "  Max KFs: " << svo::Config::maxNKfs() << std::endl;
  
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

EuRoCBenchmarkNode::~EuRoCBenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void EuRoCBenchmarkNode::runFromEuRoCDataset(const std::string& dataset_path)
{
  // Read image timestamps
  std::string data_csv = dataset_path + "/mav0/cam0/data.csv";
  std::ifstream csv_file(data_csv);
  if (!csv_file.is_open()) {
    std::cerr << "ERROR: Cannot open " << data_csv << std::endl;
    return;
  }

  std::vector<std::string> image_files;
  std::vector<double> timestamps;
  
  std::string line;
  std::getline(csv_file, line); // Skip header
  
  int count = 0;
  while (std::getline(csv_file, line) && count < 200) {  // Process first 200 frames
    if (line.empty() || line[0] == '#') continue;
    
    size_t comma_pos = line.find(',');
    if (comma_pos == std::string::npos) continue;
    
    std::string timestamp_str = line.substr(0, comma_pos);
    std::string filename = line.substr(comma_pos + 1);
    
    // Remove any whitespace or newlines
    filename.erase(std::remove_if(filename.begin(), filename.end(), ::isspace), filename.end());
    
    image_files.push_back(dataset_path + "/mav0/cam0/data/" + filename);
    timestamps.push_back(std::stod(timestamp_str) / 1e9);  // Convert nanoseconds to seconds
    count++;
  }
  csv_file.close();

  std::cout << "Found " << image_files.size() << " images in dataset" << std::endl;

  // Process images
  for (size_t i = 0; i < image_files.size(); ++i) {
    // Load image
    cv::Mat img = cv::imread(image_files[i], cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::cerr << "ERROR: Failed to load image: " << image_files[i] << std::endl;
      continue;
    }

    if (i == 0) {
      std::cout << "First image: " << image_files[i] << std::endl;
      std::cout << "Image size: " << img.cols << "x" << img.rows << std::endl;
    }

    // Process frame
    vo_->addImage(img, timestamps[i]);

    // Display tracking quality
    if (vo_->lastFrame() != NULL) {
      std::cout << "Frame-Id: " << std::setw(4) << vo_->lastFrame()->id_ << " \t"
                << "#Features: " << std::setw(4) << vo_->lastNumObservations() << " \t"
                << "Proc. Time: " << std::setw(6) << std::fixed << std::setprecision(2) 
                << vo_->lastProcessingTime() * 1000 << " ms";
      
      // Show stage
      if (vo_->stage() == svo::FrameHandlerMono::STAGE_PAUSED)
        std::cout << " \t[PAUSED]";
      else if (vo_->stage() == svo::FrameHandlerMono::STAGE_FIRST_FRAME)
        std::cout << " \t[FIRST_FRAME]";
      else if (vo_->stage() == svo::FrameHandlerMono::STAGE_SECOND_FRAME)
        std::cout << " \t[SECOND_FRAME]";
      else if (vo_->stage() == svo::FrameHandlerMono::STAGE_DEFAULT_FRAME)
        std::cout << " \t[TRACKING]";
      else if (vo_->stage() == svo::FrameHandlerMono::STAGE_RELOCALIZING)
        std::cout << " \t[RELOCALIZING]";
      
      // Show number of keyframes
      if (vo_->map().size() > 0) {
        std::cout << " \tKFs: " << vo_->map().size();
      }
      
      std::cout << std::endl;
    }
  }
}

}  // namespace svo

int main(int argc, char ** argv)
{
  std::string dataset_path = "/home/lxy/1/datasheet/MH_01_easy";
  
  if (argc > 1) {
    dataset_path = argv[1];
  }
  
  std::cout << "Testing SVO with EuRoC dataset: " << dataset_path << std::endl;
  
  {
    svo::EuRoCBenchmarkNode benchmark;
    benchmark.runFromEuRoCDataset(dataset_path);
  }
  
  printf("EuRoC Benchmark finished.\n");
  return 0;
}

