/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <iostream>
#include <algorithm>
#include <set>
#include <Eigen/Dense>

#include <sensor_msgs/image_encodings.hpp>

// NOTE: random_numbers/random_numbers.h is a ROS1 package.
// Use C++ <random> instead (see twoPointRansac()).
#include <random>

#include "asr_sdm_perception_msgs/msg/camera_measurement.hpp"
#include "asr_sdm_perception_msgs/msg/feature_measurement.hpp"
#include "asr_sdm_perception_msgs/msg/tracking_info.hpp"

#include <svo_ros/msckf_frontend/image_processor.h>

#include <svo/feature_alignment.h>

// ================= SVO 相关改动说明 =================
// 本文件在“时序跟踪(上一帧->当前帧)的 cam0 tracking”环节，
// 将原 MSCKF_VIO 常见的 OpenCV LK 光流(calcOpticalFlowPyrLK)替换为：
//   svo::feature_alignment::align2D
// 该方法基于小 patch 的直接法对齐(光度一致性)，可提供亚像素精度。
// 注意：本注释仅聚焦“单目核心流程 + SVO 改动”，双目(stereoMatch/cam1)相关流程不展开。
// ====================================================

#include <svo_ros/msckf_frontend/utils.h>

using namespace std;
using namespace cv;
using namespace Eigen;

namespace msckf_vio {

// Constructor - takes a ROS2 node pointer
ImageProcessor::ImageProcessor(rclcpp::Node* n) :
  nh_(n),
  is_first_img(true),
  prev_features_ptr(std::make_shared<GridFeatures>()),
  curr_features_ptr(std::make_shared<GridFeatures>()) {
  return;
}

ImageProcessor::~ImageProcessor() {
  cv::destroyAllWindows();
  return;
}

bool ImageProcessor::loadParameters() {
  // === 参数与标定读取 ===
  // 读取相机 cam0/cam1 的内参/畸变参数，以及 IMU<->cam0、cam0<->cam1 的外参。
  // 同时读取前端处理参数（网格划分、FAST 阈值、patch 大小、RANSAC 阈值等）。
  //
  // 关键外参说明：
  //   - 参数 cam0/T_cam_imu 在本实现里先读成 T_imu_cam0，然后取逆得到 (R_cam0_imu, t_cam0_imu)
  //   - 参数 cam1/T_cn_cnm1 视为 T_cam0_cam1，与 T_imu_cam0 相乘得到 T_imu_cam1，再取逆得到 (R_cam1_imu, t_cam1_imu)
  //
  // 输出到成员变量：
  //   cam{0,1}_intrinsics = [fx, fy, cx, cy]
  //   cam{0,1}_distortion_coeffs：radtan 通常为 [k1, k2, p1, p2]；fisheye/equidistant 则为对应系数
  //   R_cam{0,1}_imu / t_cam{0,1}_imu

  // Camera calibration parameters
  nh_->declare_parameter<std::string>("cam0/distortion_model", "radtan");
  nh_->declare_parameter<std::string>("cam1/distortion_model", "radtan");
  nh_->get_parameter("cam0/distortion_model", cam0_distortion_model);
  nh_->get_parameter("cam1/distortion_model", cam1_distortion_model);

  std::vector<double> cam0_resolution_temp;
  nh_->declare_parameter<std::vector<double>>("cam0/resolution", std::vector<double>({752.0, 480.0}));
  nh_->get_parameter("cam0/resolution", cam0_resolution_temp);
  cam0_resolution[0] = static_cast<int>(cam0_resolution_temp[0]);
  cam0_resolution[1] = static_cast<int>(cam0_resolution_temp[1]);

  std::vector<double> cam1_resolution_temp;
  nh_->declare_parameter<std::vector<double>>("cam1/resolution", std::vector<double>({752.0, 480.0}));
  nh_->get_parameter("cam1/resolution", cam1_resolution_temp);
  cam1_resolution[0] = static_cast<int>(cam1_resolution_temp[0]);
  cam1_resolution[1] = static_cast<int>(cam1_resolution_temp[1]);

  std::vector<double> cam0_intrinsics_temp;
  nh_->declare_parameter<std::vector<double>>("cam0/intrinsics", std::vector<double>({0.0, 0.0, 0.0, 0.0}));
  nh_->get_parameter("cam0/intrinsics", cam0_intrinsics_temp);
  cam0_intrinsics[0] = cam0_intrinsics_temp[0];
  cam0_intrinsics[1] = cam0_intrinsics_temp[1];
  cam0_intrinsics[2] = cam0_intrinsics_temp[2];
  cam0_intrinsics[3] = cam0_intrinsics_temp[3];

  std::vector<double> cam1_intrinsics_temp;
  nh_->declare_parameter<std::vector<double>>("cam1/intrinsics", std::vector<double>({0.0, 0.0, 0.0, 0.0}));
  nh_->get_parameter("cam1/intrinsics", cam1_intrinsics_temp);
  cam1_intrinsics[0] = cam1_intrinsics_temp[0];
  cam1_intrinsics[1] = cam1_intrinsics_temp[1];
  cam1_intrinsics[2] = cam1_intrinsics_temp[2];
  cam1_intrinsics[3] = cam1_intrinsics_temp[3];

  std::vector<double> cam0_distortion_coeffs_temp;
  nh_->declare_parameter<std::vector<double>>("cam0/distortion_coeffs", std::vector<double>({0.0, 0.0, 0.0, 0.0}));
  nh_->get_parameter("cam0/distortion_coeffs", cam0_distortion_coeffs_temp);
  cam0_distortion_coeffs[0] = cam0_distortion_coeffs_temp[0];
  cam0_distortion_coeffs[1] = cam0_distortion_coeffs_temp[1];
  cam0_distortion_coeffs[2] = cam0_distortion_coeffs_temp[2];
  cam0_distortion_coeffs[3] = cam0_distortion_coeffs_temp[3];

  std::vector<double> cam1_distortion_coeffs_temp;
  nh_->declare_parameter<std::vector<double>>("cam1/distortion_coeffs", std::vector<double>({0.0, 0.0, 0.0, 0.0}));
  nh_->get_parameter("cam1/distortion_coeffs", cam1_distortion_coeffs_temp);
  cam1_distortion_coeffs[0] = cam1_distortion_coeffs_temp[0];
  cam1_distortion_coeffs[1] = cam1_distortion_coeffs_temp[1];
  cam1_distortion_coeffs[2] = cam1_distortion_coeffs_temp[2];
  cam1_distortion_coeffs[3] = cam1_distortion_coeffs_temp[3];

  cv::Mat     T_imu_cam0 = utils::getTransformCV(*nh_, "cam0/T_cam_imu");
  cv::Matx33d R_imu_cam0(T_imu_cam0(cv::Rect(0,0,3,3)));
  cv::Vec3d   t_imu_cam0 = T_imu_cam0(cv::Rect(3,0,1,3));
  R_cam0_imu = R_imu_cam0.t();
  t_cam0_imu = -R_imu_cam0.t() * t_imu_cam0;

  cv::Mat T_cam0_cam1 = utils::getTransformCV(*nh_, "cam1/T_cn_cnm1");
  cv::Mat T_imu_cam1 = T_cam0_cam1 * T_imu_cam0;
  cv::Matx33d R_imu_cam1(T_imu_cam1(cv::Rect(0,0,3,3)));
  cv::Vec3d   t_imu_cam1 = T_imu_cam1(cv::Rect(3,0,1,3));
  R_cam1_imu = R_imu_cam1.t();
  t_cam1_imu = -R_imu_cam1.t() * t_imu_cam1;

  // Processor parameters
  nh_->declare_parameter<int>("grid_row", 4);
  nh_->declare_parameter<int>("grid_col", 4);
  nh_->declare_parameter<int>("grid_min_feature_num", 2);
  nh_->declare_parameter<int>("grid_max_feature_num", 4);
  nh_->declare_parameter<int>("pyramid_levels", 3);
  nh_->declare_parameter<int>("patch_size", 31);
  nh_->declare_parameter<int>("fast_threshold", 20);
  nh_->declare_parameter<int>("max_iteration", 30);
  nh_->declare_parameter<double>("track_precision", 0.01);
  nh_->declare_parameter<double>("ransac_threshold", 3.0);
  nh_->declare_parameter<double>("stereo_threshold", 3.0);

  nh_->get_parameter("grid_row", processor_config.grid_row);
  nh_->get_parameter("grid_col", processor_config.grid_col);
  nh_->get_parameter("grid_min_feature_num", processor_config.grid_min_feature_num);
  nh_->get_parameter("grid_max_feature_num", processor_config.grid_max_feature_num);
  nh_->get_parameter("pyramid_levels", processor_config.pyramid_levels);
  nh_->get_parameter("patch_size", processor_config.patch_size);
  nh_->get_parameter("fast_threshold", processor_config.fast_threshold);
  nh_->get_parameter("max_iteration", processor_config.max_iteration);
  nh_->get_parameter("track_precision", processor_config.track_precision);
  nh_->get_parameter("ransac_threshold", processor_config.ransac_threshold);
  nh_->get_parameter("stereo_threshold", processor_config.stereo_threshold);

  RCLCPP_INFO(nh_->get_logger(), "===========================================");
  RCLCPP_INFO(nh_->get_logger(), "cam0_resolution: %d, %d",
      cam0_resolution[0], cam0_resolution[1]);
  RCLCPP_INFO(nh_->get_logger(), "cam0_intrinscs: %f, %f, %f, %f",
      cam0_intrinsics[0], cam0_intrinsics[1],
      cam0_intrinsics[2], cam0_intrinsics[3]);
  RCLCPP_INFO(nh_->get_logger(), "cam0_distortion_model: %s",
      cam0_distortion_model.c_str());
  RCLCPP_INFO(nh_->get_logger(), "cam0_distortion_coefficients: %f, %f, %f, %f",
      cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
      cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

  RCLCPP_INFO(nh_->get_logger(), "cam1_resolution: %d, %d",
      cam1_resolution[0], cam1_resolution[1]);
  RCLCPP_INFO(nh_->get_logger(), "cam1_intrinscs: %f, %f, %f, %f",
      cam1_intrinsics[0], cam1_intrinsics[1],
      cam1_intrinsics[2], cam1_intrinsics[3]);
  RCLCPP_INFO(nh_->get_logger(), "cam1_distortion_model: %s",
      cam1_distortion_model.c_str());
  RCLCPP_INFO(nh_->get_logger(), "cam1_distortion_coefficients: %f, %f, %f, %f",
      cam1_distortion_coeffs[0], cam1_distortion_coeffs[1],
      cam1_distortion_coeffs[2], cam1_distortion_coeffs[3]);

  std::cout << R_imu_cam0 << std::endl;
  std::cout << t_imu_cam0.t() << std::endl;

  RCLCPP_INFO(nh_->get_logger(), "grid_row: %d",
      processor_config.grid_row);
  RCLCPP_INFO(nh_->get_logger(), "grid_col: %d",
      processor_config.grid_col);
  RCLCPP_INFO(nh_->get_logger(), "grid_min_feature_num: %d",
      processor_config.grid_min_feature_num);
  RCLCPP_INFO(nh_->get_logger(), "grid_max_feature_num: %d",
      processor_config.grid_max_feature_num);
  RCLCPP_INFO(nh_->get_logger(), "pyramid_levels: %d",
      processor_config.pyramid_levels);
  RCLCPP_INFO(nh_->get_logger(), "patch_size: %d",
      processor_config.patch_size);
  RCLCPP_INFO(nh_->get_logger(), "fast_threshold: %d",
      processor_config.fast_threshold);
  RCLCPP_INFO(nh_->get_logger(), "max_iteration: %d",
      processor_config.max_iteration);
  RCLCPP_INFO(nh_->get_logger(), "track_precision: %f",
      processor_config.track_precision);
  RCLCPP_INFO(nh_->get_logger(), "ransac_threshold: %f",
      processor_config.ransac_threshold);
  RCLCPP_INFO(nh_->get_logger(), "stereo_threshold: %f",
      processor_config.stereo_threshold);
  RCLCPP_INFO(nh_->get_logger(), "===========================================");
  return true;
}

bool ImageProcessor::createRosIO() {
  // Create publishers
  feature_pub_ = nh_->create_publisher<asr_sdm_perception_msgs::msg::CameraMeasurement>("features", 3);
  tracking_info_pub_ = nh_->create_publisher<asr_sdm_perception_msgs::msg::TrackingInfo>("tracking_info", 1);
  
  // Point cloud publisher for feature visualization in RViz2
  feature_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("feature_points", 1);
  feature_cloud_tracked_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("feature_points_tracked", 1);
  feature_cloud_new_pub_     = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("feature_points_new", 1);
  
  // Debug images (for RViz2 Image display)
  // ImageTransport 需要 Node::SharedPtr，这里用一个不负责销毁的别名 shared_ptr 包装 nh_
  auto node_shared = rclcpp::Node::SharedPtr(nh_, [](rclcpp::Node*){});
  image_transport::ImageTransport it(node_shared);
  debug_mono_pub_   = it.advertise("debug/mono_features",   1);
  debug_stereo_pub_ = it.advertise("debug/stereo_features", 1);

  // Create subscribers with message filters for time synchronization
    // Topics (defaults match EuRoC ROS2 bag: /cam0/image_raw, /cam1/image_raw, /imu0)
  nh_->declare_parameter<std::string>("cam0_topic", "/cam0/image_raw");
  nh_->declare_parameter<std::string>("cam1_topic", "/cam1/image_raw");
  nh_->declare_parameter<std::string>("imu_topic", "/imu0");
  nh_->declare_parameter<bool>("use_stereo", true);
  nh_->get_parameter("use_stereo", use_stereo_);

  std::string cam0_topic, cam1_topic, imu_topic;
  nh_->get_parameter("cam0_topic", cam0_topic);
  nh_->get_parameter("cam1_topic", cam1_topic);
  nh_->get_parameter("imu_topic", imu_topic);

  const auto qos = rclcpp::SensorDataQoS();
  if (use_stereo_) {
    cam0_img_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      nh_, cam0_topic, qos);
    cam1_img_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      nh_, cam1_topic, qos);

    // Time synchronizer
    stereo_sub_ = std::make_unique<message_filters::Synchronizer<StereoSyncPolicy>>(
      StereoSyncPolicy(10), *cam0_img_sub_, *cam1_img_sub_);
    stereo_sub_->registerCallback(&ImageProcessor::stereoCallback, this);
  } else {
    // Mono mode: subscribe only cam0 and reuse stereoCallback with nullptr cam1.
    cam0_img_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      nh_, cam0_topic, qos);
    cam0_img_sub_->registerCallback(
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& cam0_img) {
        this->stereoCallback(cam0_img, sensor_msgs::msg::Image::ConstSharedPtr());
      });
    cam1_img_sub_.reset();
    stereo_sub_.reset();
  }

  // ================= IMU 订阅（不发布 IMU） =================
  // 本节点只“订阅并缓存” IMU，用于给图像跟踪提供旋转初值：
  //   - imuCallback(): 将 IMU 追加到 imu_msg_buffer
  //   - integrateImuData(): 从 buffer 中取出上一帧到当前帧之间的 IMU 段，
  //     对角速度求均值并积分，得到两帧间相对旋转 R_p_c
  // 注意：这里不会发布任何 IMU topic；IMU 仅作为内部计算输入。
  // ==========================================================
  imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&ImageProcessor::imuCallback, this, std::placeholders::_1));

  return true;
}

bool ImageProcessor::initialize() {
  if (!loadParameters()) return false;
  RCLCPP_INFO(nh_->get_logger(), "Finish loading ROS parameters...");

  // Create feature detector.
  detector_ptr = FastFeatureDetector::create(
      processor_config.fast_threshold);

  if (!createRosIO()) return false;
  RCLCPP_INFO(nh_->get_logger(), "Finish creating ROS IO...");

  return true;
}

void ImageProcessor::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& cam0_img,
    const sensor_msgs::msg::Image::ConstSharedPtr& cam1_img) {

  // === 每帧主处理流程（只注释核心/单目流程，不展开双目细节） ===
  // 输入：
  //   - cam0 图像（必需）
  //   - IMU 缓存在 imu_msg_buffer（用于估计两帧间相对旋转，给跟踪提供初值）
  // 输出：
  //   - 发布 "features"：asr_sdm_perception_msgs::msg::CameraMeasurement（特征 id + 去畸变后的像素坐标 u0,v0；单目时 u1,v1 为 0）
  //   - 发布 "tracking_info"：跟踪统计信息（before_tracking/after_tracking/after_ransac 等）
  //
  // 核心处理环节：
  //   A) 建图像金字塔：用于后续跟踪/匹配（本文件 cam0 的时序跟踪实际走 SVO patch，对金字塔依赖较小，但仍保留）
  //   B) 首帧初始化：FAST 检测 -> 按网格挑选 -> 分配 id/lifetime
  //   C) 非首帧跟踪：
  //        1) IMU 积分得到相对旋转 R_p_c
  //        2) 用 H=K*R*K^{-1} 预测像素初值
  //        3) 用 SVO align2D 做 patch 对齐，得到亚像素跟踪结果
  //        4) two-point RANSAC（旋转已知模型）去外点
  //   D) 补点与裁剪：在每个网格内不足时补新点；超出上限时按 lifetime/response 等规则裁剪
  //   E) 发布：输出去畸变后的特征坐标给后端（MSCKF）使用


  //cout << "==================================" << endl;

  // Get the current image.
  cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_img,
      sensor_msgs::image_encodings::MONO8);

  if (use_stereo_ && cam1_img) {
    cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_img,
        sensor_msgs::image_encodings::MONO8);
  } else {
    cam1_curr_img_ptr.reset();
  }

  // Build the image pyramids once since they're used at multiple places
  createImagePyramids();

  // Detect features in the first frame.
  if (is_first_img) {
    auto start_time = nh_->now();
    initializeFirstFrame();
    //ROS_INFO("Detection time: %f",
    //    (ros::Time::now()-start_time).toSec());
    is_first_img = false;

    // Draw results.
    start_time = nh_->now();
    if (use_stereo_) drawFeaturesStereo(); else drawFeaturesMono();
    //ROS_INFO("Draw features: %f",
    //    (ros::Time::now()-start_time).toSec());
  } else {
    // Track the feature in the previous image.
    auto start_time = nh_->now();
    trackFeatures();
    //ROS_INFO("Tracking time: %f",
    //    (ros::Time::now()-start_time).toSec());

    // Add new features into the current image.
    start_time = nh_->now();
    addNewFeatures();
    //ROS_INFO("Addition time: %f",
    //    (ros::Time::now()-start_time).toSec());

    // Add new features into the current image.
    start_time = nh_->now();
    pruneGridFeatures();
    //ROS_INFO("Prune grid features: %f",
    //    (ros::Time::now()-start_time).toSec());

    // Draw results.
    start_time = nh_->now();
    if (use_stereo_) drawFeaturesStereo(); else drawFeaturesMono();
    //ROS_INFO("Draw features: %f",
    //    (ros::Time::now()-start_time).toSec());
  }

  //ros::Time start_time = ros::Time::now();
  //updateFeatureLifetime();
  //ROS_INFO("Statistics: %f",
  //    (ros::Time::now()-start_time).toSec());

  // Publish features in the current image.
  auto start_time = nh_->now();
  publish();
  //ROS_INFO("Publishing: %f",
  //    (ros::Time::now()-start_time).toSec());

  // Update the previous image and previous features.
  cam0_prev_img_ptr = cam0_curr_img_ptr;
  prev_features_ptr = curr_features_ptr;
  std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

  // Initialize the current features to empty vectors.
  curr_features_ptr = std::make_shared<GridFeatures>();
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code) {
    (*curr_features_ptr)[code] = vector<FeatureMetaData>(0);
  }

  return;
}

void ImageProcessor::imuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  // IMU 回调：缓存 IMU（不做发布）。
  // 目的：给 integrateImuData() 提供两帧图像之间的 IMU 段，用于估计相对旋转。
  // 说明：
  //   - 首帧图像到来之前不缓存（因为此时还没有 prev_img_time，无法形成有效区间）
  //   - 缓存内容是完整的 sensor_msgs::msg::Imu（包含角速度/线加速度等），
  //     但本文件当前只使用角速度来做旋转预测。
  if (is_first_img) return;
  imu_msg_buffer.push_back(*msg);
  return;
}

void ImageProcessor::createImagePyramids() {
  // 构建 cam0 的金字塔（OpenCV optical flow pyramid）。
  // 说明：
  //   - 在原版 MSCKF_VIO 中，时序跟踪通常直接依赖该金字塔做 LK 光流。
  //   - 本实现的 cam0 时序跟踪已改为 SVO align2D（直接法 patch 对齐），
  //     对金字塔依赖降低，但仍保留：
  //       1) 兼容其它流程/调试
  //       2) 未来如需恢复 LK 或混合策略可直接使用

  const Mat& curr_cam0_img = cam0_curr_img_ptr->image;
  buildOpticalFlowPyramid(
      curr_cam0_img, curr_cam0_pyramid_,
      Size(processor_config.patch_size, processor_config.patch_size),
      processor_config.pyramid_levels, true, BORDER_REFLECT_101,
      BORDER_CONSTANT, false);

  curr_cam1_pyramid_.clear();
  if (use_stereo_ && cam1_curr_img_ptr) {
    const Mat& curr_cam1_img = cam1_curr_img_ptr->image;
    buildOpticalFlowPyramid(
        curr_cam1_img, curr_cam1_pyramid_,
        Size(processor_config.patch_size, processor_config.patch_size),
        processor_config.pyramid_levels, true, BORDER_REFLECT_101,
        BORDER_CONSTANT, false);
  }
}

void ImageProcessor::initializeFirstFrame() {
  // 首帧初始化（核心单目逻辑）：
  //   1) FAST 检测出候选特征点
  //   2) 按网格(grid)分桶，并按 response 由高到低排序
  //   3) 每个网格至少取 grid_min_feature_num 个，分配全局唯一 id，并设置 lifetime=1
  // 说明：
  //   - 网格化的目的：让特征分布更均匀，避免集中在纹理丰富区域
  //   - 本文件的后续跟踪依赖“id + lifetime”维护特征轨迹

  // Size of each grid.
  const Mat& img = cam0_curr_img_ptr->image;
  static int grid_height = img.rows / processor_config.grid_row;
  static int grid_width = img.cols / processor_config.grid_col;

  // Detect new features on the frist image.
  vector<KeyPoint> new_features(0);
  detector_ptr->detect(img, new_features);

  // Find the stereo matched points for the newly
  // detected features.
  vector<cv::Point2f> cam0_points(new_features.size());
  for (int i = 0; i < new_features.size(); ++i)
    cam0_points[i] = new_features[i].pt;

  vector<cv::Point2f> cam1_points(0);
  vector<unsigned char> inlier_markers(0);
  stereoMatch(cam0_points, cam1_points, inlier_markers);

  vector<cv::Point2f> cam0_inliers(0);
  vector<cv::Point2f> cam1_inliers(0);
  vector<float> response_inliers(0);
  for (int i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(new_features[i].response);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code)
      grid_new_features[code] = vector<FeatureMetaData>(0);

  for (int i = 0; i < cam0_inliers.size(); ++i) {
    const cv::Point2f& cam0_point = cam0_inliers[i];
    const cv::Point2f& cam1_point = cam1_inliers[i];
    const float& response = response_inliers[i];

    int row = static_cast<int>(cam0_point.y / grid_height);
    int col = static_cast<int>(cam0_point.x / grid_width);
    int code = row*processor_config.grid_col + col;

    FeatureMetaData new_feature;
    new_feature.response = response;
    new_feature.cam0_point = cam0_point;
    new_feature.cam1_point = cam1_point;
    grid_new_features[code].push_back(new_feature);
  }

  // Sort the new features in each grid based on its response.
  for (auto& item : grid_new_features)
    std::sort(item.second.begin(), item.second.end(),
        &ImageProcessor::featureCompareByResponse);

  // Collect new features within each grid with high response.
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code) {
    vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
    vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

    for (int k = 0; k < processor_config.grid_min_feature_num &&
        k < new_features_this_grid.size(); ++k) {
      features_this_grid.push_back(new_features_this_grid[k]);
      features_this_grid.back().id = next_feature_id++;
      features_this_grid.back().lifetime = 1;
    }
  }

  return;
}

void ImageProcessor::predictFeatureTracking(
    const vector<cv::Point2f>& input_pts,
    const cv::Matx33f& R_p_c,
    const cv::Vec4d& intrinsics,
    vector<cv::Point2f>& compensated_pts) {

  // 用旋转预测像素初值：
  // 已知两帧间相对旋转 R_p_c（来自 IMU），在小视角/远景近似下可用
  //   H = K * R_p_c * K^{-1}
  // 把上一帧像素点投影到当前帧，得到 compensated_pts。
  // 该初值会喂给后续的 SVO patch 对齐（align2D），显著影响对齐成功率。


  // Return directly if there are no input features.
  if (input_pts.size() == 0) {
    compensated_pts.clear();
    return;
  }
  compensated_pts.resize(input_pts.size());

  // Intrinsic matrix.
  cv::Matx33f K(
      intrinsics[0], 0.0, intrinsics[2],
      0.0, intrinsics[1], intrinsics[3],
      0.0, 0.0, 1.0);
  cv::Matx33f H = K * R_p_c * K.inv();

  for (int i = 0; i < input_pts.size(); ++i) {
    cv::Vec3f p1(input_pts[i].x, input_pts[i].y, 1.0f);
    cv::Vec3f p2 = H * p1;
    compensated_pts[i].x = p2[0] / p2[2];
    compensated_pts[i].y = p2[1] / p2[2];
  }

  return;
}

void ImageProcessor::trackFeatures() {
  // === 特征跟踪流程（上一帧 -> 当前帧；只注释单目核心与 SVO 改动） ===
  // 1) IMU 旋转预测：
  //    integrateImuData() 会在 imu_msg_buffer 中截取 prev_img_stamp ~ curr_img_stamp 之间的 IMU，
  //    对角速度求均值并积分，得到两帧间的相对旋转 R_p_c（旋转-only）。
  // 2) 像素初值预测：
  //    predictFeatureTracking() 使用 H = K * R_p_c * K^{-1}，把上一帧像素点投影到当前帧，作为跟踪初值。
  // 3) 【SVO 改动点】cam0 时序跟踪：
  //    原本常见实现是 calcOpticalFlowPyrLK(KLT)；这里改为 SVO 的直接法 patch 对齐：
  //      svo::feature_alignment::align2D(curr_img, ref_patch_border, ref_patch, max_iter, cur_px_est)
  //    要点：
  //      - ref_patch 从上一帧以 pt_prev 为中心裁剪，尺寸为 patch_size
  //      - 额外加 1 像素 border（align2D 需要）
  //      - cur_px_est 初值来自步骤 2（IMU 预测）
  //      - 成功后可得到亚像素级的 curr 点
  //    边界处理：若上一帧 patch 越界、或对齐结果越界，则该点判为跟踪失败。
  // 4) two-point RANSAC 去外点：
  //    twoPointRansac() 假设旋转已知，通过两点即可拟合平移方向的约束，对 prev<->curr 匹配进行鲁棒筛选。
  // 5) 将通过 RANSAC 的内点重新按网格放入 curr_features，并把 lifetime +1。

  // Size of each grid.
  static int grid_height =
    cam0_curr_img_ptr->image.rows / processor_config.grid_row;
  static int grid_width =
    cam0_curr_img_ptr->image.cols / processor_config.grid_col;

  // Compute a rough relative rotation which takes a vector
  // from the previous frame to the current frame.
  Matx33f cam0_R_p_c;
  Matx33f cam1_R_p_c;
  integrateImuData(cam0_R_p_c, cam1_R_p_c);

  // Organize the features in the previous image.
  vector<FeatureIDType> prev_ids(0);
  vector<int> prev_lifetime(0);
  vector<Point2f> prev_cam0_points(0);
  vector<Point2f> prev_cam1_points(0);

  for (const auto& item : *prev_features_ptr) {
    for (const auto& prev_feature : item.second) {
      prev_ids.push_back(prev_feature.id);
      prev_lifetime.push_back(prev_feature.lifetime);
      prev_cam0_points.push_back(prev_feature.cam0_point);
      prev_cam1_points.push_back(prev_feature.cam1_point);
    }
  }

  // Number of the features before tracking.
  before_tracking = prev_cam0_points.size();

  // Abort tracking if there is no features in
  // the previous frame.
  if (prev_ids.size() == 0) return;

  // Track features using LK optical flow method.
  vector<Point2f> curr_cam0_points(0);
  vector<unsigned char> track_inliers(0);

  predictFeatureTracking(prev_cam0_points,
      cam0_R_p_c, cam0_intrinsics, curr_cam0_points);

  // -------------------- 【SVO 改动：用直接法 patch 对齐替换 KLT 光流】 --------------------
  // 这里不再调用 OpenCV calcOpticalFlowPyrLK 做上一帧->当前帧的 LK 跟踪，
  // 而是对每个特征点：
  //   1) 在上一帧提取一个以特征为中心的 ref_patch（patch_size x patch_size）
  //   2) 额外加 1 像素边界(ref_patch_border)，满足 align2D 的输入需求
  //   3) 用 IMU+单应预测得到的 curr_cam0_points[idx] 作为初值 cur_px_est
  //   4) 调用 svo::feature_alignment::align2D 在当前帧图像上做光度对齐
  //      成功则输出亚像素坐标 cur_px_est，并标记 track_inliers[idx]=1
  //
  // 注意：这里的对齐是“单点局部直接法”，没有显式金字塔 coarse-to-fine（依赖 align2D 内部实现）。
  // 因此对初值较敏感：IMU 预测越准，成功率越高。
  // ----------------------------------------------------------------------------------
  track_inliers.assign(prev_cam0_points.size(), 0);
  const int half_patch = processor_config.patch_size / 2;
  for(size_t idx = 0; idx < prev_cam0_points.size(); ++idx)
  {
    const cv::Point2f& pt_prev = prev_cam0_points[idx];
    // ensure patch inside image in prev
    if(pt_prev.x < half_patch+1 || pt_prev.y < half_patch+1 ||
       pt_prev.x >= cam0_prev_img_ptr->image.cols - half_patch -1 ||
       pt_prev.y >= cam0_prev_img_ptr->image.rows - half_patch -1)
      continue;

    // Extract reference patch with 1-pixel border (needed by align2D)
    cv::Rect box(static_cast<int>(pt_prev.x) - half_patch,
                 static_cast<int>(pt_prev.y) - half_patch,
                 processor_config.patch_size,
                 processor_config.patch_size);
    cv::Mat ref_patch = cam0_prev_img_ptr->image(box).clone();
    cv::Mat ref_patch_border;
    cv::copyMakeBorder(ref_patch, ref_patch_border, 1,1,1,1, cv::BORDER_REPLICATE);

    // initial guess from IMU prediction already in curr_cam0_points[idx]
    Eigen::Vector2d cur_px_est(curr_cam0_points[idx].x, curr_cam0_points[idx].y);

    bool ok = svo::feature_alignment::align2D(
      cam0_curr_img_ptr->image,
      ref_patch_border.data,
      ref_patch.data,
      processor_config.max_iteration,
      cur_px_est);

    if(!ok) continue;

    // check within image bounds
    if(cur_px_est.x() < 0 || cur_px_est.y() < 0 ||
       cur_px_est.x() >= cam0_curr_img_ptr->image.cols-1 ||
       cur_px_est.y() >= cam0_curr_img_ptr->image.rows-1)
      continue;

    curr_cam0_points[idx] = cv::Point2f(cur_px_est.x(), cur_px_est.y());
    track_inliers[idx] = 1;
  }

  // Mark those tracked points out of the image region
  // as untracked.
  for (int i = 0; i < curr_cam0_points.size(); ++i) {
    if (track_inliers[i] == 0) continue;
    if (curr_cam0_points[i].y < 0 ||
        curr_cam0_points[i].y > cam0_curr_img_ptr->image.rows-1 ||
        curr_cam0_points[i].x < 0 ||
        curr_cam0_points[i].x > cam0_curr_img_ptr->image.cols-1)
      track_inliers[i] = 0;
  }

  // Collect the tracked points.
  vector<FeatureIDType> prev_tracked_ids(0);
  vector<int> prev_tracked_lifetime(0);
  vector<Point2f> prev_tracked_cam0_points(0);
  vector<Point2f> prev_tracked_cam1_points(0);
  vector<Point2f> curr_tracked_cam0_points(0);

  removeUnmarkedElements(
      prev_ids, track_inliers, prev_tracked_ids);
  removeUnmarkedElements(
      prev_lifetime, track_inliers, prev_tracked_lifetime);
  removeUnmarkedElements(
      prev_cam0_points, track_inliers, prev_tracked_cam0_points);
  removeUnmarkedElements(
      prev_cam1_points, track_inliers, prev_tracked_cam1_points);
  removeUnmarkedElements(
      curr_cam0_points, track_inliers, curr_tracked_cam0_points);

  // Number of features left after tracking.
  after_tracking = curr_tracked_cam0_points.size();


  // Outlier removal involves three steps, which forms a close
  // loop between the previous and current frames of cam0 (left)
  // and cam1 (right). Assuming the stereo matching between the
  // previous cam0 and cam1 images are correct, the three steps are:
  //
  // prev frames cam0 ----------> cam1
  //              |                |
  //              |ransac          |ransac
  //              |   stereo match |
  // curr frames cam0 ----------> cam1
  //
  // 1) Stereo matching between current images of cam0 and cam1.
  // 2) RANSAC between previous and current images of cam0.
  // 3) RANSAC between previous and current images of cam1.
  //
  // For Step 3, tracking between the images is no longer needed.
  // The stereo matching results are directly used in the RANSAC.

  // Step 1: stereo matching (or mono passthrough)
  vector<FeatureIDType> prev_matched_ids(0);
  vector<int> prev_matched_lifetime(0);
  vector<Point2f> prev_matched_cam0_points(0);
  vector<Point2f> prev_matched_cam1_points(0);
  vector<Point2f> curr_matched_cam0_points(0);
  vector<Point2f> curr_matched_cam1_points(0);

  if (use_stereo_) {
    vector<Point2f> curr_cam1_points(0);
    vector<unsigned char> match_inliers(0);
    stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

    removeUnmarkedElements(prev_tracked_ids, match_inliers, prev_matched_ids);
    removeUnmarkedElements(prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
    removeUnmarkedElements(prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
    removeUnmarkedElements(prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
    removeUnmarkedElements(curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
    removeUnmarkedElements(curr_cam1_points, match_inliers, curr_matched_cam1_points);

    after_matching = static_cast<int>(curr_matched_cam0_points.size());
  } else {
    // Mono: keep all tracked cam0 points, dummy cam1 points.
    prev_matched_ids = prev_tracked_ids;
    prev_matched_lifetime = prev_tracked_lifetime;
    prev_matched_cam0_points = prev_tracked_cam0_points;
    curr_matched_cam0_points = curr_tracked_cam0_points;

    prev_matched_cam1_points.assign(prev_matched_cam0_points.size(), cv::Point2f(0.f, 0.f));
    curr_matched_cam1_points.assign(curr_matched_cam0_points.size(), cv::Point2f(0.f, 0.f));

    after_matching = after_tracking;
  }

  // Step 2: RANSAC on temporal image pair of cam0.
  vector<int> cam0_ransac_inliers(0);
  twoPointRansac(prev_matched_cam0_points, curr_matched_cam0_points,
      cam0_R_p_c, cam0_intrinsics, cam0_distortion_model,
      cam0_distortion_coeffs, processor_config.ransac_threshold,
      0.99, cam0_ransac_inliers);

  vector<int> cam1_ransac_inliers;
  if (use_stereo_) {
    cam1_ransac_inliers.resize(curr_matched_cam0_points.size(), 0);
    twoPointRansac(prev_matched_cam1_points, curr_matched_cam1_points,
        cam1_R_p_c, cam1_intrinsics, cam1_distortion_model,
        cam1_distortion_coeffs, processor_config.ransac_threshold,
        0.99, cam1_ransac_inliers);
  } else {
    cam1_ransac_inliers.assign(curr_matched_cam0_points.size(), 1);
  }

  // Number of features after ransac.
  after_ransac = 0;

  for (int i = 0; i < static_cast<int>(cam0_ransac_inliers.size()); ++i) {
    if (cam0_ransac_inliers[i] == 0 || cam1_ransac_inliers[i] == 0) continue;

    int row = static_cast<int>(curr_matched_cam0_points[i].y / grid_height);
    int col = static_cast<int>(curr_matched_cam0_points[i].x / grid_width);
    int code = row * processor_config.grid_col + col;
    (*curr_features_ptr)[code].push_back(FeatureMetaData());

    FeatureMetaData& grid_new_feature = (*curr_features_ptr)[code].back();
    grid_new_feature.id = prev_matched_ids[i];
    grid_new_feature.lifetime = ++prev_matched_lifetime[i];
    grid_new_feature.cam0_point = curr_matched_cam0_points[i];
    grid_new_feature.cam1_point = use_stereo_ ? curr_matched_cam1_points[i] : cv::Point2f(0.f, 0.f);

    ++after_ransac;
  }

  // Compute the tracking rate.
  int prev_feature_num = 0;
  for (const auto& item : *prev_features_ptr)
    prev_feature_num += item.second.size();

  int curr_feature_num = 0;
  for (const auto& item : *curr_features_ptr)
    curr_feature_num += item.second.size();

  RCLCPP_INFO_THROTTLE(
      nh_->get_logger(), *nh_->get_clock(), 500,
      "candidates: %d; track: %d; match: %d; ransac: %d/%d=%f",
      before_tracking, after_tracking, after_matching,
      curr_feature_num, prev_feature_num,
      static_cast<double>(curr_feature_num) /
        (static_cast<double>(prev_feature_num) + 1e-5));
  //printf(
  //    "\033[0;32m candidates: %d; raw track: %d; stereo match: %d; ransac: %d/%d=%f\033[0m\n",
  //    before_tracking, after_tracking, after_matching,
  //    curr_feature_num, prev_feature_num,
  //    static_cast<double>(curr_feature_num)/
  //    (static_cast<double>(prev_feature_num)+1e-5));

  return;
}

void ImageProcessor::stereoMatch(
    const vector<cv::Point2f>& cam0_points,
    vector<cv::Point2f>& cam1_points,
    vector<unsigned char>& inlier_markers) {

  if (!use_stereo_) {
    // Mono mode: no stereo matching. Mark all as inliers and provide dummy cam1 points.
    cam1_points.assign(cam0_points.size(), cv::Point2f(0.f, 0.f));
    inlier_markers.assign(cam0_points.size(), 1);
    return;
  }

  if (cam0_points.empty()) return;

  if(cam1_points.size() == 0) {
    // Initialize cam1_points by projecting cam0_points to cam1 using the
    // rotation from stereo extrinsics
    const cv::Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
    vector<cv::Point2f> cam0_points_undistorted;
    undistortPoints(cam0_points, cam0_intrinsics, cam0_distortion_model,
                    cam0_distortion_coeffs, cam0_points_undistorted,
                    R_cam0_cam1);
    cam1_points = distortPoints(cam0_points_undistorted, cam1_intrinsics,
                                cam1_distortion_model, cam1_distortion_coeffs);
  }

  // Track features using LK optical flow method.
  calcOpticalFlowPyrLK(curr_cam0_pyramid_, curr_cam1_pyramid_,
      cam0_points, cam1_points,
      inlier_markers, noArray(),
      Size(processor_config.patch_size, processor_config.patch_size),
      processor_config.pyramid_levels,
      TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                   processor_config.max_iteration,
                   processor_config.track_precision),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  // Mark those tracked points out of the image region
  // as untracked.
  for (int i = 0; i < cam1_points.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    if (cam1_points[i].y < 0 ||
        cam1_points[i].y > cam1_curr_img_ptr->image.rows-1 ||
        cam1_points[i].x < 0 ||
        cam1_points[i].x > cam1_curr_img_ptr->image.cols-1)
      inlier_markers[i] = 0;
  }

  // Compute the relative rotation between the cam0
  // frame and cam1 frame.
  const cv::Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
  const cv::Vec3d t_cam0_cam1 = R_cam1_imu.t() * (t_cam0_imu-t_cam1_imu);
  // Compute the essential matrix.
  const cv::Matx33d t_cam0_cam1_hat(
      0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
      t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
      -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
  const cv::Matx33d E = t_cam0_cam1_hat * R_cam0_cam1;

  // Further remove outliers based on the known
  // essential matrix.
  vector<cv::Point2f> cam0_points_undistorted(0);
  vector<cv::Point2f> cam1_points_undistorted(0);
  undistortPoints(
      cam0_points, cam0_intrinsics, cam0_distortion_model,
      cam0_distortion_coeffs, cam0_points_undistorted);
  undistortPoints(
      cam1_points, cam1_intrinsics, cam1_distortion_model,
      cam1_distortion_coeffs, cam1_points_undistorted);

  double norm_pixel_unit = 4.0 / (
      cam0_intrinsics[0]+cam0_intrinsics[1]+
      cam1_intrinsics[0]+cam1_intrinsics[1]);

  for (int i = 0; i < cam0_points_undistorted.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    cv::Vec3d pt0(cam0_points_undistorted[i].x,
        cam0_points_undistorted[i].y, 1.0);
    cv::Vec3d pt1(cam1_points_undistorted[i].x,
        cam1_points_undistorted[i].y, 1.0);
    cv::Vec3d epipolar_line = E * pt0;
    double error = fabs((pt1.t() * epipolar_line)[0]) / sqrt(
        epipolar_line[0]*epipolar_line[0]+
        epipolar_line[1]*epipolar_line[1]);
    if (error > processor_config.stereo_threshold*norm_pixel_unit)
      inlier_markers[i] = 0;
  }

  return;
}

void ImageProcessor::addNewFeatures() {
  // 补充新特征（保持特征数量与空间分布）：
  //   - 先根据当前已跟踪到的点生成 mask，避免在已有特征附近重复检测
  //   - FAST 在 mask 上检测候选点
  //   - 按网格分桶，每格最多保留 grid_max_feature_num 个（按 response 取强特征）
  //   - 仅对“当前网格特征数 < grid_min_feature_num”的网格补点
  //   - 对补入的新点分配新 id，lifetime=1
  // 注：这里是“检测/补点”环节，与 SVO 改动无关；SVO 改动仅发生在时序跟踪 trackFeatures() 内。

  const Mat& curr_img = cam0_curr_img_ptr->image;

  // Size of each grid.
  static int grid_height =
    cam0_curr_img_ptr->image.rows / processor_config.grid_row;
  static int grid_width =
    cam0_curr_img_ptr->image.cols / processor_config.grid_col;

  // Create a mask to avoid redetecting existing features.
  Mat mask(curr_img.rows, curr_img.cols, CV_8U, Scalar(1));

  for (const auto& features : *curr_features_ptr) {
    for (const auto& feature : features.second) {
      const int y = static_cast<int>(feature.cam0_point.y);
      const int x = static_cast<int>(feature.cam0_point.x);

      int up_lim = y-2, bottom_lim = y+3,
          left_lim = x-2, right_lim = x+3;
      if (up_lim < 0) up_lim = 0;
      if (bottom_lim > curr_img.rows) bottom_lim = curr_img.rows;
      if (left_lim < 0) left_lim = 0;
      if (right_lim > curr_img.cols) right_lim = curr_img.cols;

      Range row_range(up_lim, bottom_lim);
      Range col_range(left_lim, right_lim);
      mask(row_range, col_range) = 0;
    }
  }

  // Detect new features.
  vector<KeyPoint> new_features(0);
  detector_ptr->detect(curr_img, new_features, mask);

  // Collect the new detected features based on the grid.
  // Select the ones with top response within each grid afterwards.
  vector<vector<KeyPoint> > new_feature_sieve(
      processor_config.grid_row*processor_config.grid_col);
  for (const auto& feature : new_features) {
    int row = static_cast<int>(feature.pt.y / grid_height);
    int col = static_cast<int>(feature.pt.x / grid_width);
    new_feature_sieve[
      row*processor_config.grid_col+col].push_back(feature);
  }

  new_features.clear();
  for (auto& item : new_feature_sieve) {
    if (item.size() > processor_config.grid_max_feature_num) {
      std::sort(item.begin(), item.end(),
          &ImageProcessor::keyPointCompareByResponse);
      item.erase(
          item.begin()+processor_config.grid_max_feature_num, item.end());
    }
    new_features.insert(new_features.end(), item.begin(), item.end());
  }

  int detected_new_features = new_features.size();

  // Find the stereo matched points for the newly
  // detected features.
  vector<cv::Point2f> cam0_points(new_features.size());
  for (int i = 0; i < new_features.size(); ++i)
    cam0_points[i] = new_features[i].pt;

  vector<cv::Point2f> cam1_points(0);
  vector<unsigned char> inlier_markers(0);
  stereoMatch(cam0_points, cam1_points, inlier_markers);

  vector<cv::Point2f> cam0_inliers(0);
  vector<cv::Point2f> cam1_inliers(0);
  vector<float> response_inliers(0);
  for (int i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] == 0) continue;
    cam0_inliers.push_back(cam0_points[i]);
    cam1_inliers.push_back(cam1_points[i]);
    response_inliers.push_back(new_features[i].response);
  }

  int matched_new_features = cam0_inliers.size();

  if (matched_new_features < 5 &&
      static_cast<double>(matched_new_features)/
      static_cast<double>(detected_new_features) < 0.1) {
    double t = rclcpp::Time(cam0_curr_img_ptr->header.stamp).seconds();
    RCLCPP_WARN(nh_->get_logger(), "Images at [%f] seems unsynced...", t);
  }

  // Group the features into grids
  GridFeatures grid_new_features;
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code)
      grid_new_features[code] = vector<FeatureMetaData>(0);

  for (int i = 0; i < cam0_inliers.size(); ++i) {
    const cv::Point2f& cam0_point = cam0_inliers[i];
    const cv::Point2f& cam1_point = cam1_inliers[i];
    const float& response = response_inliers[i];

    int row = static_cast<int>(cam0_point.y / grid_height);
    int col = static_cast<int>(cam0_point.x / grid_width);
    int code = row*processor_config.grid_col + col;

    FeatureMetaData new_feature;
    new_feature.response = response;
    new_feature.cam0_point = cam0_point;
    new_feature.cam1_point = cam1_point;
    grid_new_features[code].push_back(new_feature);
  }

  // Sort the new features in each grid based on its response.
  for (auto& item : grid_new_features)
    std::sort(item.second.begin(), item.second.end(),
        &ImageProcessor::featureCompareByResponse);

  int new_added_feature_num = 0;
  // Collect new features within each grid with high response.
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code) {
    vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
    vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

    if (features_this_grid.size() >=
        processor_config.grid_min_feature_num) continue;

    int vacancy_num = processor_config.grid_min_feature_num -
      features_this_grid.size();
    for (int k = 0;
        k < vacancy_num && k < new_features_this_grid.size(); ++k) {
      features_this_grid.push_back(new_features_this_grid[k]);
      features_this_grid.back().id = next_feature_id++;
      features_this_grid.back().lifetime = 1;

      ++new_added_feature_num;
    }
  }

  //printf("\033[0;33m detected: %d; matched: %d; new added feature: %d\033[0m\n",
  //    detected_new_features, matched_new_features, new_added_feature_num);

  return;
}

void ImageProcessor::pruneGridFeatures() {
  // 网格内裁剪：如果某个网格的特征数量超过 grid_max_feature_num，
  // 则根据 lifetime 排序（一般优先保留更“稳定/寿命长”的点），删除多余的。
  // 目的：
  //   - 控制计算量
  //   - 保持特征在整幅图上的均匀分布（结合网格策略）

  for (auto& item : *curr_features_ptr) {
    auto& grid_features = item.second;
    // Continue if the number of features in this grid does
    // not exceed the upper bound.
    if (grid_features.size() <=
        processor_config.grid_max_feature_num) continue;
    std::sort(grid_features.begin(), grid_features.end(),
        &ImageProcessor::featureCompareByLifetime);
    grid_features.erase(grid_features.begin()+
        processor_config.grid_max_feature_num,
        grid_features.end());
  }
  return;
}

void ImageProcessor::undistortPoints(
    const vector<cv::Point2f>& pts_in,
    const cv::Vec4d& intrinsics,
    const string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    vector<cv::Point2f>& pts_out,
    const cv::Matx33d &rectification_matrix,
    const cv::Vec4d &new_intrinsics) {

  if (pts_in.size() == 0) return;

  const cv::Matx33d K(
      intrinsics[0], 0.0, intrinsics[2],
      0.0, intrinsics[1], intrinsics[3],
      0.0, 0.0, 1.0);

  const cv::Matx33d K_new(
      new_intrinsics[0], 0.0, new_intrinsics[2],
      0.0, new_intrinsics[1], new_intrinsics[3],
      0.0, 0.0, 1.0);

  if (distortion_model == "radtan") {
    cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                        rectification_matrix, K_new);
  } else if (distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                                 rectification_matrix, K_new);
  } else {
    RCLCPP_WARN_ONCE(nh_->get_logger(), "The model %s is unrecognized, use radtan instead...",
                  distortion_model.c_str());
    cv::undistortPoints(pts_in, pts_out, K, distortion_coeffs,
                        rectification_matrix, K_new);
  }

  return;
}

vector<cv::Point2f> ImageProcessor::distortPoints(
    const vector<cv::Point2f>& pts_in,
    const cv::Vec4d& intrinsics,
    const string& distortion_model,
    const cv::Vec4d& distortion_coeffs) {

  const cv::Matx33d K(intrinsics[0], 0.0, intrinsics[2],
                      0.0, intrinsics[1], intrinsics[3],
                      0.0, 0.0, 1.0);

  vector<cv::Point2f> pts_out;
  if (distortion_model == "radtan") {
    vector<cv::Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  } else if (distortion_model == "equidistant") {
    cv::fisheye::distortPoints(pts_in, pts_out, K, distortion_coeffs);
  } else {
    RCLCPP_WARN_ONCE(nh_->get_logger(), "The model %s is unrecognized, using radtan instead...",
                  distortion_model.c_str());
    vector<cv::Point3f> homogenous_pts;
    cv::convertPointsToHomogeneous(pts_in, homogenous_pts);
    cv::projectPoints(homogenous_pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), K,
                      distortion_coeffs, pts_out);
  }

  return pts_out;
}

void ImageProcessor::integrateImuData(
    Matx33f& cam0_R_p_c, Matx33f& cam1_R_p_c) {
  // IMU 预积分（简化版，仅估计两帧间相对旋转）：
  //   - 在 imu_msg_buffer 中找到时间落在 [prev_img_time, curr_img_time] 附近的 IMU 段
  //   - 计算该段角速度均值（IMU frame）
  //   - 转到 cam0 坐标系，再用 Rodrigues(ω*dt) 得到旋转
  // 输出：cam0_R_p_c = 从 prev 到 curr 的相对旋转（后面会取转置以匹配方向约定）
  //
  // 注意：这里没有积分加速度、没有估计平移；它的主要用途是给像素跟踪提供一个更好的初值。

  // Find the start and the end limit within the imu msg buffer.
  auto begin_iter = imu_msg_buffer.begin();
  while (begin_iter != imu_msg_buffer.end()) {
    rclcpp::Time t_imu(begin_iter->header.stamp);
    rclcpp::Time t_prev(cam0_prev_img_ptr->header.stamp);
    if ((t_imu - t_prev).seconds() < -0.01)
      ++begin_iter;
    else
      break;
  }

  auto end_iter = begin_iter;
  while (end_iter != imu_msg_buffer.end()) {
    rclcpp::Time t_imu(end_iter->header.stamp);
    rclcpp::Time t_curr(cam0_curr_img_ptr->header.stamp);
    if ((t_imu - t_curr).seconds() < 0.005)
      ++end_iter;
    else
      break;
  }

  // Compute the mean angular velocity in the IMU frame.
  // ----------------- 核心计算 1：平均角速度（IMU 系） -----------------
  // 这里没有逐条积分，而是“求均值角速度 * 总 dt”的简化积分。
  Vec3f mean_ang_vel(0.0, 0.0, 0.0);
  for (auto iter = begin_iter; iter < end_iter; ++iter)
    mean_ang_vel += Vec3f(iter->angular_velocity.x,
        iter->angular_velocity.y, iter->angular_velocity.z);

  if (end_iter-begin_iter > 0)
    mean_ang_vel *= 1.0f / (end_iter-begin_iter);
  // --------------------------------------------------------------

  // ----------------- 核心计算 2：角速度从 IMU 系转到相机系 -----------------
  // R_cam*_imu 为外参旋转（由 loadParameters() 解析得到）。
  // 通过转置把 IMU 系角速度转换到 cam0/cam1 坐标系下。
  Vec3f cam0_mean_ang_vel = R_cam0_imu.t() * mean_ang_vel;
  Vec3f cam1_mean_ang_vel = R_cam1_imu.t() * mean_ang_vel;
  // --------------------------------------------------------------------

  // ----------------- 核心计算 3：Rodrigues(ω*dt) 得到相对旋转 -----------------
  // dtime 为两帧图像时间差。
  // 使用旋转向量：theta = ω * dt，通过 Rodrigues 映射到 SO(3)。
  rclcpp::Time t_curr(cam0_curr_img_ptr->header.stamp);
  rclcpp::Time t_prev(cam0_prev_img_ptr->header.stamp);
  double dtime = (t_curr - t_prev).seconds();

  Rodrigues(cam0_mean_ang_vel*dtime, cam0_R_p_c);
  Rodrigues(cam1_mean_ang_vel*dtime, cam1_R_p_c);

  // 方向约定修正：取转置以匹配后续 predictFeatureTracking() 中的使用方向。
  cam0_R_p_c = cam0_R_p_c.t();
  cam1_R_p_c = cam1_R_p_c.t();
  // --------------------------------------------------------------------

  // Delete the useless and used imu messages.
  imu_msg_buffer.erase(imu_msg_buffer.begin(), end_iter);
  return;
}

void ImageProcessor::rescalePoints(
    vector<Point2f>& pts1, vector<Point2f>& pts2,
    float& scaling_factor) {

  scaling_factor = 0.0f;

  for (size_t i = 0; i < pts1.size(); ++i) {
    scaling_factor += sqrt(pts1[i].dot(pts1[i]));
    scaling_factor += sqrt(pts2[i].dot(pts2[i]));
  }

  scaling_factor = (pts1.size()+pts2.size()) /
    scaling_factor * sqrt(2.0f);

  for (size_t i = 0; i < pts1.size(); ++i) {
    pts1[i] *= scaling_factor;
    pts2[i] *= scaling_factor;
  }

  return;
}

void ImageProcessor::twoPointRansac(
    const vector<Point2f>& pts1, const vector<Point2f>& pts2,
    const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
    const std::string& distortion_model,
    const cv::Vec4d& distortion_coeffs,
    const double& inlier_error,
    const double& success_probability,
    vector<int>& inlier_markers) {

  // two-point RANSAC（旋转已知的鲁棒去外点）：
  //   - 输入为 prev(pts1) 与 curr(pts2) 的匹配像素点
  //   - 先去畸变，并用已知旋转 R_p_c 对上一帧点做旋转补偿
  //   - 在旋转已知的假设下，两对点即可约束平移方向（因此称 two-point）
  //   - RANSAC 随机采样两对点拟合模型，评估其余点的代数误差，筛选内点
  // 作用：把由于跟踪失败/遮挡/光照变化导致的错误匹配从“时序跟踪结果”中剔除。


  // Check the size of input point size.
  if (pts1.size() != pts2.size())
    RCLCPP_ERROR(nh_->get_logger(), "Sets of different size (%zu and %zu) are used...",
        pts1.size(), pts2.size());

  double norm_pixel_unit = 2.0 / (intrinsics[0]+intrinsics[1]);
  int iter_num = static_cast<int>(
      ceil(log(1-success_probability) / log(1-0.7*0.7)));

  // Initially, mark all points as inliers.
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 1);

  // Undistort all the points.
  vector<Point2f> pts1_undistorted(pts1.size());
  vector<Point2f> pts2_undistorted(pts2.size());
  undistortPoints(
      pts1, intrinsics, distortion_model,
      distortion_coeffs, pts1_undistorted);
  undistortPoints(
      pts2, intrinsics, distortion_model,
      distortion_coeffs, pts2_undistorted);

  // Compenstate the points in the previous image with
  // the relative rotation.
  for (auto& pt : pts1_undistorted) {
    Vec3f pt_h(pt.x, pt.y, 1.0f);
    //Vec3f pt_hc = dR * pt_h;
    Vec3f pt_hc = R_p_c * pt_h;
    pt.x = pt_hc[0];
    pt.y = pt_hc[1];
  }

  // Normalize the points to gain numerical stability.
  float scaling_factor = 0.0f;
  rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
  norm_pixel_unit *= scaling_factor;

  // Compute the difference between previous and current points,
  // which will be used frequently later.
  vector<Point2d> pts_diff(pts1_undistorted.size());
  for (size_t i = 0; i < pts1_undistorted.size(); ++i)
    pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

  // Mark the point pairs with large difference directly.
  // BTW, the mean distance of the rest of the point pairs
  // are computed.
  double mean_pt_distance = 0.0;
  int raw_inlier_cntr = 0;
  for (int i = 0; i < pts_diff.size(); ++i) {
    double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
    // 25 pixel distance is a pretty large tolerance for normal motion.
    // However, to be used with aggressive motion, this tolerance should
    // be increased significantly to match the usage.
    if (distance > 50.0*norm_pixel_unit) {
      inlier_markers[i] = 0;
    } else {
      mean_pt_distance += distance;
      ++raw_inlier_cntr;
    }
  }
  mean_pt_distance /= raw_inlier_cntr;

  // If the current number of inliers is less than 3, just mark
  // all input as outliers. This case can happen with fast
  // rotation where very few features are tracked.
  if (raw_inlier_cntr < 3) {
    for (auto& marker : inlier_markers) marker = 0;
    return;
  }

  // Before doing 2-point RANSAC, we have to check if the motion
  // is degenerated, meaning that there is no translation between
  // the frames, in which case, the model of the RANSAC does not
  // work. If so, the distance between the matched points will
  // be almost 0.
  //if (mean_pt_distance < inlier_error*norm_pixel_unit) {
  if (mean_pt_distance < norm_pixel_unit) {
    //ROS_WARN_THROTTLE(1.0, "Degenerated motion...");
    for (int i = 0; i < pts_diff.size(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (sqrt(pts_diff[i].dot(pts_diff[i])) >
          inlier_error*norm_pixel_unit)
        inlier_markers[i] = 0;
    }
    return;
  }

  // In the case of general motion, the RANSAC model can be applied.
  // The three column corresponds to tx, ty, and tz respectively.
  MatrixXd coeff_t(pts_diff.size(), 3);
  for (int i = 0; i < pts_diff.size(); ++i) {
    coeff_t(i, 0) = pts_diff[i].y;
    coeff_t(i, 1) = -pts_diff[i].x;
    coeff_t(i, 2) = pts1_undistorted[i].x*pts2_undistorted[i].y -
      pts1_undistorted[i].y*pts2_undistorted[i].x;
  }

  vector<int> raw_inlier_idx;
  for (int i = 0; i < inlier_markers.size(); ++i) {
    if (inlier_markers[i] != 0)
      raw_inlier_idx.push_back(i);
  }

  vector<int> best_inlier_set;
  double best_error = 1e10;
  std::mt19937 rng(std::random_device{}());

  for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
    // Randomly select two point pairs.
    // Although this is a weird way of selecting two pairs, but it
    // is able to efficiently avoid selecting repetitive pairs.
    std::uniform_int_distribution<int> dist1(0, static_cast<int>(raw_inlier_idx.size()-1));
    std::uniform_int_distribution<int> dist_diff(1, static_cast<int>(raw_inlier_idx.size()-1));
    int select_idx1 = dist1(rng);
    int select_idx_diff = dist_diff(rng);
    int select_idx2 = select_idx1+select_idx_diff<raw_inlier_idx.size() ?
      select_idx1+select_idx_diff :
      select_idx1+select_idx_diff-raw_inlier_idx.size();

    int pair_idx1 = raw_inlier_idx[select_idx1];
    int pair_idx2 = raw_inlier_idx[select_idx2];

    // Construct the model;
    Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
    Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
    Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
    vector<double> coeff_l1_norm(3);
    coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
    coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
    coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
    int base_indicator = min_element(coeff_l1_norm.begin(),
        coeff_l1_norm.end())-coeff_l1_norm.begin();

    Vector3d model(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      Matrix2d A;
      A << coeff_ty, coeff_tz;
      Vector2d solution = A.inverse() * (-coeff_tx);
      model(0) = 1.0;
      model(1) = solution(0);
      model(2) = solution(1);
    } else if (base_indicator ==1) {
      Matrix2d A;
      A << coeff_tx, coeff_tz;
      Vector2d solution = A.inverse() * (-coeff_ty);
      model(0) = solution(0);
      model(1) = 1.0;
      model(2) = solution(1);
    } else {
      Matrix2d A;
      A << coeff_tx, coeff_ty;
      Vector2d solution = A.inverse() * (-coeff_tz);
      model(0) = solution(0);
      model(1) = solution(1);
      model(2) = 1.0;
    }

    // Find all the inliers among point pairs.
    VectorXd error = coeff_t * model;

    vector<int> inlier_set;
    for (int i = 0; i < error.rows(); ++i) {
      if (inlier_markers[i] == 0) continue;
      if (std::abs(error(i)) < inlier_error*norm_pixel_unit)
        inlier_set.push_back(i);
    }

    // If the number of inliers is small, the current
    // model is probably wrong.
    if (inlier_set.size() < 0.2*pts1_undistorted.size())
      continue;

    // Refit the model using all of the possible inliers.
    VectorXd coeff_tx_better(inlier_set.size());
    VectorXd coeff_ty_better(inlier_set.size());
    VectorXd coeff_tz_better(inlier_set.size());
    for (int i = 0; i < inlier_set.size(); ++i) {
      coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
      coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
      coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
    }

    Vector3d model_better(0.0, 0.0, 0.0);
    if (base_indicator == 0) {
      MatrixXd A(inlier_set.size(), 2);
      A << coeff_ty_better, coeff_tz_better;
      Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
      model_better(0) = 1.0;
      model_better(1) = solution(0);
      model_better(2) = solution(1);
    } else if (base_indicator ==1) {
      MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_tz_better;
      Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
      model_better(0) = solution(0);
      model_better(1) = 1.0;
      model_better(2) = solution(1);
    } else {
      MatrixXd A(inlier_set.size(), 2);
      A << coeff_tx_better, coeff_ty_better;
      Vector2d solution =
          (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
      model_better(0) = solution(0);
      model_better(1) = solution(1);
      model_better(2) = 1.0;
    }

    // Compute the error and upate the best model if possible.
    VectorXd new_error = coeff_t * model_better;

    double this_error = 0.0;
    for (const auto& inlier_idx : inlier_set)
      this_error += std::abs(new_error(inlier_idx));
    this_error /= inlier_set.size();

    if (inlier_set.size() > best_inlier_set.size()) {
      best_error = this_error;
      best_inlier_set = inlier_set;
    }
  }

  // Fill in the markers.
  inlier_markers.clear();
  inlier_markers.resize(pts1.size(), 0);
  for (const auto& inlier_idx : best_inlier_set)
    inlier_markers[inlier_idx] = 1;

  //printf("inlier ratio: %lu/%lu\n",
  //    best_inlier_set.size(), inlier_markers.size());

  return;
}

void ImageProcessor::publish() {

  // =========================== 发布环节（publish） ===========================
  // 本函数会发布以下 ROS2 消息（topics 在 createRosIO() 中创建）：
  //
  // 1) features  (asr_sdm_perception_msgs::msg::CameraMeasurement)
  //    - header.stamp：使用 cam0 当前帧时间戳
  //    - features[i].id：特征唯一 id
  //    - features[i].u0,v0：cam0 去畸变后的像素坐标（undistortPoints 后）
  //    - features[i].u1,v1：双目时为 cam1 去畸变坐标；单目时填 0
  //
  // 2) tracking_info (asr_sdm_perception_msgs::msg::TrackingInfo)
  //    - before_tracking / after_tracking / after_matching / after_ransac：用于监控跟踪质量
  //
  // 3)（可选，仅用于 RViz2 可视化）feature_points / feature_points_tracked / feature_points_new
  //    - 类型：sensor_msgs::msg::PointCloud2
  //    - 内容：把像素坐标归一化到 [-1,1]，z=0，仅用于显示特征分布
  //
  // 说明：
  //   - 后端（MSCKF）真正依赖的是 1) features（以及时间戳对齐）。
  //   - 2)/3) 都属于监控/调试输出，不参与状态估计。
  // ========================================================================


  // Publish features.
  auto feature_msg_ptr = std::make_unique<asr_sdm_perception_msgs::msg::CameraMeasurement>();
  feature_msg_ptr->header.stamp = cam0_curr_img_ptr->header.stamp;

  vector<FeatureIDType> curr_ids(0);
  vector<Point2f> curr_cam0_points(0);
  vector<Point2f> curr_cam1_points(0);

  for (const auto& grid_features : (*curr_features_ptr)) {
    for (const auto& feature : grid_features.second) {
      curr_ids.push_back(feature.id);
      curr_cam0_points.push_back(feature.cam0_point);
      curr_cam1_points.push_back(feature.cam1_point);
    }
  }

  vector<Point2f> curr_cam0_points_undistorted(0);
  vector<Point2f> curr_cam1_points_undistorted(0);

  undistortPoints(
      curr_cam0_points, cam0_intrinsics, cam0_distortion_model,
      cam0_distortion_coeffs, curr_cam0_points_undistorted);
  if (use_stereo_) {
    undistortPoints(
        curr_cam1_points, cam1_intrinsics, cam1_distortion_model,
        cam1_distortion_coeffs, curr_cam1_points_undistorted);
  } else {
    curr_cam1_points_undistorted.clear();
  }

  for (int i = 0; i < curr_ids.size(); ++i) {
    feature_msg_ptr->features.push_back(asr_sdm_perception_msgs::msg::FeatureMeasurement());
    feature_msg_ptr->features[i].id = curr_ids[i];
    feature_msg_ptr->features[i].u0 = curr_cam0_points_undistorted[i].x;
    feature_msg_ptr->features[i].v0 = curr_cam0_points_undistorted[i].y;
    if (use_stereo_ && i < static_cast<int>(curr_cam1_points_undistorted.size())) {
      feature_msg_ptr->features[i].u1 = curr_cam1_points_undistorted[i].x;
      feature_msg_ptr->features[i].v1 = curr_cam1_points_undistorted[i].y;
    } else {
      feature_msg_ptr->features[i].u1 = 0.0f;
      feature_msg_ptr->features[i].v1 = 0.0f;
    }
  }

  feature_pub_->publish(std::move(feature_msg_ptr));

  // Publish tracking info.
  auto tracking_info_msg_ptr = std::make_unique<asr_sdm_perception_msgs::msg::TrackingInfo>();
  tracking_info_msg_ptr->header.stamp = cam0_curr_img_ptr->header.stamp;
  tracking_info_msg_ptr->before_tracking = before_tracking;
  tracking_info_msg_ptr->after_tracking = after_tracking;
  tracking_info_msg_ptr->after_matching = after_matching;
  tracking_info_msg_ptr->after_ransac = after_ransac;
  tracking_info_pub_->publish(std::move(tracking_info_msg_ptr));

  // Publish feature points as PointCloud2 for RViz2 visualization (all points).
  if (feature_cloud_pub_ && feature_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = cam0_curr_img_ptr->header;  // same stamp and frame as cam0 image
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(curr_cam0_points.size());
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = false;

    // Define fields x, y, z (float32)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud_msg.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    const int img_width  = cam0_curr_img_ptr->image.cols;
    const int img_height = cam0_curr_img_ptr->image.rows;

    for (size_t i = 0; i < curr_cam0_points.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      const auto& pt = curr_cam0_points[i];

      // Normalize pixel coordinates to [-1, 1] range so they are easy to see in RViz2.
      float xn = (img_width  > 0) ? (pt.x / static_cast<float>(img_width))  * 2.0f - 1.0f : 0.0f;
      float yn = (img_height > 0) ? (pt.y / static_cast<float>(img_height)) * 2.0f - 1.0f : 0.0f;

      *iter_x = xn;
      *iter_y = yn;
      *iter_z = 0.0f;      // on image plane z = 0
    }

    feature_cloud_pub_->publish(cloud_msg);
  }

  // Helper lambda to build a normalized cloud from a vector of points.
  auto make_cloud = [&](const std::vector<cv::Point2f>& pts) -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = cam0_curr_img_ptr->header;
    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(pts.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const int img_width  = cam0_curr_img_ptr->image.cols;
    const int img_height = cam0_curr_img_ptr->image.rows;

    for (size_t i = 0; i < pts.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      const auto& pt = pts[i];
      float xn = (img_width  > 0) ? (pt.x / static_cast<float>(img_width))  * 2.0f - 1.0f : 0.0f;
      float yn = (img_height > 0) ? (pt.y / static_cast<float>(img_height)) * 2.0f - 1.0f : 0.0f;
      *iter_x = xn;
      *iter_y = yn;
      *iter_z = 0.0f;
    }
    return cloud;
  };

  // 可视化用：按“新点/老点”拆分（lifetime==1 为新点）。
  // 注意：当前实现并没有在上面收集 curr_cam0_points 时同步收集 lifetime，
  // 所以这里无法真正区分新点/老点，最终会退化为：tracked=全部点、new=空。
  // 若需要正确区分：在前面遍历 curr_features_ptr 时同步收集 feature.lifetime。
  std::vector<cv::Point2f> tracked_pts;
  std::vector<cv::Point2f> new_pts;
  tracked_pts.reserve(curr_cam0_points.size());
  new_pts.reserve(curr_cam0_points.size());

  for (size_t i = 0; i < curr_cam0_points.size(); ++i) {
    tracked_pts.push_back(curr_cam0_points[i]);
  }

  if (feature_cloud_tracked_pub_ && feature_cloud_tracked_pub_->get_subscription_count() > 0) {
    auto cloud_tracked = make_cloud(tracked_pts);
    feature_cloud_tracked_pub_->publish(cloud_tracked);
  }
  if (feature_cloud_new_pub_ && feature_cloud_new_pub_->get_subscription_count() > 0) {
    auto cloud_new = make_cloud(new_pts);
    feature_cloud_new_pub_->publish(cloud_new);
  }

  return;
}

void ImageProcessor::drawFeaturesMono() {
  // Colors for different features.
  Scalar tracked(0, 255, 0);
  Scalar new_feature(0, 255, 255);

  static int grid_height =
    cam0_curr_img_ptr->image.rows / processor_config.grid_row;
  static int grid_width =
    cam0_curr_img_ptr->image.cols / processor_config.grid_col;

  // Create an output image.
  int img_height = cam0_curr_img_ptr->image.rows;
  int img_width = cam0_curr_img_ptr->image.cols;
  Mat out_img(img_height, img_width, CV_8UC3);
  cvtColor(cam0_curr_img_ptr->image, out_img, CV_GRAY2RGB);

  // Draw grids on the image.
  for (int i = 1; i < processor_config.grid_row; ++i) {
    Point pt1(0, i*grid_height);
    Point pt2(img_width, i*grid_height);
    line(out_img, pt1, pt2, Scalar(255, 0, 0));
  }
  for (int i = 1; i < processor_config.grid_col; ++i) {
    Point pt1(i*grid_width, 0);
    Point pt2(i*grid_width, img_height);
    line(out_img, pt1, pt2, Scalar(255, 0, 0));
  }

  // Collect features ids in the previous frame.
  vector<FeatureIDType> prev_ids(0);
  for (const auto& grid_features : *prev_features_ptr)
    for (const auto& feature : grid_features.second)
      prev_ids.push_back(feature.id);

  // Collect feature points in the previous frame.
  map<FeatureIDType, Point2f> prev_points;
  for (const auto& grid_features : *prev_features_ptr)
    for (const auto& feature : grid_features.second)
      prev_points[feature.id] = feature.cam0_point;

  // Collect feature points in the current frame.
  map<FeatureIDType, Point2f> curr_points;
  for (const auto& grid_features : *curr_features_ptr)
    for (const auto& feature : grid_features.second)
      curr_points[feature.id] = feature.cam0_point;

  // Draw tracked features.
  for (const auto& id : prev_ids) {
    if (prev_points.find(id) != prev_points.end() &&
        curr_points.find(id) != curr_points.end()) {
      cv::Point2f prev_pt = prev_points[id];
      cv::Point2f curr_pt = curr_points[id];
      circle(out_img, curr_pt, 3, tracked);
      line(out_img, prev_pt, curr_pt, tracked, 1);

      prev_points.erase(id);
      curr_points.erase(id);
    }
  }

  // Draw new features.
  for (const auto& new_curr_point : curr_points) {
    cv::Point2f pt = new_curr_point.second;
    circle(out_img, pt, 3, new_feature, -1);
  }

  // Publish debug mono image for RViz2
  if (debug_mono_pub_) {
    std_msgs::msg::Header header;
    header.stamp = cam0_curr_img_ptr->header.stamp;
    header.frame_id = cam0_curr_img_ptr->header.frame_id;
    cv_bridge::CvImage cv_img(header, "bgr8", out_img);
    debug_mono_pub_.publish(cv_img.toImageMsg());
  }

  imshow("Feature", out_img);
  waitKey(5);
}

void ImageProcessor::drawFeaturesStereo() {

  if(!use_stereo_) {
    drawFeaturesMono();
    return;
  }

  if(debug_stereo_pub_.getNumSubscribers() > 0)
  {
    // Colors for different features.
    Scalar tracked(0, 255, 0);
    Scalar new_feature(0, 255, 255);

    static int grid_height =
      cam0_curr_img_ptr->image.rows / processor_config.grid_row;
    static int grid_width =
      cam0_curr_img_ptr->image.cols / processor_config.grid_col;

    // Create an output image.
    int img_height = cam0_curr_img_ptr->image.rows;
    int img_width = cam0_curr_img_ptr->image.cols;
    Mat out_img(img_height, img_width*2, CV_8UC3);
    cvtColor(cam0_curr_img_ptr->image,
             out_img.colRange(0, img_width), CV_GRAY2RGB);
    cvtColor(cam1_curr_img_ptr->image,
             out_img.colRange(img_width, img_width*2), CV_GRAY2RGB);

    // Draw grids on the image.
    for (int i = 1; i < processor_config.grid_row; ++i) {
      Point pt1(0, i*grid_height);
      Point pt2(img_width*2, i*grid_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < processor_config.grid_col; ++i) {
      Point pt1(i*grid_width, 0);
      Point pt2(i*grid_width, img_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }
    for (int i = 1; i < processor_config.grid_col; ++i) {
      Point pt1(i*grid_width+img_width, 0);
      Point pt2(i*grid_width+img_width, img_height);
      line(out_img, pt1, pt2, Scalar(255, 0, 0));
    }

    // Collect features ids in the previous frame.
    vector<FeatureIDType> prev_ids(0);
    for (const auto& grid_features : *prev_features_ptr)
      for (const auto& feature : grid_features.second)
        prev_ids.push_back(feature.id);

    // Collect feature points in the previous frame.
    map<FeatureIDType, Point2f> prev_cam0_points;
    map<FeatureIDType, Point2f> prev_cam1_points;
    for (const auto& grid_features : *prev_features_ptr)
      for (const auto& feature : grid_features.second) {
        prev_cam0_points[feature.id] = feature.cam0_point;
        prev_cam1_points[feature.id] = feature.cam1_point;
      }

    // Collect feature points in the current frame.
    map<FeatureIDType, Point2f> curr_cam0_points;
    map<FeatureIDType, Point2f> curr_cam1_points;
    for (const auto& grid_features : *curr_features_ptr)
      for (const auto& feature : grid_features.second) {
        curr_cam0_points[feature.id] = feature.cam0_point;
        curr_cam1_points[feature.id] = feature.cam1_point;
      }

    // Draw tracked features.
    for (const auto& id : prev_ids) {
      if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
          curr_cam0_points.find(id) != curr_cam0_points.end()) {
        cv::Point2f prev_pt0 = prev_cam0_points[id];
        cv::Point2f prev_pt1 = prev_cam1_points[id] + Point2f(img_width, 0.0);
        cv::Point2f curr_pt0 = curr_cam0_points[id];
        cv::Point2f curr_pt1 = curr_cam1_points[id] + Point2f(img_width, 0.0);

        circle(out_img, curr_pt0, 3, tracked, -1);
        circle(out_img, curr_pt1, 3, tracked, -1);
        line(out_img, prev_pt0, curr_pt0, tracked, 1);
        line(out_img, prev_pt1, curr_pt1, tracked, 1);

        prev_cam0_points.erase(id);
        prev_cam1_points.erase(id);
        curr_cam0_points.erase(id);
        curr_cam1_points.erase(id);
      }
    }

    // Draw new features.
    for (const auto& new_cam0_point : curr_cam0_points) {
      cv::Point2f pt0 = new_cam0_point.second;
      cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] +
        Point2f(img_width, 0.0);

      circle(out_img, pt0, 3, new_feature, -1);
      circle(out_img, pt1, 3, new_feature, -1);
    }

    cv_bridge::CvImage debug_image(cam0_curr_img_ptr->header, "bgr8", out_img);
    debug_stereo_pub_.publish(debug_image.toImageMsg());
  }
  //imshow("Feature", out_img);
  //waitKey(5);

  return;
}

void ImageProcessor::updateFeatureLifetime() {
  for (int code = 0; code <
      processor_config.grid_row*processor_config.grid_col; ++code) {
    vector<FeatureMetaData>& features = (*curr_features_ptr)[code];
    for (const auto& feature : features) {
      if (feature_lifetime.find(feature.id) == feature_lifetime.end())
        feature_lifetime[feature.id] = 1;
      else
        ++feature_lifetime[feature.id];
    }
  }

  return;
}

void ImageProcessor::featureLifetimeStatistics() {

  map<int, int> lifetime_statistics;
  for (const auto& data : feature_lifetime) {
    if (lifetime_statistics.find(data.second) ==
        lifetime_statistics.end())
      lifetime_statistics[data.second] = 1;
    else
      ++lifetime_statistics[data.second];
  }

  for (const auto& data : lifetime_statistics)
    cout << data.first << " : " << data.second << endl;

  return;
}

} // end namespace msckf_vio
