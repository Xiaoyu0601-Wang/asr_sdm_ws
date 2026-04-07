#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace asr_sdm_video_enhancement
{

using sensor_msgs::msg::CompressedImage;
using SteadyClock = std::chrono::steady_clock;

class McapPlayerNode : public rclcpp::Node
{
public:
  explicit McapPlayerNode(const rclcpp::NodeOptions & options)
  : Node("asr_sdm_mcap_player", options)
  {
    bag_path_ = declare_parameter<std::string>("bag_path", "/tmp/water_mcap_test");
    loop_ = declare_parameter<bool>("loop", false);
    playback_hz_ = declare_parameter<double>("playback_hz", 0.0);

    if (!preparePlayback()) {
      requestShutdown();
      return;
    }

    playback_thread_ = std::thread(&McapPlayerNode::playbackLoop, this);
  }

  ~McapPlayerNode() override
  {
    signalStop();
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
  }

private:
  bool preparePlayback()
  {
    if (!std::filesystem::exists(bag_path_)) {
      RCLCPP_ERROR(get_logger(), "Bag path does not exist: %s", bag_path_.c_str());
      return false;
    }
    if (playback_hz_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "playback_hz must be >= 0, got %.6f", playback_hz_);
      return false;
    }

    try {
      rosbag2_cpp::Reader reader;
      openReader(reader);
      compressed_topic_name_ = findCompressedImageTopic(reader.get_all_topics_and_types());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to inspect MCAP bag '%s': %s", bag_path_.c_str(), error.what());
      return false;
    }

    pub_compressed_ = create_publisher<CompressedImage>(
      compressed_topic_name_, rclcpp::SensorDataQoS());

    if (playback_hz_ > 0.0) {
      fixed_publish_period_ = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / playback_hz_));
      RCLCPP_INFO(
        get_logger(), "Opened MCAP bag %s -> topic %s (loop=%s, playback_hz=%.3f)",
        bag_path_.c_str(), compressed_topic_name_.c_str(), loop_ ? "true" : "false",
        playback_hz_);
    } else {
      fixed_publish_period_ = SteadyClock::duration::zero();
      RCLCPP_INFO(
        get_logger(), "Opened MCAP bag %s -> topic %s (loop=%s, playback_hz=bag)",
        bag_path_.c_str(), compressed_topic_name_.c_str(), loop_ ? "true" : "false");
    }
    return true;
  }

  void openReader(rosbag2_cpp::Reader & reader) const
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);
  }

  std::string findCompressedImageTopic(
    const std::vector<rosbag2_storage::TopicMetadata> & topics) const
  {
    std::vector<std::string> compressed_topics;
    for (const auto & topic : topics) {
      if (topic.type == "sensor_msgs/msg/CompressedImage") {
        compressed_topics.push_back(topic.name);
      }
    }

    if (compressed_topics.empty()) {
      throw std::runtime_error("no sensor_msgs/msg/CompressedImage topic found");
    }

    if (compressed_topics.size() > 1) {
      std::ostringstream stream;
      stream << "multiple sensor_msgs/msg/CompressedImage topics found:";
      for (const auto & topic_name : compressed_topics) {
        stream << " " << topic_name;
      }
      throw std::runtime_error(stream.str());
    }

    return compressed_topics.front();
  }

  void playbackLoop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        playBagOnce();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to play MCAP bag '%s': %s", bag_path_.c_str(), error.what());
        requestShutdown();
        return;
      }

      if (!loop_) {
        RCLCPP_INFO(get_logger(), "Reached end of %s, stopping player.", bag_path_.c_str());
        requestShutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "Reached end of %s, restarting playback.", bag_path_.c_str());
    }
  }

  void playBagOnce()
  {
    rosbag2_cpp::Reader reader;
    openReader(reader);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {compressed_topic_name_};
    reader.set_filter(filter);

    rclcpp::Serialization<CompressedImage> serialization;
    bool first_message = true;
    rcutils_time_point_value_t previous_bag_timestamp = 0;
    auto next_publish_time = SteadyClock::now();

    while (!stop_requested_.load(std::memory_order_relaxed) && reader.has_next()) {
      auto bag_message = reader.read_next();
      const auto current_bag_timestamp = getPlaybackTimestamp(*bag_message);

      if (!first_message) {
        if (fixed_publish_period_ > SteadyClock::duration::zero()) {
          next_publish_time += fixed_publish_period_;
          if (sleepUntil(next_publish_time)) {
            return;
          }
        } else if (current_bag_timestamp > previous_bag_timestamp) {
          next_publish_time += std::chrono::nanoseconds(
            current_bag_timestamp - previous_bag_timestamp);
          if (sleepUntil(next_publish_time)) {
            return;
          }
        } else {
          next_publish_time = SteadyClock::now();
        }
      } else {
        first_message = false;
        next_publish_time = SteadyClock::now();
      }

      rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
      CompressedImage message;
      serialization.deserialize_message(&serialized_message, &message);
      pub_compressed_->publish(message);

      previous_bag_timestamp = current_bag_timestamp;
    }
  }

  rcutils_time_point_value_t getPlaybackTimestamp(
    const rosbag2_storage::SerializedBagMessage & message) const
  {
    if (message.send_timestamp > 0) {
      return message.send_timestamp;
    }
    return message.recv_timestamp;
  }

  bool sleepUntil(const SteadyClock::time_point & deadline)
  {
    std::unique_lock<std::mutex> lock(stop_mutex_);
    return stop_cv_.wait_until(lock, deadline, [this]() {
      return stop_requested_.load(std::memory_order_relaxed);
    });
  }

  void signalStop()
  {
    stop_requested_.store(true, std::memory_order_relaxed);
    stop_cv_.notify_all();
  }

  void requestShutdown()
  {
    signalStop();
    if (!shutdown_requested_.exchange(true, std::memory_order_relaxed) && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::string bag_path_;
  bool loop_ = false;
  double playback_hz_ = 0.0;
  std::string compressed_topic_name_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  SteadyClock::duration fixed_publish_period_{SteadyClock::duration::zero()};

  rclcpp::Publisher<CompressedImage>::SharedPtr pub_compressed_;
  std::thread playback_thread_;
};

}  // namespace asr_sdm_video_enhancement

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_video_enhancement::McapPlayerNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
