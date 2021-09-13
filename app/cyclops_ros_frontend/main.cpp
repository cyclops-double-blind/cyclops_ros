#include "cyclops_ros_frontend/config.hpp"
#include "cyclops_ros_frontend/tracker.hpp"
#include "cyclops_ros_frontend/throttle.hpp"

#include "cyclops_ros/NormalizedFeatureSet.h"
#include "cyclops_ros/RawFeatureSet.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

class CyclopsRosFrontendContext {
private:
  std::shared_ptr<cyclops_ros::cyclops_ros_frontend_config_t const> _config;
  std::unique_ptr<cyclops_ros::CyclopsKltFeatureTracker> _tracker;
  std::unique_ptr<cyclops_ros::CyclopsFeatureTrackUpdateThrottle>
    _track_update_throttle;

  ros::Publisher _normal_feature_publisher;
  ros::Publisher _raw_track_publisher;

  double _last_track_update_timestamp;

  std::optional<Eigen::Matrix2f> clampInformationWeight(
    Eigen::Matrix2f const& information, float clamp_threshold) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(information);
    if (eigensolver.info() != Eigen::Success)
      return std::nullopt;

    auto const& lambda = eigensolver.eigenvalues();
    auto const& Q = eigensolver.eigenvectors();
    auto lambda_clamped = lambda.array().min(clamp_threshold).matrix().eval();

    return (Q * lambda_clamped.asDiagonal() * Q.transpose()).eval();
  }

  bool copyInformationWeight(
    boost::array<float, 4>& result, cv::Mat const& information,
    float clamp_threshold) const {
    if (information.empty())
      return false;

    Eigen::Matrix2f eigen_information;
    cv::cv2eigen(information, eigen_information);

    auto clamped_information =
      clampInformationWeight(eigen_information, clamp_threshold);
    if (!clamped_information.has_value())
      return false;

    Eigen::Map<Eigen::Matrix2f> result_map(result.data());
    result_map = *clamped_information;
    return true;
  }

  void publishNormalFeatures(std_msgs::Header const& header) {
    cyclops_ros::NormalizedFeatureSet result;
    result.header = header;

    auto features = _tracker->features();
    result.features.reserve(features.size());

    auto information_clamp = std::pow(450 / 1, 2);
    for (auto const& [id, feature] : features) {
      cyclops_ros::NormalizedFeature feature_msg;
      if (!copyInformationWeight(
            feature_msg.weight, feature.information, information_clamp)) {
        continue;
      }

      feature_msg.id = id;
      feature_msg.u = feature.point.x;
      feature_msg.v = feature.point.y;

      result.features.emplace_back(feature_msg);
    }
    _normal_feature_publisher.publish(result);
  }

  void publishRawTracks(std_msgs::Header const& header) {
    cyclops_ros::RawFeatureSet result;
    result.header = header;
    result.patch_size = _config->tracker_config.tracking_patch_size;

    auto const& tracks = _tracker->tracks();
    result.features.reserve(tracks.size());

    auto information_clamp = std::pow(1.0 / 1, 2);
    for (auto const& [id, track] : tracks) {
      cyclops_ros::RawFeature feature_msg;
      if (!copyInformationWeight(
            feature_msg.weight, track.feature.information, information_clamp)) {
        continue;
      }

      feature_msg.id = id;
      feature_msg.u = track.feature.point.x;
      feature_msg.v = track.feature.point.y;
      feature_msg.age = track.track_count;
      result.features.emplace_back(feature_msg);
    }
    _raw_track_publisher.publish(result);
  }

public:
  CyclopsRosFrontendContext(
    std::shared_ptr<cyclops_ros::cyclops_ros_frontend_config_t const> config,
    std::unique_ptr<cyclops_ros::CyclopsKltFeatureTracker> tracker,
    std::unique_ptr<cyclops_ros::CyclopsFeatureTrackUpdateThrottle>
      track_update_throttle,
    ros::NodeHandle& node, ros::NodeHandle& pnode)
      : _config(config),
        _tracker(std::move(tracker)),
        _track_update_throttle(std::move(track_update_throttle)) {
    auto it = image_transport::ImageTransport(pnode);

    _normal_feature_publisher =
      node.advertise<cyclops_ros::NormalizedFeatureSet>("cyclops_features", 1);
    _raw_track_publisher =
      node.advertise<cyclops_ros::RawFeatureSet>("cyclops_track_raw", 1);
    _last_track_update_timestamp = 0;
  }

  void handleImage(sensor_msgs::ImageConstPtr const& image) {
    auto tic = std::chrono::steady_clock::now();

    auto cv_image = cv_bridge::toCvCopy(image, "mono8");
    if (cv_image == nullptr) {
      ROS_ERROR_NAMED(
        "cyclops_ros",
        "failed to convert image to cv_image; check your image formatting.");
      return;
    }
    _tracker->followTracks(cv_image->image);

    if (_track_update_throttle->update(image->header.stamp.toSec())) {
      _tracker->updateTracks();

      publishRawTracks(image->header);
      publishNormalFeatures(image->header);
    }

    std::chrono::duration<double> dt = std::chrono::steady_clock::now() - tic;
    ROS_DEBUG_STREAM_NAMED(
      "cyclops_ros",
      "Feature tracker image callback time: " << dt.count() << " [s]");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cyclops_ros_frontend");

  ros::NodeHandle node;
  ros::NodeHandle pnode("~");

  srand(20220208);
  cv::theRNG().state = 20220208;

  {
    auto log_level = std::min(
      pnode.param<int>("log_level", 1), (int)(ros::console::levels::Warn));
    ROS_INFO_NAMED("cyclops_ros", "changing log level to %d", log_level);
    if (!ros::console::set_logger_level(
          ROSCONSOLE_DEFAULT_NAME, (ros::console::levels::Level)log_level)) {
      ROS_ERROR_NAMED("cyclops_ros", "failed to change log level");
      return -1;
    }
    ros::console::notifyLoggerLevelsChanged();
  }
  cv::setNumThreads(0);

  std::shared_ptr config = cyclops_ros::read_config(pnode);
  if (config == nullptr) {
    ROS_ERROR_NAMED(
      "cyclops_ros",
      "failed to read frontend configuration parameter. aborting...");
    ROS_ERROR_NAMED(
      "cyclops_ros",
      "note: set parameter `skip_config_check = true` to ignore this check.");
    return -1;
  }

  auto tracker =
    std::make_unique<cyclops_ros::CyclopsKltFeatureTracker>(config);
  auto throttle =
    std::make_unique<cyclops_ros::CyclopsFeatureTrackUpdateThrottle>(config);

  auto context = CyclopsRosFrontendContext(
    config, std::move(tracker), std::move(throttle), node, pnode);
  auto _ = node.subscribe(
    config->image_topic_name, 32, &CyclopsRosFrontendContext::handleImage,
    &context);

  ROS_DEBUG_NAMED("cyclops_ros", "Ready to run. spinning...");
  ros::spin();

  return 0;
}
