#include "cyclops_ros_frontend/config.hpp"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <sstream>

namespace cyclops_ros {
  template <typename value_t>
  static value_t read_as(ros::NodeHandle& pnode, std::string name) {
    value_t value;
    if (!pnode.getParam(name, value))
      throw std::domain_error("Unspecified parameter: " + name);
    return value;
  }

  static std::optional<camera_config_t> parse_camera_config(
    ros::NodeHandle& pnode) {
    try {
      return camera_config_t {
        .width = read_as<int>(pnode, "camera/width"),
        .height = read_as<int>(pnode, "camera/height"),
        .intrinsic =
          {
            .fx = read_as<double>(pnode, "camera/intrinsic/fx"),
            .fy = read_as<double>(pnode, "camera/intrinsic/fy"),
            .cx = read_as<double>(pnode, "camera/intrinsic/cx"),
            .cy = read_as<double>(pnode, "camera/intrinsic/cy"),
          },
        .distortion =
          {
            .k1 = read_as<double>(pnode, "camera/distortion/k1"),
            .k2 = read_as<double>(pnode, "camera/distortion/k2"),
            .p1 = read_as<double>(pnode, "camera/distortion/p1"),
            .p2 = read_as<double>(pnode, "camera/distortion/p2"),
          },
      };
    } catch (std::domain_error const& err) {
      ROS_ERROR_STREAM("Failed to read camera configuration: " << err.what());
      return std::nullopt;
    }
  }

  static tracker_config_t parse_tracker_config(ros::NodeHandle& pnode) {
    auto config = tracker_config_t {
      .max_features = pnode.param<int>("max_features", 200),
      .feature_min_distance = pnode.param<int>("feature_min_distance", 30),
      .tracking_patch_size = pnode.param<int>("tracking_patch_size", 21),
      .epipolar_ransac_pixel_noise =
        pnode.param<double>("epipolar_ransac_pixel_noise", 1.0),
      .epipolar_ransac_confidence_threshold =
        pnode.param<double>("epipolar_ransac_confidence_threshold", 0.99),
      .corner_detection_quality_threshold =
        pnode.param<double>("corner_detection_quality_threshold", 0.01),
      .image_noise_stddev =
        pnode.param<double>("image_noise/pixel_white_noise", 10.),
      .track_update_fps_target =
        pnode.param<double>("track_update_fps_target", 10.0),
      .track_update_fps_filter_window_size =
        pnode.param<double>("track_update_fps_filter_window_size", 1.0),
    };
    return config;
  }

  std::unique_ptr<cyclops_ros_frontend_config_t const> read_config(
    ros::NodeHandle& pnode) {
    auto maybe_camera_config = parse_camera_config(pnode);
    if (!maybe_camera_config.has_value()) {
      ROS_ERROR("Failed to parse camera configuration. aborting...");
      return nullptr;
    }

    return std::make_unique<cyclops_ros_frontend_config_t>(
      cyclops_ros_frontend_config_t {
        .image_topic_name =
          pnode.param<std::string>("image_topic_name", "camera/image"),
        .camera_config = *maybe_camera_config,
        .tracker_config = parse_tracker_config(pnode),
      });
  }
}  // namespace cyclops_ros
