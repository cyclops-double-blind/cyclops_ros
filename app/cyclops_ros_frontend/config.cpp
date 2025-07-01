#include "cyclops_ros_frontend/config.hpp"

#include <ros/console.h>
#include <ros/node_handle.h>

#include <sstream>

namespace cyclops_ros {
  template <typename value_t>
  static value_t readAs(ros::NodeHandle& pnode, std::string name) {
    value_t value;
    if (!pnode.getParam(name, value))
      throw std::domain_error("Unspecified parameter: " + name);
    return value;
  }

  static auto parseCameraDistortion(ros::NodeHandle& pnode)
    -> std::optional<decltype(CameraConfig::distortion)> {
    auto reportError = [](auto reason) {
      ROS_ERROR_STREAM(
        "Failed to read camera distortion parameter: " << reason);
    };

    try {
      auto model = readAs<std::string>(pnode, "camera/distortion/model");
      if (model == "fisheye") {
        return CameraConfig::CameraDistortionFisheye {
          readAs<double>(pnode, "camera/distortion/k1"),
          readAs<double>(pnode, "camera/distortion/k2"),
          readAs<double>(pnode, "camera/distortion/k3"),
          readAs<double>(pnode, "camera/distortion/k4"),
        };
      }

      if (model == "pinhole") {
        return CameraConfig::CameraDistortionPinhole {
          .k1 = readAs<double>(pnode, "camera/distortion/k1"),
          .k2 = readAs<double>(pnode, "camera/distortion/k2"),
          .p1 = readAs<double>(pnode, "camera/distortion/p1"),
          .p2 = readAs<double>(pnode, "camera/distortion/p2"),
        };
      }

      reportError("Unknown distortion model");
      ROS_ERROR_STREAM("Allowed: {fisheye, pinhole}, provided: " << model);
      return std::nullopt;
    } catch (std::domain_error const& err) {
      reportError(err.what());
      return std::nullopt;
    }
  }

  static std::optional<CameraConfig> parseCameraConfig(ros::NodeHandle& pnode) {
    auto distortion = parseCameraDistortion(pnode);
    if (!distortion.has_value())
      return std::nullopt;

    try {
      return CameraConfig {
        .width = readAs<int>(pnode, "camera/width"),
        .height = readAs<int>(pnode, "camera/height"),
        .intrinsic =
          {
            .fx = readAs<double>(pnode, "camera/intrinsic/fx"),
            .fy = readAs<double>(pnode, "camera/intrinsic/fy"),
            .cx = readAs<double>(pnode, "camera/intrinsic/cx"),
            .cy = readAs<double>(pnode, "camera/intrinsic/cy"),
          },
        .distortion = *distortion,
      };
    } catch (std::domain_error const& err) {
      ROS_ERROR_STREAM("Failed to read camera configuration: " << err.what());
      return std::nullopt;
    }
  }

  static TrackerConfig parseTrackerConfig(ros::NodeHandle& pnode) {
    auto config = TrackerConfig {
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

  std::unique_ptr<CyclopsFrontendConfig const> CyclopsFrontendConfig::Parse(
    ros::NodeHandle& pnode) {
    auto maybe_camera_config = parseCameraConfig(pnode);
    if (!maybe_camera_config.has_value()) {
      ROS_ERROR("Failed to parse camera configuration. aborting...");
      return nullptr;
    }

    return std::make_unique<CyclopsFrontendConfig>(CyclopsFrontendConfig {
      .image_topic_name =
        pnode.param<std::string>("image_topic_name", "camera/image"),
      .camera_config = *maybe_camera_config,
      .tracker_config = parseTrackerConfig(pnode),
    });
  }
}  // namespace cyclops_ros
