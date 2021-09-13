#pragma once

#include <memory>
#include <string>

namespace ros {
  struct NodeHandle;
}

namespace cyclops_ros {
  struct camera_config_t {
    struct camera_intrinsic_t {
      double fx;
      double fy;
      double cx;
      double cy;
    };

    struct camera_distortion_t {
      double k1;
      double k2;
      double p1;
      double p2;
    };

    int width;
    int height;
    camera_intrinsic_t intrinsic;
    camera_distortion_t distortion;
  };

  struct tracker_config_t {
    int max_features;
    int feature_min_distance;

    int tracking_patch_size;

    double epipolar_ransac_pixel_noise;
    double epipolar_ransac_confidence_threshold;
    double corner_detection_quality_threshold;
    double image_noise_stddev;

    double track_update_fps_target;
    double track_update_fps_filter_window_size;
  };

  struct cyclops_ros_frontend_config_t {
    std::string image_topic_name;

    camera_config_t camera_config;
    tracker_config_t tracker_config;
  };

  std::unique_ptr<cyclops_ros_frontend_config_t const> read_config(
    ros::NodeHandle& node_handle);
}  // namespace cyclops_ros
