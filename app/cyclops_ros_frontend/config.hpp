#pragma once

#include <array>
#include <memory>
#include <string>
#include <variant>

namespace ros {
  struct NodeHandle;
}

namespace cyclops_ros {
  struct CameraConfig {
    struct CameraIntrinsic {
      double fx;
      double fy;
      double cx;
      double cy;
    };

    struct CameraDistortionPinhole {
      double k1;
      double k2;
      double p1;
      double p2;
    };

    struct CameraDistortionFisheye {
      std::array<double, 4> coefficients;
    };

    int width;
    int height;

    CameraIntrinsic intrinsic;
    std::variant<CameraDistortionPinhole, CameraDistortionFisheye> distortion;
  };

  struct TrackerConfig {
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

  struct CyclopsFrontendConfig {
    std::string image_topic_name;

    CameraConfig camera_config;
    TrackerConfig tracker_config;

    static std::unique_ptr<CyclopsFrontendConfig const> Parse(
      ros::NodeHandle& node_handle);
  };
}  // namespace cyclops_ros
