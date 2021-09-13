#pragma once

#include <opencv2/opencv.hpp>

#include <map>
#include <memory>
#include <vector>

namespace cyclops_ros {
  struct cyclops_ros_frontend_config_t;

  using feature_id_t = int;

  struct feature_point_t {
    cv::Point2f point;
    cv::Mat information;
  };

  struct feature_track_t {
    int track_count;
    feature_point_t feature;
  };

  class CyclopsKltFeatureTracker {
  private:
    std::shared_ptr<cyclops_ros_frontend_config_t const> _config;

    int _last_feature_id = 0;
    std::map<feature_id_t, feature_track_t> _tracks;

    cv::Mat _mask;
    cv::Mat _prev_image;

    void findNewTracks(cv::Mat const& image);
    void updateTrackMask(cv::Mat const& image);
    bool isFeatureInBorder(cv::Point2f const& pt);

    std::vector<uint8_t> testEpipolarGeometry(
      std::vector<cv::Point2f> const& prev_features,
      std::vector<cv::Point2f> const& curr_features);

  public:
    explicit CyclopsKltFeatureTracker(
      std::shared_ptr<cyclops_ros_frontend_config_t const> config);
    ~CyclopsKltFeatureTracker();

    void followTracks(cv::Mat const& image);
    void updateTracks();

    std::map<feature_id_t, feature_point_t> features() const;
    std::map<feature_id_t, feature_track_t> const& tracks() const;
  };
}  // namespace cyclops_ros
