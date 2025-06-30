#pragma once

#include <opencv2/opencv.hpp>

#include <map>
#include <memory>
#include <vector>

namespace cyclops_ros {
  struct CyclopsFrontendConfig;

  using FeatureId = int;

  struct FeaturePoint {
    cv::Point2f point;
    cv::Mat information;
  };

  struct FeatureTrack {
    int track_count;
    FeaturePoint feature;
  };

  class CyclopsKltFeatureTracker {
  private:
    std::shared_ptr<CyclopsFrontendConfig const> _config;

    int _last_feature_id = 0;
    std::map<FeatureId, FeatureTrack> _tracks;

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
      std::shared_ptr<CyclopsFrontendConfig const> config);
    ~CyclopsKltFeatureTracker();

    void followTracks(cv::Mat const& image);
    void updateTracks();

    std::map<FeatureId, FeaturePoint> features() const;
    std::map<FeatureId, FeatureTrack> const& tracks() const;
  };
}  // namespace cyclops_ros
