#pragma once

#include <deque>
#include <memory>

namespace cyclops_ros {
  struct cyclops_ros_frontend_config_t;

  class CyclopsFeatureTrackUpdateThrottle {
  private:
    std::shared_ptr<cyclops_ros_frontend_config_t const> _config;

    double _dt_sum = 0;
    std::deque<double> _timestamps;

    void updateAveragingWindow();

  public:
    explicit CyclopsFeatureTrackUpdateThrottle(
      std::shared_ptr<cyclops_ros_frontend_config_t const> config);
    ~CyclopsFeatureTrackUpdateThrottle();

    bool update(double timestamp);
  };
}  // namespace cyclops_ros
