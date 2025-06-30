#include "cyclops_ros_frontend/throttle.hpp"
#include "cyclops_ros_frontend/config.hpp"

#include <ros/console.h>

namespace cyclops_ros {
  CyclopsFeatureTrackUpdateThrottle::CyclopsFeatureTrackUpdateThrottle(
    std::shared_ptr<CyclopsFrontendConfig const> config)
      : _config(config) {
  }

  CyclopsFeatureTrackUpdateThrottle::~CyclopsFeatureTrackUpdateThrottle() =
    default;

  void CyclopsFeatureTrackUpdateThrottle::updateAveragingWindow() {
    auto const& tracker_config = _config->tracker_config;

    while (!_timestamps.empty()) {
      auto windowsize = _timestamps.back() - _timestamps.front();
      auto max_windowsize = tracker_config.track_update_fps_filter_window_size;
      if (windowsize <= max_windowsize)
        break;

      auto t1 = _timestamps.at(0);
      auto t2 = _timestamps.size() == 1 ? _timestamps.at(0) : _timestamps.at(1);
      auto dt_pop = t2 - t1;

      _dt_sum -= dt_pop;
      _timestamps.pop_front();
    }
  }

  bool CyclopsFeatureTrackUpdateThrottle::update(double timestamp) {
    if (_timestamps.empty()) {
      _timestamps.emplace_back(timestamp);
      return true;
    }

    auto dt = timestamp - _timestamps.back();
    auto dt_sum = _dt_sum + dt;
    auto dt_avg = dt_sum / _timestamps.size();
    auto dt_target = 1 / _config->tracker_config.track_update_fps_target;

    if (dt_avg > dt_target) {
      _dt_sum = dt_sum;
      _timestamps.emplace_back(timestamp);

      updateAveragingWindow();
      return true;
    }
    return false;
  }
}  // namespace cyclops_ros
