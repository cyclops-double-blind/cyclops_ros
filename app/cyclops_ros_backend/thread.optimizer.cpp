#include "cyclops_ros_backend/thread.optimizer.hpp"

namespace cyclops {
  OptimizationThreadSpinner::OptimizationThreadSpinner(
    std::shared_ptr<CyclopsMain> cyclops_main,
    std::shared_ptr<RosPublisherContext> publisher)
      : _cyclops_main(cyclops_main), _publisher(publisher) {
  }

  void OptimizationThreadSpinner::spin() {
    while (ros::ok()) {
      auto update = _cyclops_main->updateEstimation();
      if (update.reset) {
        _started = false;
        continue;
      }

      if (update.update_handles.empty())
        continue;

      if (!_started) {
        auto start_timestamp = update.update_handles.front().timestamp;

        _publisher->publishStart(start_timestamp);
        _started = true;
      }

      _publisher->publishKeyframeState(_cyclops_main->motions());
      _publisher->publishLandmarks(_cyclops_main->mappedLandmarks());
    }
    ROS_INFO("[cyclops_ros] Data consumer worker thread terminated");
  }

  std::thread OptimizationThreadSpinner::start() {
    return std::thread(&OptimizationThreadSpinner::spin, this);
  }
}  // namespace cyclops
