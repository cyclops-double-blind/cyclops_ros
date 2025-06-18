#include "cyclops_ros_backend/telemetry.keyframe.hpp"
#include <cyclops_ros/Keyframe.h>

#include <ros/node_handle.h>

namespace cyclops {
  using cyclops_ros::Keyframe;

  KeyframeTelemetryRos::KeyframeTelemetryRos(ros::NodeHandle& pnode) {
    _publisher = pnode.advertise<Keyframe>("keyframe", 1024);
  }

  KeyframeTelemetryRos::~KeyframeTelemetryRos() = default;

  void KeyframeTelemetryRos::onNewMotionFrame(
    OnNewMotionFrame const& argument) {
    auto m = Keyframe();
    m.frame_id = argument.frame_id;
    m.timestamp.fromSec(argument.timestamp);

    _publisher.publish(m);
  }
}  // namespace cyclops
