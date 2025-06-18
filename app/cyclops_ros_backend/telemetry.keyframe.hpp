#pragma once

#include "cyclops/cyclops.hpp"
#include <ros/node_handle.h>

namespace cyclops {
  class KeyframeTelemetryRos: public KeyframeTelemetry {
  private:
    ros::Publisher _publisher;

  public:
    explicit KeyframeTelemetryRos(ros::NodeHandle& pnode);
    ~KeyframeTelemetryRos();

    void onNewMotionFrame(OnNewMotionFrame const& argument) override;
  };
}  // namespace cyclops
