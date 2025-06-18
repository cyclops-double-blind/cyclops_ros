#pragma once

#include "cyclops/cyclops.hpp"
#include <ros/node_handle.h>

namespace cyclops {
  class OptimizerTelemetryRos: public OptimizerTelemetry {
  private:
    ros::Publisher _sanity_statistics_publisher;
    ros::Publisher _badness_publisher;
    ros::Publisher _failure_publisher;

  public:
    explicit OptimizerTelemetryRos(ros::NodeHandle& pnode);

    void onSanityStatistics(SanityStatistics const& sanity) override;
    void onSanityBad(BadReason reason, SanityStatistics const& sanity) override;
    void onSanityFailure(
      FailureReason reason, SanityStatistics const& sanity) override;

    void onUserResetRequest() override;
  };
}  // namespace cyclops
