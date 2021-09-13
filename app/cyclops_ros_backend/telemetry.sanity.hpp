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

    void onSanityStatistics(sanity_statistics_t const&) override;

    void onSanityBad(bad_reason_t reason, sanity_statistics_t const&) override;
    void onSanityFailure(
      failure_reason_t reason, sanity_statistics_t const&) override;

    void onUserResetRequest() override;
  };
}  // namespace cyclops
