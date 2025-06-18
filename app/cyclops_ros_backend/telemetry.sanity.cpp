#include "cyclops_ros_backend/telemetry.sanity.hpp"
#include <cyclops_ros/OptimizationSanity.h>

#include <ros/node_handle.h>
#include <std_msgs/Time.h>

namespace cyclops {
  using cyclops_ros::OptimizationSanity;

  OptimizerTelemetryRos::OptimizerTelemetryRos(ros::NodeHandle& pnode)
      : _sanity_statistics_publisher(
          pnode.advertise<OptimizationSanity>("sanity/statistics", 16)),
        _badness_publisher(
          pnode.advertise<std_msgs::Time>("sanity/badness", 16)),
        _failure_publisher(
          pnode.advertise<std_msgs::Time>("sanity/failure", 16)) {
  }

  void OptimizerTelemetryRos::onSanityStatistics(
    SanityStatistics const& statistics) {
    OptimizationSanity msg;
    msg.header.stamp = ros::Time::now();
    msg.potential_cost = statistics.final_cost;
    msg.significant_probability = statistics.final_cost_significant_probability;
    msg.landmark_accept_rate = statistics.landmark_accept_rate;
    msg.landmark_outlier_rate =
      statistics.landmark_chi_square_test_failure_rate;
    msg.landmark_observation_count = statistics.landmark_observations;
    _sanity_statistics_publisher.publish(msg);
  }

  void OptimizerTelemetryRos::onSanityBad(
    BadReason reason, SanityStatistics const& statistics) {
    OptimizerTelemetry::onSanityBad(reason, statistics);

    std_msgs::Time msg;
    msg.data = ros::Time::now();
    _badness_publisher.publish(msg);
  }

  void OptimizerTelemetryRos::onSanityFailure(
    FailureReason reason, SanityStatistics const& statistics) {
    OptimizerTelemetry::onSanityFailure(reason, statistics);

    ROS_ERROR_STREAM(
      "Final cost probability: "
      << 100 * statistics.final_cost_significant_probability << "%");
    ROS_ERROR_STREAM(
      "Landmark accept rate: " << statistics.landmark_accept_rate);

    std_msgs::Time msg;
    msg.data = ros::Time::now();
    _failure_publisher.publish(msg);
  }

  void OptimizerTelemetryRos::onUserResetRequest() {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    _failure_publisher.publish(msg);
  }
}  // namespace cyclops
