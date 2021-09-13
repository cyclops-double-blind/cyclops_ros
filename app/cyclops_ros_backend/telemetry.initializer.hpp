#pragma once

#include "cyclops/cyclops.hpp"
#include <ros/node_handle.h>

namespace cyclops {
  class InitializerTelemetryRos: public InitializerTelemetry {
  private:
    ros::Publisher _vision_failure_publisher;
    ros::Publisher _vision_solution_sanity_publisher;
    ros::Publisher _vision_success_publisher;

    ros::Publisher _attempt_publisher;
    ros::Publisher _ambiguity_publisher;
    ros::Publisher _accept_publisher;
    ros::Publisher _failure_publisher;
    ros::Publisher _success_publisher;
    ros::Publisher _success_detail_publisher;

    ros::Publisher _solution_reject_publisher;
    ros::Publisher _candidate_reject_publisher;

  public:
    InitializerTelemetryRos(ros::NodeHandle& pnode);

    void onVisionFailure(
      vision_initialization_failure_t const& failure) override;
    void onBundleAdjustmentSanity(
      bundle_adjustment_candidates_sanity_t const& sanity) override;
    void onBundleAdjustmentSuccess(
      bundle_adjustment_solution_t const& solution) override;

    void onIMUMatchAttempt(imu_match_attempt_t const& argument) override;
    void onIMUMatchAmbiguity(imu_match_ambiguity_t const& argument) override;

    void onIMUMatchAccept(imu_match_accept_t const& argument) override;
    void onIMUMatchReject(imu_match_reject_t const& argument) override;
    void onIMUMatchCandidateReject(imu_match_reject_t const& argument) override;

    void onFailure(onfailure_argument_t const& argument) override;
    void onSuccess(onsuccess_argument_t const& argument) override;
  };
}  // namespace cyclops
