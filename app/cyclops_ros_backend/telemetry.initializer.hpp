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

    void onVisionFailure(VisionBootstrapFailure const& failure) override;
    void onBundleAdjustmentSanity(
      BundleAdjustmentCandidatesSanity const& sanity) override;
    void onBundleAdjustmentSuccess(
      BundleAdjustmentSolution const& solution) override;

    void onImuMatchAttempt(ImuMatchAttempt const& argument) override;
    void onImuMatchAmbiguity(ImuMatchAmbiguity const& argument) override;

    void onImuMatchAccept(ImuMatchAccept const& argument) override;
    void onImuMatchReject(ImuMatchReject const& argument) override;
    void onImuMatchCandidateReject(ImuMatchReject const& argument) override;

    void onFailure(OnFailure const& argument) override;
    void onSuccess(OnSuccess const& argument) override;
  };
}  // namespace cyclops
