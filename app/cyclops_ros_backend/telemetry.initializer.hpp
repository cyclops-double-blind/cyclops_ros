#pragma once

#include "cyclops/cyclops.hpp"
#include <ros/node_handle.h>

namespace cyclops {
  class InitializerTelemetryRos: public InitializerTelemetry {
  private:
    ros::Publisher _vision_failure;
    ros::Publisher _vision_sanity;
    ros::Publisher _vision_success;

    ros::Publisher _twoview_selection;
    ros::Publisher _twoview_hypothesis;
    ros::Publisher _twoview_success;

    ros::Publisher _imu_attempt;
    ros::Publisher _imu_ambiguity;
    ros::Publisher _imu_accept;
    ros::Publisher _imu_solution_reject;
    ros::Publisher _imu_candidate_reject;

    ros::Publisher _failure;
    ros::Publisher _success;
    ros::Publisher _success_detail;

  public:
    InitializerTelemetryRos(ros::NodeHandle& pnode);

    void onVisionFailure(VisionBootstrapFailure const& failure) override;
    void onBestTwoViewSelection(BestTwoViewSelection const& selection) override;
    void onTwoViewMotionHypothesis(
      TwoViewMotionHypothesis const& hypothesis) override;
    void onTwoViewSolverSuccess(TwoViewSolverSuccess const& success) override;

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
