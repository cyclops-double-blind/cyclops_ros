#include "cyclops_ros_backend/telemetry.initializer.hpp"

#include <cyclops_ros/BestTwoViewSelection.h>
#include <cyclops_ros/IMUMatchAccept.h>
#include <cyclops_ros/IMUMatchAmbiguity.h>
#include <cyclops_ros/IMUMatchAttempt.h>
#include <cyclops_ros/IMUMatchReject.h>
#include <cyclops_ros/IMUMatchSolutionPoint.h>
#include <cyclops_ros/IMUMatchSolutionUncertainty.h>
#include <cyclops_ros/InitializationFailure.h>
#include <cyclops_ros/InitializationSuccess.h>
#include <cyclops_ros/TwoViewGeometryCandidate.h>
#include <cyclops_ros/TwoViewMotionHypothesis.h>
#include <cyclops_ros/TwoViewSolverSuccess.h>
#include <cyclops_ros/VisionFailure.h>
#include <cyclops_ros/VisionSolutionCandidatesSanity.h>
#include <cyclops_ros/VisionSuccess.h>

#include <ros/node_handle.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Time.h>

namespace cyclops {
  using MImuAccept = cyclops_ros::IMUMatchAccept;
  using MImuAmbiguity = cyclops_ros::IMUMatchAmbiguity;
  using MImuAttempt = cyclops_ros::IMUMatchAttempt;
  using MImuReject = cyclops_ros::IMUMatchReject;
  using MImuSolutionPoint = cyclops_ros::IMUMatchSolutionPoint;
  using MImuSolutionUncertainty = cyclops_ros::IMUMatchSolutionUncertainty;
  using MInitFailure = cyclops_ros::InitializationFailure;
  using MInitFailImuSummary = cyclops_ros::InitializationFailureIMUDigest;
  using MInitFailVisionSummary = cyclops_ros::InitializationFailureVisionDigest;
  using MInitSuccess = cyclops_ros::InitializationSuccess;
  using MTwoViewSelection = cyclops_ros::BestTwoViewSelection;
  using MTwoViewGeometry = cyclops_ros::TwoViewGeometryCandidate;
  using MTwoViewHypothesis = cyclops_ros::TwoViewMotionHypothesis;
  using MTwoViewSuccess = cyclops_ros::TwoViewSolverSuccess;
  using MVisionFailure = cyclops_ros::VisionFailure;
  using MVisionSanity = cyclops_ros::VisionSolutionCandidatesSanity;
  using MVisionSuccess = cyclops_ros::VisionSuccess;

  template <typename vector3_msg_t = geometry_msgs::Vector3>
  static auto makeVector3Msg(Eigen::Vector3d const& v) {
    vector3_msg_t msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
  }

  static auto makeQuaternionMsg(Eigen::Quaterniond const& q) {
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }

  static geometry_msgs::Pose makePoseMsg(SE3Transform const& x) {
    geometry_msgs::Pose msg;
    msg.position = makeVector3Msg<geometry_msgs::Point>(x.translation);
    msg.orientation = makeQuaternionMsg(x.rotation);
    return msg;
  }

  template <typename value_t, typename range_t>
  static auto flatten(range_t const& range) {
    return std::vector<value_t>(range.begin(), range.end());
  }

  void InitializerTelemetryRos::onVisionFailure(
    VisionBootstrapFailure const& failure) {
    MVisionFailure msg;
    msg.frame_id = flatten<int64_t>(failure.frames);

#define ASSIGN_FAILURE_REASON(name)         \
  case name: {                              \
    msg.reason_code = MVisionFailure::name; \
    msg.reason_readable = #name;            \
    break;                                  \
  }
    switch (failure.reason) {
      ASSIGN_FAILURE_REASON(NOT_ENOUGH_CONNECTED_IMAGE_FRAMES)
      ASSIGN_FAILURE_REASON(NOT_ENOUGH_MOTION_PARALLAX)
      ASSIGN_FAILURE_REASON(BEST_TWO_VIEW_SELECTION_FAILED)
      ASSIGN_FAILURE_REASON(TWO_VIEW_GEOMETRY_FAILED)
      ASSIGN_FAILURE_REASON(MULTI_VIEW_GEOMETRY_FAILED)
      ASSIGN_FAILURE_REASON(BUNDLE_ADJUSTMENT_FAILED)
    }
#undef ASSIGN_FAILURE_REASON
    ROS_INFO_STREAM("Vision bootstrap failed. Reason: " << msg.reason_readable);

    _vision_failure.publish(msg);
  }

  void InitializerTelemetryRos::onBestTwoViewSelection(
    BestTwoViewSelection const& selection) {
    MTwoViewSelection msg;
    msg.frames = flatten<int64_t>(selection.frames);
    msg.frame_id_1 = selection.frame_id_1;
    msg.frame_id_2 = selection.frame_id_2;
    _twoview_selection.publish(msg);
  }

  static auto asTelemetryMessage(
    InitializerTelemetry::TwoViewGeometry const& candidate) {
    MTwoViewGeometry msg;

#define COPY_FIELD(field) (msg.field = candidate.field)
    COPY_FIELD(acceptable);
    COPY_FIELD(rotation_prior_test_passed);
    COPY_FIELD(triangulation_test_passed);
    COPY_FIELD(rotation_prior_p_value);
    COPY_FIELD(triangulation_success_count);
#undef COPY_FIELD
    msg.motion = makePoseMsg(candidate.motion);
    return msg;
  }

  void InitializerTelemetryRos::onTwoViewMotionHypothesis(
    TwoViewMotionHypothesis const& hypothesis) {
    MTwoViewHypothesis msg;
    msg.frames = flatten<int64_t>(hypothesis.frames);
    msg.frame_id_1 = hypothesis.frame_id_1;
    msg.frame_id_2 = hypothesis.frame_id_2;

    for (auto const& candidate : hypothesis.candidates)
      msg.candidates.emplace_back(asTelemetryMessage(candidate));
    _twoview_hypothesis.publish(msg);
  }

  void InitializerTelemetryRos::onTwoViewSolverSuccess(
    TwoViewSolverSuccess const& success) {
    MTwoViewSuccess msg;
    msg.frames = flatten<int64_t>(success.frames);

#define COPY_FIELD(field) (msg.field = success.field)
    COPY_FIELD(initial_selected_model);
    COPY_FIELD(final_selected_model);
    COPY_FIELD(landmarks_count);
    COPY_FIELD(homography_expected_inliers);
    COPY_FIELD(epipolar_expected_inliers);
#undef COPY_FIELD

    for (auto const& candidate : success.candidates)
      msg.candidates.emplace_back(asTelemetryMessage(candidate));
    _twoview_success.publish(msg);
  }

  void InitializerTelemetryRos::onBundleAdjustmentSanity(
    BundleAdjustmentCandidatesSanity const& sanity) {
    MVisionSanity msg;
    msg.frame_id = flatten<int64_t>(sanity.frames);

    for (auto const& candidate_sanity : sanity.candidates_sanity) {
      msg.candidates_sanity.emplace_back();

      msg.candidates_sanity.back().acceptable = candidate_sanity.acceptable;
      msg.candidates_sanity.back().inlier_ratio = candidate_sanity.inlier_ratio;
      msg.candidates_sanity.back().final_cost_significant_probability =
        candidate_sanity.final_cost_significant_probability;
    }

    _vision_sanity.publish(msg);
  }

  template <typename vision_success_t>
  static auto makeVisionSuccessMessage(vision_success_t const& success) {
    auto result = MVisionSuccess();
    for (auto const& [frame_id, x] : success.camera_motions) {
      result.frame_id.emplace_back(frame_id);
      result.camera_motion.emplace_back(makePoseMsg(x));
    }

    for (auto const& [id, f] : success.landmarks) {
      geometry_msgs::Point32 p;
      p.x = f.x();
      p.y = f.y();
      p.z = f.z();
      result.landmarks.push_back(p);
    }

    return result;
  }

  void InitializerTelemetryRos::onBundleAdjustmentSuccess(
    BundleAdjustmentSolution const& solution) {
    _vision_success.publish(makeVisionSuccessMessage(solution));
  }

  static std_msgs::Float64MultiArray makeScaleCostLandscapeMessage(
    std::vector<std::tuple<double, double>> const& landscape) {
    std_msgs::Float64MultiArray msg;

    msg.layout.dim.resize(2);
    msg.layout.dim.at(0).label = "";
    msg.layout.dim.at(0).size = landscape.size();
    msg.layout.dim.at(0).stride = 2 * landscape.size();
    msg.layout.dim.at(1).label = "";
    msg.layout.dim.at(1).size = 2;
    msg.layout.dim.at(1).stride = 2;

    msg.layout.data_offset = 0;

    msg.data.resize(2 * landscape.size());
    for (int i = 0; i < landscape.size(); i++) {
      auto const& [scale, cost] = landscape.at(i);
      msg.data[2 * i] = scale;
      msg.data[2 * i + 1] = cost;
    }
    return msg;
  }

  void InitializerTelemetryRos::onImuMatchAttempt(
    ImuMatchAttempt const& argument) {
    ROS_INFO("IMU match attempt");

    auto msg = boost::make_shared<MImuAttempt>();
    msg->degrees_of_freedom = argument.degrees_of_freedom;
    msg->frame_id = flatten<int64_t>(argument.frames);
    msg->cost_landscape = makeScaleCostLandscapeMessage(argument.landscape);
    msg->local_minima = makeScaleCostLandscapeMessage(argument.minima);
    _imu_attempt.publish(msg);
  }

  template <typename solution_point_t>
  static auto makeSolutionPointMessage(solution_point_t const& solution) {
    auto result = MImuSolutionPoint();
    result.scale = solution.scale;
    result.cost = solution.cost;

    result.gravity = makeVector3Msg(solution.gravity);
    result.bias_gyr = makeVector3Msg(solution.gyr_bias);
    result.bias_acc = makeVector3Msg(solution.acc_bias);

    for (auto const& [_, v] : solution.imu_body_velocities)
      result.imu_body_velocity.emplace_back(makeVector3Msg(v));

    for (auto const& [frame_id, p_c] : solution.sfm_positions) {
      auto const& q_b = solution.imu_orientations.at(frame_id);

      result.frame_id.emplace_back(frame_id);
      result.imu_orientation.emplace_back(makeQuaternionMsg(q_b));
      result.sfm_camera_position.emplace_back(makeVector3Msg(p_c));
    }
    return result;
  }

  template <typename solution_uncertainty_t>
  static auto makeSolutionUncertaintyMessage(
    solution_uncertainty_t const& uncertainty) {
    MImuSolutionUncertainty result;
    result.valid = true;

    result.final_cost_significant_probability =
      uncertainty.final_cost_significant_probability;
    result.scale_log_deviation = uncertainty.scale_log_deviation;
    result.gravity_max_deviation = uncertainty.gravity_max_deviation;
    result.bias_max_deviation = uncertainty.bias_max_deviation;
    result.body_velocity_max_deviation =
      uncertainty.body_velocity_max_deviation;
    result.scale_symmetric_translation_error_max_deviation =
      uncertainty.scale_symmetric_translation_error_max_deviation;

    return result;
  }

  template <typename solution_uncertainty_t>
  static auto makeSolutionUncertaintyMessage(
    std::optional<solution_uncertainty_t> const& maybe_uncertainty) {
    MImuSolutionUncertainty result;
    if (!maybe_uncertainty.has_value()) {
      result.valid = false;
      return result;
    }
    return makeSolutionUncertaintyMessage(*maybe_uncertainty);
  }

  void InitializerTelemetryRos::onImuMatchAmbiguity(
    ImuMatchAmbiguity const& argument) {
    auto msg = boost::make_shared<MImuAmbiguity>();

    for (auto const& solution : argument.solutions)
      msg->solution.emplace_back(makeSolutionPointMessage(solution));

    for (auto const& uncertainty : argument.uncertainties) {
      msg->uncertainty.emplace_back(
        makeSolutionUncertaintyMessage(uncertainty));
    }

    _imu_ambiguity.publish(msg);

    ROS_INFO_STREAM("IMU match ambiguity. Solutions:");
    for (auto const& solution : argument.solutions)
      ROS_INFO_STREAM("  s = " << solution.scale);

    ROS_INFO_STREAM("Uncertainties:");
    for (auto const& uncertainty : msg->uncertainty)
      ROS_INFO_STREAM("  " << uncertainty);
  }

  template <typename reject_reason_t>
  static auto rejectToString(reject_reason_t reason) {
    switch (reason) {
    case reject_reason_t::UNCERTAINTY_EVALUATION_FAILED:
      return "UNCERTAINTY_EVALUATION_FAILED";
    case reject_reason_t::COST_PROBABILITY_INSIGNIFICANT:
      return "COST_PROBABILITY_INSIGNIFICANT";
    case reject_reason_t::UNDERINFORMATIVE_PARAMETER:
      return "UNDERINFORMATIVE_PARAMETER";
    default:
      return "Unspecified";
    }
    return "Unspecified";
  }

  template <typename reject_reason_t>
  static auto makeRejectReasonMessage(reject_reason_t reason) {
    switch (reason) {
    case reject_reason_t::UNCERTAINTY_EVALUATION_FAILED:
      return MImuReject::REJECT_REASON_UNCERTAINTY_EVALUATION_FAILED;
    case reject_reason_t::COST_PROBABILITY_INSIGNIFICANT:
      return MImuReject::REJECT_REASON_COST_PROBABILITY_INSIGNIFICANT;
    case reject_reason_t::UNDERINFORMATIVE_PARAMETER:
      return MImuReject::REJECT_REASON_PARAMETER_UNDERINFORMATIVE;
    default:
      return MImuReject::REJECT_REASON_UNSPECIFIED;
    }
    return MImuReject::REJECT_REASON_UNSPECIFIED;
  }

  void InitializerTelemetryRos::onImuMatchAccept(
    ImuMatchAccept const& argument) {
    MImuAccept msg;
    msg.solution = makeSolutionPointMessage(argument.solution);
    msg.uncertainty = makeSolutionUncertaintyMessage(argument.uncertainty);

    _imu_accept.publish(msg);
  }

  void InitializerTelemetryRos::onImuMatchReject(
    ImuMatchReject const& argument) {
    MImuReject msg;
    msg.solution = makeSolutionPointMessage(argument.solution);
    msg.uncertainty = makeSolutionUncertaintyMessage(argument.uncertainty);
    msg.reject_reason = makeRejectReasonMessage(argument.reason);

    _imu_solution_reject.publish(msg);

    ROS_INFO_STREAM("IMU match rejected: " << rejectToString(argument.reason));
  }

  void InitializerTelemetryRos::onImuMatchCandidateReject(
    ImuMatchReject const& argument) {
    MImuReject msg;
    msg.solution = makeSolutionPointMessage(argument.solution);
    msg.uncertainty = makeSolutionUncertaintyMessage(argument.uncertainty);
    msg.reject_reason = makeRejectReasonMessage(argument.reason);

    _imu_candidate_reject.publish(msg);

    ROS_INFO_STREAM(
      "IMU match candidate rejected: " << rejectToString(argument.reason));
    ROS_INFO_STREAM("Scale: " << msg.solution.scale);
    ROS_INFO_STREAM("Uncertainty: " << msg.uncertainty);
  }

  void InitializerTelemetryRos::onFailure(OnFailure const& argument) {
    MInitFailure msg;

    for (auto const& vision_digest : argument.vision_solutions) {
      MInitFailVisionSummary digest;
      digest.acceptable = vision_digest.acceptable;
      for (auto frame_id : vision_digest.keyframes)
        digest.keyframes.push_back(frame_id);
      msg.vision_solutions.push_back(digest);
    }

    for (auto const& imu_digest : argument.imu_solutions) {
      MInitFailImuSummary digest;
      digest.acceptable = imu_digest.acceptable;
      digest.vision_solution_index = imu_digest.vision_solution_index;
      digest.scale = imu_digest.scale;

      for (auto frame_id : imu_digest.keyframes)
        digest.keyframes.push_back(frame_id);
      msg.imu_solutions.push_back(digest);
    }

#define REPORT(REASON)                                                 \
  {                                                                    \
    msg.failure_reason = MInitFailure::REASON;                         \
    msg.failure_reason_readable = #REASON;                             \
    ROS_INFO_STREAM("IMU initialization failed. Reason: " << #REASON); \
    _failure.publish(msg);                                             \
    return;                                                            \
  }

    if (argument.vision_solutions.empty())
      REPORT(VISION_INITIALIZATION_FAILED);

    if (argument.imu_solutions.empty())
      REPORT(NO_IMU_MATCH_CANDIDATE);

    if (argument.imu_solutions.size() > 1)
      REPORT(AMBIGUOUS_IMU_MATCH);

    auto const& imu_solution = argument.imu_solutions.front();
    auto const& vision_solution =
      argument.vision_solutions.at(imu_solution.vision_solution_index);

    if (!vision_solution.acceptable)
      REPORT(UNACCEPTABLE_VISION_SOLUTION);

    if (!imu_solution.acceptable)
      REPORT(UNACCEPTABLE_IMU_SOLUTION);

    REPORT(UNKNOWN);
  }

  void InitializerTelemetryRos::onSuccess(OnSuccess const& success) {
    auto timestamp = success.initial_motion_frame_timestamp;
    auto success_msg = boost::make_shared<std_msgs::Time>();
    success_msg->data.fromSec(timestamp);
    _success.publish(success_msg);

    auto detail_msg = boost::make_shared<MInitSuccess>();
    detail_msg->vision_solution_index = success.vision_solution_index;
    detail_msg->initial_keyframe_id = success.initial_motion_frame_id;
    detail_msg->initial_keyframe_timestamp.fromSec(timestamp);
    detail_msg->scale = success.scale;
    detail_msg->cost = success.cost;
    detail_msg->gravity = makeVector3Msg(success.gravity);

    for (auto const& [frame_id, motion] : success.motions) {
      detail_msg->frame_id.emplace_back(frame_id);
      detail_msg->imu_position.emplace_back(makeVector3Msg(motion.position));
      detail_msg->imu_velocity.emplace_back(makeVector3Msg(motion.velocity));
      detail_msg->imu_orientation.emplace_back(
        makeQuaternionMsg(motion.orientation));
    }
    for (auto const& [_, camera_pose] : success.sfm_camera_pose)
      detail_msg->sfm_camera_pose.emplace_back(makePoseMsg(camera_pose));
    _success_detail.publish(detail_msg);
  }

  class AdvertisementHelper {
  private:
    ros::NodeHandle& _pnode;

  public:
    explicit AdvertisementHelper(ros::NodeHandle& pnode): _pnode(pnode) {
    }

    template <typename message_t>
    ros::Publisher make(std::string topic) {
      return _pnode.advertise<message_t>(topic, 16);
    }
  };

  InitializerTelemetryRos::InitializerTelemetryRos(ros::NodeHandle& pnode) {
    auto _ = AdvertisementHelper(pnode);
    _vision_failure = _.make<MVisionFailure>("init/vision/failure");
    _vision_success = _.make<MVisionSuccess>("init/vision/success");
    _vision_sanity = _.make<MVisionSanity>("init/vision/sanity");
    _twoview_selection = _.make<MTwoViewSelection>("init/twoview/selection");
    _twoview_hypothesis = _.make<MTwoViewHypothesis>("init/twoview/hypothesis");
    _twoview_success = _.make<MTwoViewSuccess>("init/twoview/success");

    _imu_attempt = _.make<MImuAttempt>("init/imu/attempt");
    _imu_ambiguity = _.make<MImuAmbiguity>("init/imu/ambiguity");
    _imu_accept = _.make<MImuAccept>("init/imu/accept");
    _imu_solution_reject = _.make<MImuReject>("init/imu/solution_reject");
    _imu_candidate_reject = _.make<MImuReject>("init/imu/candidate_reject");

    _failure = _.make<MInitFailure>("init/failure");
    _success = _.make<std_msgs::Time>("init/success");
    _success_detail = _.make<MInitSuccess>("init/success/detail");
  }
}  // namespace cyclops
