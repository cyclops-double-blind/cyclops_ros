cmake_minimum_required(VERSION 3.10)
project(cyclops_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs roscpp sensor_msgs std_msgs tf2_ros genmsg image_transport)

add_message_files(
  FILES
  IMUMatchAccept.msg
  IMUMatchAmbiguity.msg
  IMUMatchAttempt.msg
  IMUMatchReject.msg
  IMUMatchSolutionPoint.msg
  IMUMatchSolutionUncertainty.msg
  InitializationFailure.msg
  InitializationFailureIMUDigest.msg
  InitializationFailureVisionDigest.msg
  InitializationSuccess.msg
  Keyframe.msg
  KeyframeMotionState.msg
  KeyframeMotionStates.msg
  NormalizedFeature.msg
  NormalizedFeatureSet.msg
  OptimizationSanity.msg
  RawFeature.msg
  RawFeatureSet.msg
  VisionFailure.msg
  VisionSolutionCandidatesSanity.msg
  VisionSolutionSanity.msg
  VisionSuccess.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

add_subdirectory(src)
add_subdirectory(app)
