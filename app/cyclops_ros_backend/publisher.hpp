#pragma once

#include "cyclops_ros_backend/config.hpp"
#include "cyclops/cyclops.hpp"

#include <ros/ros.h>

namespace cyclops {
  class RosPublisherContext {
  private:
    std::shared_ptr<CyclopsRosConfig const> _config;

    ros::Publisher _start_publisher;
    ros::Publisher _propagation_pose_publisher;
    ros::Publisher _propagation_velocity_publisher;
    ros::Publisher _keyframe_motions_publisher;
    ros::Publisher _pointcloud_publisher;

  public:
    explicit RosPublisherContext(
      std::shared_ptr<CyclopsRosConfig const> config);
    void bind(ros::NodeHandle& pnode);

    // Invoked in the optimizer thread.
    void publishKeyframeState(std::map<FrameID, KeyframeState> const& motions);
    void publishLandmarks(LandmarkPositions const& landmarks);
    void publishStart(Timestamp timestamp);

    // Invoked in the data thread (by each IMU update callback).
    void publishPropagation(PropagationState const& motion);
  };
}  // namespace cyclops
