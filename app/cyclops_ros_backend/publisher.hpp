#pragma once

#include "cyclops_ros_backend/config.hpp"
#include "cyclops/cyclops.hpp"

#include <ros/ros.h>

namespace cyclops {
  class RosPublisherContext {
  private:
    std::shared_ptr<cyclops_ros_config_t const> _config;

    ros::Publisher _start_publisher;
    ros::Publisher _propagation_pose_publisher;
    ros::Publisher _propagation_velocity_publisher;
    ros::Publisher _keyframe_motions_publisher;
    ros::Publisher _pointcloud_publisher;

  public:
    explicit RosPublisherContext(
      std::shared_ptr<cyclops_ros_config_t const> config);
    void bind(ros::NodeHandle& pnode);

    // Invoked in the optimizer thread.
    void publishKeyframeState(
      std::map<frame_id_t, cyclops_keyframe_state_t> const& motions);
    void publishLandmarks(landmark_positions_t const& landmarks);
    void publishStart(timestamp_t timestamp);

    // Invoked in the data thread (by each IMU update callback).
    void publishPropagation(cyclops_propagation_state_t const& motion);
  };
}  // namespace cyclops
