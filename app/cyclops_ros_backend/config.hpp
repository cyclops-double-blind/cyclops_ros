#pragma once

#include <ros/node_handle.h>
#include <memory>

namespace cyclops {
  struct cyclops_global_config_t;

  struct cyclops_ros_config_t {
    std::string imu_topic_name = "imu/data_raw";
    std::string map_frame_id = "map";
    std::shared_ptr<cyclops_global_config_t const> core_config;
  };

  std::unique_ptr<cyclops_ros_config_t const> read_config(
    ros::NodeHandle& pnode);
}  // namespace cyclops
