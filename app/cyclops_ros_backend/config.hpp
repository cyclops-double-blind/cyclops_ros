#pragma once

#include <ros/node_handle.h>
#include <memory>

namespace cyclops {
  struct CyclopsConfig;

  struct CyclopsRosConfig {
    std::string imu_topic_name = "imu/data_raw";
    std::string map_frame_id = "map";
    std::shared_ptr<CyclopsConfig const> core_config;
  };

  std::unique_ptr<CyclopsRosConfig const> readConfig(ros::NodeHandle& pnode);
}  // namespace cyclops
