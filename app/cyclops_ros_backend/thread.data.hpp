#pragma once

#include "cyclops_ros_backend/publisher.hpp"
#include "cyclops_ros/NormalizedFeatureSet.h"

#include "cyclops/cyclops.hpp"

#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

namespace cyclops {
  class DataThreadSpinner {
  private:
    std::shared_ptr<CyclopsMain> _cyclops_main;
    std::shared_ptr<RosPublisherContext> _publisher;

    std::vector<ros::Subscriber> _subscribers;

    void handleReset(std_msgs::BoolConstPtr const& msg);
    void handleImu(sensor_msgs::ImuConstPtr const& imu);
    void handleFeatures(
      cyclops_ros::NormalizedFeatureSetConstPtr const& features);

  public:
    DataThreadSpinner(
      std::shared_ptr<CyclopsMain> cyclops_main,
      std::shared_ptr<RosPublisherContext> publisher);
    void bind(ros::NodeHandle& node, std::string imu_topic);
  };
}  // namespace cyclops
