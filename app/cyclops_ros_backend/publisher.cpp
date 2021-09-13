#include "cyclops_ros_backend/publisher.hpp"
#include "cyclops_ros/KeyframeMotionStates.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Time.h>

namespace cyclops {
  using cyclops_ros::KeyframeMotionStates;

  using geometry_msgs::PoseStamped;
  using geometry_msgs::Vector3Stamped;
  using sensor_msgs::PointCloud;

  static auto make_pose_message(imu_motion_state_t const& pose) {
    geometry_msgs::Pose result;
    result.position.x = pose.position.x();
    result.position.y = pose.position.y();
    result.position.z = pose.position.z();

    result.orientation.w = pose.orientation.w();
    result.orientation.x = pose.orientation.x();
    result.orientation.y = pose.orientation.y();
    result.orientation.z = pose.orientation.z();

    return result;
  }

  static auto make_vector3_message(Eigen::Vector3d const& v) {
    geometry_msgs::Vector3 result;
    result.x = v.x();
    result.y = v.y();
    result.z = v.z();
    return result;
  }

  RosPublisherContext::RosPublisherContext(
    std::shared_ptr<cyclops_ros_config_t const> config)
      : _config(config) {
  }

  void RosPublisherContext::bind(ros::NodeHandle& pnode) {
    _start_publisher = pnode.advertise<std_msgs::Time>("start", 32);

    _propagation_pose_publisher =
      pnode.advertise<PoseStamped>("propagation/pose", 32);
    _propagation_velocity_publisher =
      pnode.advertise<Vector3Stamped>("propagation/velocity", 32);

    _keyframe_motions_publisher =
      pnode.advertise<KeyframeMotionStates>("keyframe_state", 32);
    _pointcloud_publisher = pnode.advertise<PointCloud>("pointcloud", 32);
  }

  void RosPublisherContext::publishPropagation(
    cyclops_propagation_state_t const& motion) {
    auto const& [timestamp, state] = motion;

    auto pose_msg = geometry_msgs::PoseStamped();
    pose_msg.header.stamp.fromSec(timestamp);
    pose_msg.header.frame_id = _config->map_frame_id;
    pose_msg.pose = make_pose_message(state);
    _propagation_pose_publisher.publish(pose_msg);

    auto velocity_msg = geometry_msgs::Vector3Stamped();
    velocity_msg.header.stamp.fromSec(timestamp);
    velocity_msg.header.frame_id = _config->map_frame_id;
    velocity_msg.vector = make_vector3_message(state.velocity);
    _propagation_velocity_publisher.publish(velocity_msg);
  }

  void RosPublisherContext::publishKeyframeState(
    std::map<frame_id_t, cyclops_keyframe_state_t> const& motions) {
    if (motions.empty())
      return;
    auto const& [_, last_keyframe] = *motions.rbegin();

    KeyframeMotionStates msg;
    msg.header.stamp.fromSec(last_keyframe.timestamp);
    msg.header.frame_id = _config->map_frame_id;

    for (auto const& [frame_id, motion] : motions) {
      msg.motions.emplace_back();
      msg.motions.back().frame_id = frame_id;
      msg.motions.back().timestamp.fromSec(motion.timestamp);
      msg.motions.back().pose = make_pose_message(motion.motion_state);
      msg.motions.back().velocity =
        make_vector3_message(motion.motion_state.velocity);
      msg.motions.back().accelerometer_bias =
        make_vector3_message(motion.acc_bias);
      msg.motions.back().gyrometer_bias = make_vector3_message(motion.gyr_bias);
    }
    _keyframe_motions_publisher.publish(msg);
  }

  void RosPublisherContext::publishLandmarks(
    landmark_positions_t const& landmarks) {
    auto msg = boost::make_shared<sensor_msgs::PointCloud>();
    msg->header.frame_id = _config->map_frame_id;
    msg->points.reserve(landmarks.size());

    msg->channels.emplace_back();
    msg->channels.back().name = "id";

    for (auto const& [id, f] : landmarks) {
      geometry_msgs::Point32 p;
      p.x = f.x();
      p.y = f.y();
      p.z = f.z();
      msg->points.push_back(p);
      msg->channels.back().values.push_back(id);
    }
    _pointcloud_publisher.publish(msg);
  }

  void RosPublisherContext::publishStart(timestamp_t timestamp) {
    std_msgs::Time msg;
    msg.data.fromSec(timestamp);
    _start_publisher.publish(msg);
  }
}  // namespace cyclops
