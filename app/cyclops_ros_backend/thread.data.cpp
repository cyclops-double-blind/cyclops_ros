#include "cyclops_ros_backend/thread.data.hpp"

namespace cyclops {
  using cyclops_ros::NormalizedFeatureSetConstPtr;

  using sensor_msgs::ImuConstPtr;
  using std_msgs::BoolConstPtr;

  static auto parseVector3Message(geometry_msgs::Vector3 const& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
  }

  DataThreadSpinner::DataThreadSpinner(
    std::shared_ptr<CyclopsMain> cyclops_main,
    std::shared_ptr<RosPublisherContext> publisher)
      : _cyclops_main(cyclops_main), _publisher(publisher) {
  }

  void DataThreadSpinner::bind(ros::NodeHandle& node, std::string imu_topic) {
    using This = DataThreadSpinner;

    _subscribers = {
      node.subscribe(imu_topic, 2048, &This::handleImu, this),
      node.subscribe("cyclops_features", 8, &This::handleFeatures, this),
      node.subscribe("reset", 8, &This::handleReset, this),
    };
  }

  void DataThreadSpinner::handleReset(BoolConstPtr const&) {
    _cyclops_main->enqueueResetRequest();
  }

  void DataThreadSpinner::handleImu(ImuConstPtr const& msg) {
    auto t = msg->header.stamp.toSec();
    auto a = parseVector3Message(msg->linear_acceleration);
    auto w = parseVector3Message(msg->angular_velocity);
    _cyclops_main->enqueueImuData({.timestamp = t, .accel = a, .rotat = w});

    auto propagation = _cyclops_main->propagation();
    if (propagation.has_value())
      _publisher->publishPropagation(*propagation);
  }

  void DataThreadSpinner::handleFeatures(
    NormalizedFeatureSetConstPtr const& msg) {
    auto data = ImageData {msg->header.stamp.toSec(), {}};
    for (auto const& feature : msg->features) {
      auto u = Eigen::Vector2d(feature.u, feature.v);
      auto H = Eigen::Matrix2f(feature.weight.data()).cast<double>().eval();
      data.features.emplace(feature.id, FeaturePoint {u, H});
    }
    _cyclops_main->enqueueLandmarkData(data);
  }
}  // namespace cyclops
