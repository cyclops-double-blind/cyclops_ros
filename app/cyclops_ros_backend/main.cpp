#include "cyclops_ros_backend/publisher.hpp"
#include "cyclops_ros_backend/thread.data.hpp"
#include "cyclops_ros_backend/thread.optimizer.hpp"

#include "cyclops_ros_backend/telemetry.initializer.hpp"
#include "cyclops_ros_backend/telemetry.keyframe.hpp"
#include "cyclops_ros_backend/telemetry.sanity.hpp"

#include "cyclops_ros_backend/config.hpp"

#include "cyclops/cyclops.hpp"

#include <ros/ros.h>

namespace cyclops {
  static bool initLogger(ros::NodeHandle& pnode) {
    auto log_level = pnode.param<int>("log_level", 2);  // INFO by default
    auto log_path = pnode.param<std::string>("log_path", "");

    ROS_INFO("Changing cyclops log level to %d", log_level);
    ROS_ERROR_STREAM("logger path: " << log_path);

    if (log_path.empty()) {
      ::cyclops::initLogger(log_level);
    } else {
      ROS_INFO_STREAM("Setting cyclops log path to " << log_path);
      ::cyclops::initLogger(log_path, log_level);
    }

    auto ros_log_level = std::max<int>(0, log_level - 1);
    ROS_INFO("changing rosconsole log level to %d", ros_log_level);
    if (!ros::console::set_logger_level(
          ROSCONSOLE_DEFAULT_NAME,
          (ros::console::levels::Level)ros_log_level)) {
      ROS_ERROR("failed to change log level");
      return false;
    }
    ros::console::notifyLoggerLevelsChanged();
    return true;
  }

  static int main(int argc, char** argv) {
    ros::init(argc, argv, "cyclops_ros_backend");
    ros::NodeHandle node;
    ros::NodeHandle pnode("~");

    if (!initLogger(pnode))
      return -1;

    std::shared_ptr config = readConfig(pnode);
    if (config == nullptr) {
      ROS_ERROR("failed to read cyclops configuration from rosparam");
      return -1;
    }

    ROS_INFO("successed to read parameters and set logging level.");
    ROS_DEBUG("n_a: %f", config->core_config->noise.acc_white_noise);
    ROS_DEBUG("n_w: %f", config->core_config->noise.gyr_white_noise);
    ROS_DEBUG("t_d: %f", config->core_config->extrinsics.imu_camera_time_delay);

    srand(20220208);

    std::shared_ptr cyclops_main = CyclopsMain::Create({
      .config = config->core_config,
      .seed = 20210914,
      .optimizer_telemetry = std::make_shared<OptimizerTelemetryRos>(pnode),
      .keyframe_telemetry = std::make_shared<KeyframeTelemetryRos>(pnode),
      .initializer_telemetry = std::make_shared<InitializerTelemetryRos>(pnode),
    });

    auto publisher = std::make_shared<RosPublisherContext>(config);
    auto data_spinner =
      std::make_shared<DataThreadSpinner>(cyclops_main, publisher);

    publisher->bind(pnode);
    data_spinner->bind(node, config->imu_topic_name);

    auto optimization_spinner =
      std::make_unique<OptimizationThreadSpinner>(cyclops_main, publisher);
    auto spin_thread = optimization_spinner->start();

    ros::spin();
    spin_thread.join();
    return 0;
  }
}  // namespace cyclops

int main(int argc, char** argv) {
  return cyclops::main(argc, argv);
}
