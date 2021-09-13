#include "cyclops_ros_backend/config.hpp"

#include "cyclops/cyclops.hpp"
#include <ros/node_handle.h>

#define CYCLOPS_CONFIG_READ_FIELD(NS, CONFIG, FIELD) \
  { ::cyclops::config::read_field(NS, #FIELD, (CONFIG).FIELD); }

#define CYCLOPS_CONFIG_READ_OBJECT(NS, CONFIG, FIELD)         \
  {                                                           \
    auto FIELD_NS = ros::NodeHandle(NS, #FIELD);              \
    ::cyclops::config::read_object(FIELD_NS, (CONFIG).FIELD); \
  }

namespace cyclops::config::sensors {
  using ros::NodeHandle;
  using std::optional;

  template <typename value_t>
  struct failurable_t {
    bool failed;
    value_t& context;

    template <typename application_t>
    failurable_t<value_t> next(application_t&& app) {
      if (failed)
        return {.failed = true, .context = context};
      if (!app(context))
        return {.failed = true, .context = context};
      return *this;
    }
  };

  static optional<sensor_statistics_t> read_sensor_statistics_config(
    NodeHandle& pnode) {
    NodeHandle config_ns(pnode, "imu_noise");

    sensor_statistics_t config = {0, 0, 0, 0, 0, 0};
    auto failure = failurable_t<sensor_statistics_t> {
      .failed = false,
      .context = config,
    };

    failure
      .next([&config_ns](auto& config) {
        return config_ns.getParam("acc_white_noise", config.acc_white_noise);
      })
      .next([&config_ns](auto& config) {
        return config_ns.getParam("gyr_white_noise", config.gyr_white_noise);
      })
      .next([&config_ns](auto& config) {
        return config_ns.getParam("acc_random_walk", config.acc_random_walk);
      })
      .next([&config_ns](auto& config) {
        return config_ns.getParam("gyr_random_walk", config.gyr_random_walk);
      })
      .next([&config_ns](auto& config) {
        return config_ns.getParam(
          "acc_bias_prior_stddev", config.acc_bias_prior_stddev);
      })
      .next([&config_ns](auto& config) {
        return config_ns.getParam(
          "gyr_bias_prior_stddev", config.gyr_bias_prior_stddev);
      });

    if (failure.failed)
      return std::nullopt;
    return config;
  }

  static optional<sensor_extrinsics_t> read_sensor_extrinsics_config(
    NodeHandle& pnode) {
    NodeHandle config_ns(pnode, "extrinsic");
    NodeHandle transform_ns(config_ns, "imu_camera_transform");

    sensor_extrinsics_t config = {};
    auto failure = failurable_t<sensor_extrinsics_t> {
      .failed = false,
      .context = config,
    };

    failure
      .next([&config_ns](auto& config) {
        return config_ns.getParam(
          "imu_camera_time_delay", config.imu_camera_time_delay);
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "translation/x", config.imu_camera_transform.translation.x());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "translation/y", config.imu_camera_transform.translation.y());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "translation/z", config.imu_camera_transform.translation.z());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "rotation/w", config.imu_camera_transform.rotation.w());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "rotation/x", config.imu_camera_transform.rotation.x());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "rotation/y", config.imu_camera_transform.rotation.y());
      })
      .next([&transform_ns](auto& config) {
        return transform_ns.getParam(
          "rotation/z", config.imu_camera_transform.rotation.z());
      });

    if (failure.failed)
      return std::nullopt;
    return config;
  }
}  // namespace cyclops::config::sensors

namespace cyclops::config {
  using ros::NodeHandle;

  template <typename value_t>
  static void read_field(
    NodeHandle& ns, std::string const& field, value_t& value) {
    if (ns.getParam(field, value))
      return;

    ROS_DEBUG_STREAM("Failed to read field '" << ns.resolveName(field) << "'.");
    ROS_DEBUG_STREAM("Defaulting to: " << value);
  }

  static void read_object(
    NodeHandle& ns, measurement::keyframe_window_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, optimization_phase_max_keyframes);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, initialization_phase_max_keyframes);
  }

  static void read_object(
    NodeHandle& ns, measurement::image_update_throttling_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, update_rate_target);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, update_rate_smoothing_window_size);
  }

  static void read_object(
    NodeHandle& ns, initializer::vision::multiview_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, bundle_adjustment_max_iterations);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, bundle_adjustment_max_solver_time);
  }

  static void read_object(
    NodeHandle& ns, initializer::vision_solver_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, feature_point_isotropic_noise);
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, multiview);
  }

  static void read_object(
    NodeHandle& ns, initializer::imu::scale_sampling_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, sampling_domain_lowerbound);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, sampling_domain_upperbound);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, samples_count);
  }

  static void read_object(
    NodeHandle& ns, initializer::imu::solution_acceptance_threshold_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_scale_log_deviation);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_normalized_gravity_deviation);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_normalized_velocity_deviation);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_sfm_perturbation);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, translation_match_min_p_value);
  }

  static void read_object(
    NodeHandle& ns, initializer::imu_solver_config_t& config) {
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, sampling);
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, acceptance_test);
  }

  static void read_object(
    NodeHandle& ns, initializer::initialization_config_t& config) {
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, vision);
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, imu);
  }

  static void read_object(
    NodeHandle& ns, estimation::optimizer_config_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_num_iterations);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_solver_time_in_seconds);
  }

  static void read_object(
    NodeHandle& ns, estimation::fault_detection_threshold_t& config) {
    CYCLOPS_CONFIG_READ_FIELD(ns, config, min_landmark_accept_rate);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, min_final_cost_p_value);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_landmark_update_failures);
    CYCLOPS_CONFIG_READ_FIELD(ns, config, max_final_cost_sanity_failures);
  }

  static void read_object(
    NodeHandle& ns, estimation::estimator_config_t& config) {
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, optimizer);
    CYCLOPS_CONFIG_READ_OBJECT(ns, config, fault_detection);
  }

  static std::shared_ptr<cyclops_global_config_t const> read_core_config(
    NodeHandle& pnode) {
    auto maybe_noise = sensors::read_sensor_statistics_config(pnode);
    auto maybe_extrinsic = sensors::read_sensor_extrinsics_config(pnode);

    if (!maybe_noise) {
      ROS_ERROR("Failed to read sensor noise parameters");
      return nullptr;
    }

    if (!maybe_extrinsic) {
      ROS_ERROR("Failed to read sensor extrinsic parameters");
      return nullptr;
    }

    auto config =
      make_default_cyclops_global_config(*maybe_noise, *maybe_extrinsic);

    CYCLOPS_CONFIG_READ_OBJECT(pnode, *config, keyframe_window);
    CYCLOPS_CONFIG_READ_OBJECT(pnode, *config, update_throttling);
    CYCLOPS_CONFIG_READ_OBJECT(pnode, *config, initialization);
    CYCLOPS_CONFIG_READ_OBJECT(pnode, *config, estimation);
    return config;
  }
}  // namespace cyclops::config

namespace cyclops {
  std::unique_ptr<cyclops_ros_config_t const> read_config(
    ros::NodeHandle& pnode) {
    auto config = std::make_unique<cyclops_ros_config_t>();

    auto core_config = ::cyclops::config::read_core_config(pnode);
    if (core_config == nullptr)
      return nullptr;
    config->core_config = std::move(core_config);

    auto topic_io_ns = ros::NodeHandle(pnode, "topic_io");
    CYCLOPS_CONFIG_READ_FIELD(topic_io_ns, *config, map_frame_id);
    CYCLOPS_CONFIG_READ_FIELD(topic_io_ns, *config, imu_topic_name);

    return config;
  }
}  // namespace cyclops
