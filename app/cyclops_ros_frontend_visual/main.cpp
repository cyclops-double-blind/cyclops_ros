#include "cyclops_ros/RawFeatureSet.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <chrono>

using Eigen::Matrix2f;
using Eigen::Vector2f;

struct Ellipse {
  double angle;
  double major_axis_size;
  double minor_axis_size;
};

static std::optional<Ellipse> makeErrorEllipse(Matrix2f const& information) {
  Eigen::SelfAdjointEigenSolver<Matrix2f> eigensolver(information.inverse());
  if (eigensolver.info() != Eigen::Success)
    return std::nullopt;

  auto const& lambda = eigensolver.eigenvalues();
  auto const& Q = eigensolver.eigenvectors();

  auto theta = std::atan2(Q(1, 0), Q(0, 0));
  auto lambda_max = lambda.y();
  auto lambda_min = lambda.x();

  return Ellipse {
    .angle = (theta < 0 ? theta + 2 * M_PI : theta) * 180 / M_PI,
    .major_axis_size = std::sqrt(std::min(lambda_min, 1e4f)),
    .minor_axis_size = std::sqrt(std::min(lambda_max, 1e4f)),
  };
}

static double safeExp(double z) {
  return std::exp(-std::max(0.0, std::min(20.0, z)));
}

static cv::Scalar determineEllipseColor(int track_age) {
  auto target_age = 90.0;

  auto s = 1. - std::clamp(track_age / target_age, 0., 1.);

  auto color_start = cv::Scalar(0.0, 0x55, 0xFF);
  auto color_end = cv::Scalar(0xFF, 0.0, 0.0);
  return color_start * s + color_end * (1 - s);
}

static std::optional<cv::Mat> makeTrackingImage(
  sensor_msgs::Image const& image, cyclops_ros::RawFeatureSet const& tracks) {
  auto cv_image = cv_bridge::toCvCopy(image, "rgb8");
  if (cv_image == nullptr)
    return std::nullopt;

  auto& result = cv_image->image;
  for (auto const& feature : tracks.features) {
    if (feature.age == 0)
      continue;

    auto information = Matrix2f(feature.weight.data());
    auto point = cv::Point2f(feature.u, feature.v);

    auto patch_size = cv::Point2f(tracks.patch_size, tracks.patch_size);
    auto rect_corner_lo = point - 0.5f * patch_size;
    auto rect_corner_hi = point + 0.5f * patch_size;
    auto rect_color = cv::Scalar(0x80, 0xFF, 0x00);
    cv::rectangle(result, rect_corner_lo, rect_corner_hi, rect_color, 1);

    if (!feature.weight.empty()) {
      auto maybe_ellipse = makeErrorEllipse(information);
      if (!maybe_ellipse)
        continue;

      auto [angle, major_axis, minor_axis] = *maybe_ellipse;
      auto ellipse_color = determineEllipseColor(feature.age);

      auto major_axis_clamp = std::max(2., major_axis);
      auto minor_axis_clamp = std::max(2., minor_axis);
      auto ellipse_axes = cv::Size(major_axis_clamp, minor_axis_clamp);
      cv::ellipse(result, point, ellipse_axes, angle, 0, 360, ellipse_color, 1);
    }
  }
  return result;
}

class CyclopsRosFrontendVisualizerContext {
private:
  image_transport::Publisher _publisher;

public:
  explicit CyclopsRosFrontendVisualizerContext(ros::NodeHandle& pnode) {
    auto it = image_transport::ImageTransport(pnode);
    _publisher = it.advertise("tracking_image", 1);
  }

  void handleMessage(
    sensor_msgs::ImageConstPtr const& image,
    cyclops_ros::RawFeatureSetConstPtr const& tracks);
};

void CyclopsRosFrontendVisualizerContext::handleMessage(
  sensor_msgs::ImageConstPtr const& image,
  cyclops_ros::RawFeatureSetConstPtr const& tracks) {
  auto tic = std::chrono::steady_clock::now();
  auto tracking_image = makeTrackingImage(*image, *tracks);
  if (!tracking_image.has_value()) {
    ROS_ERROR("Failed to convert sensor_msgs::Image to cv::Mat.");
    ROS_ERROR("Hint: check your image formatting.");
    return;
  }

  std::chrono::duration<double> dt1 = std::chrono::steady_clock::now() - tic;
  ROS_DEBUG_STREAM("Tracking image generation time: " << dt1.count() << " [s]");

  auto result_cv = cv_bridge::CvImage(image->header, "rgb8", *tracking_image);
  auto result = result_cv.toImageMsg();
  if (result == nullptr) {
    ROS_ERROR("Failed to convert cv::Mat to sensor_msgs::ImagePtr");
    return;
  }
  _publisher.publish(result);

  std::chrono::duration<double> dt2 = std::chrono::steady_clock::now() - tic;
  ROS_DEBUG_STREAM(
    "Tracking visualizer callback time: " << dt2.count() << " [s]");
}

static auto changeLogLevel(ros::NodeHandle& pnode) {
  auto log_level = std::min(
    pnode.param<int>("log_level", 1), (int)(ros::console::levels::Warn));
  ROS_INFO("Changing the log level to %d", log_level);
  if (!ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME, (ros::console::levels::Level)log_level)) {
    ROS_ERROR("Failed to change the log level");
    return false;
  }
  ros::console::notifyLoggerLevelsChanged();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cyclops_ros_frontend_visualizer");

  ros::NodeHandle node;
  ros::NodeHandle pnode("~");

  cv::setNumThreads(0);

  if (!changeLogLevel(pnode))
    return -1;
  auto image_topic_name = pnode.param<std::string>("image_topic_name", "image");
  auto context = CyclopsRosFrontendVisualizerContext(pnode);

  auto image_sub =
    message_filters::Subscriber<sensor_msgs::Image>(node, image_topic_name, 32);
  auto track_sub = message_filters::Subscriber<cyclops_ros::RawFeatureSet>(
    node, "cyclops_track_raw", 32);
  auto synchronizer = message_filters::TimeSynchronizer<
    sensor_msgs::Image, cyclops_ros::RawFeatureSet>(image_sub, track_sub, 32);

  synchronizer.registerCallback(
    &CyclopsRosFrontendVisualizerContext::handleMessage, &context);

  ROS_DEBUG("Ready to run. spinning...");
  ros::spin();

  return 0;
}
