#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace cyclops_ros {
  struct CyclopsFrontendConfig;

  class DistortionModel {
  public:
    virtual ~DistortionModel() = default;

    virtual std::vector<cv::Point2f> undistort(
      std::vector<cv::Point2f> const& points) = 0;
    virtual cv::Mat evaluateJacobian(cv::Point2f const& point) = 0;

    static std::unique_ptr<DistortionModel> Create(
      std::shared_ptr<CyclopsFrontendConfig const> config);
  };
}  // namespace cyclops_ros
