#include "cyclops_ros_frontend/distortion.hpp"
#include "cyclops_ros_frontend/config.hpp"

#include <opencv2/opencv.hpp>

namespace cyclops_ros {
  using std::vector;

  static cv::Mat makeCameraMatrix(CameraConfig const& config) {
    auto const& K = config.intrinsic;

    // clang-format off
    return (cv::Mat_<float>(3, 3) <<
      K.fx,   +0.,    K.cx,
      +0.,    K.fy,   K.cy,
      +0.,    +0.,    +1.
    );
    // clang-format on
  }

  class DistortionModelPinhole: public DistortionModel {
  private:
    std::shared_ptr<CyclopsFrontendConfig const> _config;
    CameraConfig::CameraDistortionPinhole const& _distortion;

    cv::Mat const _K;
    cv::Mat const _D;

    cv::Mat makeDistortionCoeffs() const;

  public:
    DistortionModelPinhole(
      std::shared_ptr<CyclopsFrontendConfig const> config,
      CameraConfig::CameraDistortionPinhole const& distortion);

    vector<cv::Point2f> undistort(vector<cv::Point2f> const& points) override;
    cv::Mat evaluateJacobian(cv::Point2f const& point) override;
  };

  cv::Mat DistortionModelPinhole::makeDistortionCoeffs() const {
    auto const& [k1, k2, p1, p2] = _distortion;
    return (cv::Mat_<float>(1, 4) << k1, k2, p1, p2);
  }

  DistortionModelPinhole::DistortionModelPinhole(
    std::shared_ptr<CyclopsFrontendConfig const> config,
    CameraConfig::CameraDistortionPinhole const& distortion)
      : _config(config),
        _distortion(distortion),
        _K(makeCameraMatrix(config->camera_config)),
        _D(makeDistortionCoeffs()) {
  }

  vector<cv::Point2f> DistortionModelPinhole::undistort(
    vector<cv::Point2f> const& points) {
    vector<cv::Point2f> result;
    cv::undistortPoints(points, result, _K, _D);
    return result;
  }

  cv::Mat DistortionModelPinhole::evaluateJacobian(cv::Point2f const& u) {
    auto u_mat = cv::Mat(u);

    auto const& [k1, k2, p1, p2] = _distortion;
    auto p = cv::Mat(cv::Point2f(p2, p1));

    auto u2 = u.dot(u);
    auto rho = 1 + k1 * u2 + k2 * u2 * u2;

    // clang-format off
    cv::Mat J_distortion =
      (2 * k1 + 4 * k2 * u2) * u_mat * u_mat.t() +
      (rho + 2 * u_mat.dot(p)) * cv::Mat::eye(2, 2, CV_32F) +
      2 * (p * u_mat.t() + u_mat * p.t());
    // clang-format on
    return _K(cv::Range(0, 2), cv::Range(0, 2)) * J_distortion;
  }

  class DistortionModelFisheye: public DistortionModel {
  private:
    std::shared_ptr<CyclopsFrontendConfig const> _config;
    CameraConfig::CameraDistortionFisheye const& _distortion;

    cv::Mat const _K;
    cv::Mat const _D;

    cv::Mat makeDistortionCoeffs() const;

  public:
    DistortionModelFisheye(
      std::shared_ptr<CyclopsFrontendConfig const> config,
      CameraConfig::CameraDistortionFisheye const& distortion);

    vector<cv::Point2f> undistort(vector<cv::Point2f> const& points) override;
    cv::Mat evaluateJacobian(cv::Point2f const& point) override;
  };

  cv::Mat DistortionModelFisheye::makeDistortionCoeffs() const {
    auto const& [k1, k2, k3, k4] = _distortion.coefficients;
    return (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);
  }

  DistortionModelFisheye::DistortionModelFisheye(
    std::shared_ptr<CyclopsFrontendConfig const> config,
    CameraConfig::CameraDistortionFisheye const& distortion)
      : _config(config),
        _distortion(distortion),
        _K(makeCameraMatrix(config->camera_config)),
        _D(makeDistortionCoeffs()) {
  }

  vector<cv::Point2f> DistortionModelFisheye::undistort(
    vector<cv::Point2f> const& points) {
    vector<cv::Point2f> result;
    cv::fisheye::undistortPoints(points, result, _K, _D);
    return result;
  }

  template <int N>
  static double evaluatePolynomial(
    std::array<double, N> const& coeffs, double x) {
    if (N <= 0)
      return 0;

    // Use Horner's method to evaluate polynomial.
    double r = 0;
    for (int i = N; i > 0; i--) {
      auto a = coeffs.at(i - 1);
      r = a + r * x;
    }

    return r;
  }

  cv::Mat DistortionModelFisheye::evaluateJacobian(cv::Point2f const& u) {
    auto const& [k1, k2, k3, k4] = _distortion.coefficients;

    auto r = cv::norm(u);
    auto u_mat = cv::Mat(u);

    auto theta = std::atan(r);
    auto theta2 = theta * theta;

    auto K = _K(cv::Range(0, 2), cv::Range(0, 2));
    auto I = cv::Mat::eye(2, 2, CV_32F);

    if (r <= 1e-3) {
      auto A = 1 + k1 * theta2;
      auto B = 2 * k1 - 2.0 / 3.0;

      return K * (A * I + B * u_mat * u_mat.t());
    }

    auto cos_theta = std::cos(theta);
    auto cos2_theta = cos_theta * cos_theta;

    auto theta_d = theta * evaluatePolynomial<5>({1, k1, k2, k3, k4}, theta2);
    auto theta_d_prime =
      evaluatePolynomial<5>({1, 3 * k1, 5 * k2, 7 * k3, 9 * k4}, theta2);

    auto n = u_mat / r;
    auto N = n * n.t();

    return K * ((I - N) / r * theta_d + N * theta_d_prime * cos2_theta);
  }

  template <class... Ts>
  struct overloaded: Ts... {
    using Ts::operator()...;
  };

  template <class... Ts>
  overloaded(Ts...) -> overloaded<Ts...>;

  std::unique_ptr<DistortionModel> DistortionModel::Create(
    std::shared_ptr<CyclopsFrontendConfig const> config) {
    auto visitor = overloaded {
      [&](CameraConfig::CameraDistortionPinhole const& pinhole)
        -> std::unique_ptr<DistortionModel> {
        return std::make_unique<DistortionModelPinhole>(config, pinhole);
      },
      [&](CameraConfig::CameraDistortionFisheye const& fisheye)
        -> std::unique_ptr<DistortionModel> {
        return std::make_unique<DistortionModelFisheye>(config, fisheye);
      },
    };

    return std::visit(visitor, config->camera_config.distortion);
  }
}  // namespace cyclops_ros
