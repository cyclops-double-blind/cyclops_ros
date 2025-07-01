#include "cyclops_ros_frontend/tracker.hpp"
#include "cyclops_ros_frontend/config.hpp"
#include "cyclops_ros_frontend/distortion.hpp"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <ros/console.h>
#include <ros/assert.h>

namespace cyclops_ros {
  using std::map;
  using std::vector;

  static std::tuple<vector<FeatureId>, vector<cv::Point2f>, vector<int>>
  flatten(map<FeatureId, FeatureTrack> const& tracks) {
    vector<FeatureId> flatten_ids;
    flatten_ids.reserve(tracks.size());

    vector<cv::Point2f> flatten_features;
    flatten_features.reserve(tracks.size());

    vector<int> flatten_track_counts;

    for (auto const& [id, track] : tracks) {
      flatten_ids.push_back(id);
      flatten_features.push_back(track.feature.point);
      flatten_track_counts.push_back(track.track_count);
    }
    return std::make_tuple(flatten_ids, flatten_features, flatten_track_counts);
  }

  template <typename value_t, typename validity_t>
  static void reduce(vector<value_t>& v, vector<validity_t> validity) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
      if (validity[i])
        v[j++] = v[i];
    v.resize(j);
  }

  enum class DerivativeDirection { X, Y };

  static cv::Mat evaluateNormalizedDerivative(
    cv::Mat const& image, DerivativeDirection direction) {
    cv::Mat diff_raw;
    switch (direction) {
    case decltype(direction)::X:
      cv::Scharr(image, diff_raw, CV_32F, 1, 0);
      break;
    case decltype(direction)::Y:
      cv::Scharr(image, diff_raw, CV_32F, 0, 1);
      break;
    }
    // Scharr kernel applies [3, 10, 3] gaussian convolution after a midpoint
    // differentiation. In order to preserve scale, the result needs to be
    // divided by 32 (2 by midpoint differentiation, 16 by 3 + 10 + 3).
    diff_raw /= 32;

    cv::Mat diff_blur;
    cv::blur(diff_raw, diff_blur, cv::Size(3, 3));

    return diff_blur;
  }

  static cv::Rect evaluateFeatureRoI(
    int imwidth, int imheight, cv::Point2f const& f0, cv::Size const& patch) {
    auto x0 = static_cast<int>(std::floor(f0.x));
    auto y0 = static_cast<int>(std::floor(f0.y));
    auto x1 = x0 + patch.width;
    auto y1 = y0 + patch.height;
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(imwidth, x1);
    y1 = std::min(imheight, y1);

    auto width = x1 - x0;
    auto height = y1 - y0;
    return cv::Rect(x0, y0, width, height);
  }

  static cv::Mat estimatePatchHessian(
    cv::Mat const& image_dx, cv::Mat const& image_dy,
    cv::Point2f const& feature_center, cv::Size const& patch) {
    ROS_ASSERT(image_dx.cols == image_dy.cols);
    ROS_ASSERT(image_dx.rows == image_dy.rows);

    auto f0 =
      feature_center - 0.5f * cv::Point2f(patch.width - 1, patch.height - 1);
    auto roi = evaluateFeatureRoI(image_dx.cols, image_dx.rows, f0, patch);

    cv::Mat H = cv::Mat::zeros(2, 2, CV_32F);
    for (int y = roi.y; y < roi.y + roi.height; y++) {
      for (int x = roi.x; x < roi.x + roi.width; x++) {
        auto dx_j = image_dx.at<float>(y, x);
        auto dy_j = image_dy.at<float>(y, x);
        H.at<float>(0, 0) += dx_j * dx_j;
        H.at<float>(0, 1) += dx_j * dy_j;
        H.at<float>(1, 0) += dy_j * dx_j;
        H.at<float>(1, 1) += dy_j * dy_j;
      }
    }
    return H;
  }

  bool CyclopsKltFeatureTracker::isFeatureInBorder(cv::Point2f const& pt) {
    int const margin = 1;
    int x = cvRound(pt.x);
    int y = cvRound(pt.y);

    auto contained = [&](auto x, auto min, auto max) {
      return x >= min && x < max;
    };
    auto x_min = 0 + margin;
    auto x_max = _config->camera_config.width - margin;
    auto y_min = 0 + margin;
    auto y_max = _config->camera_config.height - margin;
    auto x_in_border = contained(x, x_min, x_max);
    auto y_in_border = contained(y, y_min, y_max);

    return x_in_border && y_in_border;
  }

  vector<uint8_t> CyclopsKltFeatureTracker::testEpipolarGeometry(
    vector<cv::Point2f> const& prev_features,
    vector<cv::Point2f> const& curr_features) {
    if (curr_features.size() >= 8) {
      auto undistorted_prev = _distortion_model->undistort(prev_features);
      auto undistorted_curr = _distortion_model->undistort(curr_features);

      vector<uint8_t> status;
      cv::findFundamentalMat(
        undistorted_prev, undistorted_curr, cv::FM_RANSAC,
        _config->tracker_config.epipolar_ransac_pixel_noise,
        _config->tracker_config.epipolar_ransac_confidence_threshold, status);
      return status;
    }

    vector<uint8_t> status;
    status.reserve(curr_features.size());
    for (auto const& _ : curr_features)
      status.emplace_back(true);
    return status;
  }

  void CyclopsKltFeatureTracker::updateTrackMask(cv::Mat const& image) {
    _mask = cv::Mat(image.size(), CV_8UC1, cv::Scalar(255));

    vector<decltype(_tracks)::iterator> tracks;
    for (auto i = _tracks.begin(); i != _tracks.end(); i++)
      tracks.emplace_back(i);

    std::sort(tracks.begin(), tracks.end(), [](auto const& i, auto const& j) {
      auto const& [id_i, track_i] = *i;
      auto const& [id_j, track_j] = *i;
      return track_i.track_count > track_j.track_count;
    });

    decltype(_tracks) updated_tracks;
    for (auto const& i : tracks) {
      auto const& [id, track] = *i;
      if (_mask.at<uchar>(track.feature.point) == 255) {
        auto radius = _config->tracker_config.feature_min_distance;
        cv::circle(_mask, track.feature.point, radius, 0, -1);
        updated_tracks.emplace(id, track);
      }
    }
    _tracks = std::move(updated_tracks);
  }

  void CyclopsKltFeatureTracker::findNewTracks(cv::Mat const& image) {
    auto max = _config->tracker_config.max_features;
    auto current = static_cast<int>(_tracks.size());
    int new_features_count = max - current;
    if (new_features_count > 0) {
      ROS_DEBUG("Trying to find %d new features", new_features_count);

      vector<cv::Point2f> new_features;
      cv::goodFeaturesToTrack(
        image, new_features, new_features_count,
        _config->tracker_config.corner_detection_quality_threshold,
        _config->tracker_config.feature_min_distance, _mask);
      ROS_DEBUG_STREAM(
        "Successed to find " << new_features.size() << " new features");

      for (auto const& feature : new_features) {
        _last_feature_id++;
        _tracks.emplace(
          _last_feature_id, FeatureTrack {0, {feature, cv::Mat()}});
      }
    }
  }

  void CyclopsKltFeatureTracker::followTracks(cv::Mat const& raw_image) {
    cv::Mat image;
    auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(raw_image, image);

    if (_prev_image.empty()) {
      _prev_image = image;
      return;
    }

    if (_tracks.size() > 0) {
      auto [feature_ids, prev_features, track_counts] = flatten(_tracks);
      auto patch_size = cv::Size(
        _config->tracker_config.tracking_patch_size,
        _config->tracker_config.tracking_patch_size);

      vector<uint8_t> status;
      vector<float> errors;
      vector<cv::Point2f> curr_features;
      cv::calcOpticalFlowPyrLK(
        _prev_image, image, prev_features, curr_features, status, errors,
        patch_size, 3,
        cv::TermCriteria(
          cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        0, 1e-6);

      for (int i = 0; i < int(curr_features.size()); i++) {
        if (!isFeatureInBorder(curr_features[i]))
          status[i] = false;
      }

      auto success_rate = [](auto const& status) {
        int n = 0;
        for (auto flag : status) {
          if (flag)
            n++;
        }
        return static_cast<double>(n) / status.size();
      };

      ROS_INFO_STREAM("KLT tracking success rate: " << success_rate(status));

      reduce(feature_ids, status);
      reduce(track_counts, status);
      reduce(prev_features, status);
      reduce(curr_features, status);
      auto epipolar_validity =
        testEpipolarGeometry(prev_features, curr_features);

      ROS_INFO_STREAM(
        "Epipolar test pass rate: " << success_rate(epipolar_validity));

      reduce(feature_ids, epipolar_validity);
      reduce(track_counts, epipolar_validity);
      reduce(prev_features, epipolar_validity);
      reduce(curr_features, epipolar_validity);

      auto dx = evaluateNormalizedDerivative(image, DerivativeDirection::X);
      auto dy = evaluateNormalizedDerivative(image, DerivativeDirection::Y);

      _tracks.clear();
      for (int i = 0; i < int(curr_features.size()); i++) {
        auto id = feature_ids.at(i);
        auto count = track_counts.at(i);

        auto const& feature = curr_features.at(i);
        auto const& sigma = _config->tracker_config.image_noise_stddev;
        cv::Mat hessian = estimatePatchHessian(dx, dy, feature, patch_size);
        cv::Mat information = hessian / 2 / sigma / sigma;

        _tracks.emplace(id, FeatureTrack {count + 1, {feature, information}});
      }
    }

    _prev_image = image;
  }

  void CyclopsKltFeatureTracker::updateTracks() {
    updateTrackMask(_prev_image);
    findNewTracks(_prev_image);
  }

  map<FeatureId, FeaturePoint> CyclopsKltFeatureTracker::features() const {
    vector<FeatureId> ids;
    ids.reserve(_tracks.size());

    vector<cv::Point2f> features;
    features.reserve(_tracks.size());

    vector<cv::Mat> informations;
    informations.reserve(_tracks.size());

    for (auto const& [id, track] : _tracks) {
      if (track.track_count < 1)
        continue;
      ids.emplace_back(id);
      features.emplace_back(track.feature.point);
      informations.emplace_back(track.feature.information);
    }
    if (ids.empty())
      return {};

    auto undistorted_features = _distortion_model->undistort(features);

    map<FeatureId, FeaturePoint> result;
    for (size_t i = 0; i < undistorted_features.size(); i++) {
      auto const& u = undistorted_features.at(i);
      auto J = _distortion_model->evaluateJacobian(u);

      result.emplace(
        ids.at(i),
        FeaturePoint {
          .point = u,
          .information = J.t() * informations.at(i) * J,
        });
    }
    return result;
  }

  std::map<FeatureId, FeatureTrack> const& CyclopsKltFeatureTracker::tracks()
    const {
    return _tracks;
  }

  CyclopsKltFeatureTracker::CyclopsKltFeatureTracker(
    std::shared_ptr<CyclopsFrontendConfig const> config,
    std::unique_ptr<DistortionModel> distortion_model)
      : _config(std::move(config)),
        _distortion_model(std::move(distortion_model)) {
  }

  CyclopsKltFeatureTracker::~CyclopsKltFeatureTracker() = default;
}  // namespace cyclops_ros
