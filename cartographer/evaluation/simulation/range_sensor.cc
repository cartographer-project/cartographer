#include "cartographer/evaluation/simulation/range_sensor.h"

namespace cartographer {
namespace evaluation {

RangeSensor::RangeSensor()
    : fov_horizontal_(2.0 * M_PI),
      fov_vertical_(2.0 * M_PI * 30.0 / 360.0),
      resolution_vertical_(2.0 * M_PI * 2.0 / 360.0),
      resolution_horizontal_(2.0 * M_PI * 0.2 / 360.0),
      tilt_angle_(2.0 * M_PI * 30.0 / 360.0),
      rotation_period_(1.0) {}

RangeSensor::RangeSensor(double fovHorizontal, double fovVertical,
                         double resolutionVertical,
                         double resolution_horizontal, double tilt_angle,
                         double rotation_frequency)
    : fov_horizontal_(fovHorizontal),
      fov_vertical_(fovVertical),
      resolution_vertical_(resolutionVertical),
      resolution_horizontal_(resolution_horizontal),
      tilt_angle_(tilt_angle),
      rotation_period_(rotation_frequency) {}

sensor::TimedRangeData RangeSensor::GenerateRangeData(
    double time, const transform::Rigid3d& pose, const Scene& scene) const {
  // 360 degree rays at 16 angles.
  sensor::TimedPointCloud directions_in_inner_rangefinder_frame;
  for (int r = -8; r != 8; ++r) {
    for (int s = -250; s != 250; ++s) {
      const sensor::TimedRangefinderPoint first_point{
          100.f * Eigen::Vector3f{Eigen::AngleAxisf(M_PI * s / 250.,
                                                    Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(M_PI / 12. * r / 8.,
                                                    Eigen::Vector3f::UnitY()) *
                                  Eigen::Vector3f::UnitX()},
          0.};
      directions_in_inner_rangefinder_frame.push_back(first_point);
      //      // Second orthogonal rangefinder.
      //      const sensor::TimedRangefinderPoint second_point{
      //          100.f *
      //              Eigen::Vector3f{
      //                  Eigen::AngleAxisf(M_PI / 2., Eigen::Vector3f::UnitX())
      //                  * Eigen::AngleAxisf(M_PI * s / 250.,
      //                  Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI
      //                  / 12. * r / 8.,
      //                                    Eigen::Vector3f::UnitY()) *
      //                  Eigen::Vector3f::UnitX()},
      //          0.};
      //      directions_in_inner_rangefinder_frame.push_back(second_point);
    }
  }

  sensor::TimedPointCloud returns_in_world_frame;
  Eigen::Quaterniond internal_rotation = transform::RollPitchYaw(
      0, tilt_angle_,
      2 * M_PI * std::fmod(time, rotation_period_) / rotation_period_);
  transform::Rigid3d internal_transform =
      transform::Rigid3d::Rotation(internal_rotation);
  sensor::TimedPointCloud directions_in_rangefinder_frame =
      sensor::TransformTimedPointCloud(directions_in_inner_rangefinder_frame,
                                       internal_transform.cast<float>());

  for (const auto& direction_in_world_frame : sensor::TransformTimedPointCloud(
           directions_in_rangefinder_frame, pose.cast<float>())) {
    const Eigen::Vector3f origin = pose.cast<float>() * Eigen::Vector3f::Zero();
    float ratio = 1.f;
    const sensor::TimedRangefinderPoint return_point{
        scene.CollideWithScene(origin, direction_in_world_frame.position,
                               ratio),
        0.};
    returns_in_world_frame.push_back(return_point);
  }
  return {Eigen::Vector3f::Zero(),
          sensor::TransformTimedPointCloud(returns_in_world_frame,
                                           pose.inverse().cast<float>()),
          {}};
  //  return sensor::TransformTimedRangeData(range_data_in_inner_frame,
  //  internal_transform.inverse().cast<float>());
}
}  // namespace evaluation
}  // namespace cartographer
