#include "cartographer/evaluation/lidar_simulator.h"

#include <random>

namespace cartographer {
namespace evaluation {
namespace {
static std::default_random_engine e1(42);
}  // namespace

LidarSimulator::LidarSimulator()
    : max_range_(0.02f),
      depth_stdv_(0.01f),
      resolution_vertical_((0.1f / 180.f) * M_PI),
      resolution_horizontal_((0.4f / 180.f) * M_PI),
      vertical_range_(2.f * M_PI),
      horizontal_range_((30.f / 180.f) * M_PI) {}

      void LidarSimulator::
      generateScan(const EnvironmentModel& environment) {

}

}  // namespace evaluation
}  // namespace cartographer

//
//bool intersectPlane(const Vec3f &plane_normal, const Vec3f &plane_origin, const Vec3f &ray_origin, const Vec3f &ray_direction, float &t)
//{
//  float denom = dotProduct(plane_normal, ray_direction);
//  if (denom > 1e-6) {
//    Vec3f p0l0 = plane_origin - ray_origin;
//    t = dotProduct(p0l0, plane_normal) / denom;
//    return (t >= 0);
//  }
//
//  return false;
//}