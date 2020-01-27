#include "cartographer/evaluation/environment_model.h"

#include <random>

namespace cartographer {
namespace evaluation {

namespace {
bool intersectPlane(const Eigen::Vector3f &plane_origin, const Eigen::Vector3f &plane_normal, const Eigen::Vector3f &ray_origin, const Eigen::Vector3f &ray_direction, float* intersection_distance)
{
  const float direction_ratio = dotProduct(plane_normal, ray_direction);
  if (direction_ratio > 1e-7) {
    const Eigen::Vector3f origin_to_origin = plane_origin - ray_origin;
    &intersection_distance = dotProduct(origin_to_origin, plane_normal) / direction_ratio;
    return true;
  }

  return false;
}

}
EnvironmentModel::EnvironmentModel() {
};

void EnvironmentModel::AddPlane(Eigen::Vector3f origin, Eigen::Vector3f normal) {
  planes_.emplace_back(orign, normal.normalized());
}

bool EnvironmentModel::checkRayIntersection(Eigen::Vector3f origin, Eigen::Vector3f direction, float* intersection_distance) {

  float min_dist = std::numeric_limits<float>::infinity();
  for(const auto& plane : planes_) {
    float plane_dist = 0.f;
    if(intersectPlane(plane.first, plane.second, origin, direction, plane_dist)) {
      if (plane_dist < min_dist) min_dist = plane_dist;
    }
  }

}


}  // namespace evaluation
}  // namespace cartographer

