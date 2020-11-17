#include "cartographer/evaluation/simulation/scene.h"

namespace cartographer {
namespace evaluation {

Scene::Scene() {}
void Scene::AddSphere(const Eigen::Vector3f& origin, float radius) {
  spheres_.emplace_back(origin, radius);
}

void Scene::AddBox(const Eigen::Vector3f& origin, const Eigen::Vector3f& size) {
  boxes_.emplace_back(origin, size);
}
Eigen::Vector3f Scene::CollideWithScene(const Eigen::Vector3f& from,
                                        const Eigen::Vector3f& to,
                                        float& ratio) const {
  float ratio_spheres = 1.f;
  Eigen::Vector3f result_spheres = CollideWithSpheres(from, to, ratio_spheres);
  float ratio_boxes = 1.f;
  Eigen::Vector3f result_boxes = CollideWithBoxes(from, to, ratio_boxes);
  if (ratio_spheres < ratio_boxes) {
    ratio = ratio_spheres;
    return result_spheres;
  } else {
    ratio = ratio_boxes;
    return result_boxes;
  }
}
Eigen::Vector3f Scene::CollideWithSpheres(const Eigen::Vector3f& from,
                                          const Eigen::Vector3f& to,
                                          float& ratio) const {
  float first = 1.f;
  for (auto& sphere : spheres_) {
    const float a = (to - from).squaredNorm();
    const float beta = (to - from).dot(from - sphere.first);
    const float c =
        (from - sphere.first).squaredNorm() - sphere.second * sphere.second;
    const float discriminant = beta * beta - a * c;
    if (discriminant < 0.f) {
      continue;
    }
    const float solution = (-beta - std::sqrt(discriminant)) / a;
    if (solution < 0.f) {
      continue;
    }
    first = std::min(first, solution);
  }
  ratio = first;
  return first * (to - from) + from;
}
Eigen::Vector3f Scene::CollideWithBoxes(const Eigen::Vector3f& from,
                                        const Eigen::Vector3f& to,
                                        float& ratio) const {
  ratio = 1.f;
  for (auto& box : boxes_) {
    if (to.x() > from.x()) {
      ratio = std::min(ratio, (box.first.x() + box.second.x() - from.x()) /
                                  (to.x() - from.x()));
    } else if (to.x() < from.x()) {
      ratio = std::min(ratio, (box.first.x() - from.x()) / (to.x() - from.x()));
    }
    if (to.y() > from.y()) {
      ratio = std::min(ratio, (box.first.y() + box.second.y() - from.y()) /
                                  (to.y() - from.y()));
    } else if (to.y() < from.y()) {
      ratio = std::min(ratio, (box.first.y() - from.y()) / (to.y() - from.y()));
    }
    if (to.z() > from.z()) {
      ratio = std::min(ratio, (box.first.z() + box.second.z() - from.z()) /
                                  (to.z() - from.z()));
    } else if (to.z() < from.z()) {
      ratio = std::min(ratio, (box.first.z() - from.z()) / (to.z() - from.z()));
    }
  }
  return ratio * (to - from) + from;
}

}  // namespace evaluation
}  // namespace cartographer
