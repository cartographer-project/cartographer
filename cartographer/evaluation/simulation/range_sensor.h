#ifndef CARTOGRAPHER_EVALUATION_RANGE_SENSOR_H
#define CARTOGRAPHER_EVALUATION_RANGE_SENSOR_H

#include "cartographer/evaluation/simulation/scene.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace evaluation {

class RangeSensor {
 public:
  RangeSensor();
  RangeSensor(double fov_horizontal, double fov_vertical,
              double resolution_vertical, double resolution_horizontal,
              double tilt_angle, double rotation_frequency);

  sensor::TimedRangeData GenerateRangeData(double time,
                                           const transform::Rigid3d& pose,
                                           const Scene& scene) const;

 private:
  double fov_horizontal_;
  double fov_vertical_;
  double resolution_vertical_;
  double resolution_horizontal_;
  double tilt_angle_;
  double rotation_period_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_RANGE_SENSOR_H
