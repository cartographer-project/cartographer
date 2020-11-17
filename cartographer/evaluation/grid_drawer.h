#ifndef CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
#define CARTOGRAPHER_EVALUATION_GRID_DRAWER_H

#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"

#include "cairo/cairo-svg.h"
#include "cairo/cairo.h"

namespace cartographer {
namespace evaluation {

class GridDrawer {
 public:
  GridDrawer();
  GridDrawer(const cartographer::mapping::HybridGridTSDF& grid);
  void DrawTSD(const cartographer::mapping::HybridGridTSDF& grid, double z);

  void DrawInterpolatedTSD(const cartographer::mapping::HybridGridTSDF& grid,
                           double z);

  void DrawSinglePointCostFunction(
      const cartographer::mapping::HybridGridTSDF& grid,
      mapping::scan_matching::CeresScanMatcher3D& scan_matcher);

  //  void DrawWeights(const cartographer::mapping::HybridGridTSDF& grid);
  //
  //  void DrawScan(const sensor::RangeData& range_data,
  //                const cartographer::transform::Rigid2d& initial_transform,
  //                const cartographer::transform::Rigid2d& matched_transform);

  void DrawPointcloud(const sensor::PointCloud& range_data,
                      const cartographer::transform::Rigid3d& transform);

  void DrawPose(const cartographer::transform::Rigid3d& transform, float r, float g, float b);

  void ToFile(std::string filename);

 private:
  cairo_surface_t* grid_surface_;
  cairo_t* grid_surface_context_;
  double scale_;
  Eigen::Vector3d min_limits_;
  Eigen::Vector3d max_limits_;
  Eigen::Vector3i scaled_num_cells_;
  int axis0_;
  int axis1_;
  int axis2_;

  float max_tsd_;
  float resolution_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
