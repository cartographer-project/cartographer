#ifndef CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
#define CARTOGRAPHER_EVALUATION_GRID_DRAWER_H

#include <cartographer/mapping/3d/hybrid_grid_tsdf.h>
#include <cartographer/sensor/point_cloud.h>
#include "cartographer/sensor/range_data.h"

#include "cairo/cairo-svg.h"
#include "cairo/cairo.h"

namespace cartographer {
namespace evaluation {

class GridDrawer {
 public:
  GridDrawer();
  void DrawTSD(const cartographer::mapping::HybridGridTSDF& grid);
  void DrawWeights(const cartographer::mapping::HybridGridTSDF& grid);

  void DrawScan(const sensor::RangeData& range_data,
                const cartographer::transform::Rigid2d& initial_transform,
                const cartographer::transform::Rigid2d& matched_transform);

  void DrawPointcloud(
      const sensor::PointCloud& range_data,
      const cartographer::transform::Rigid2d& initial_transform,
      const cartographer::transform::Rigid2d& matched_transform);

  void ToFile(std::string filename);

 private:
  cairo_surface_t* grid_surface_;
  cairo_t* grid_surface_context_;
  double scale_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_GRID_DRAWER_H
