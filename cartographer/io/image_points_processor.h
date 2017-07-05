#ifndef CARTOGRAPHER_IO_IMAGE_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_IMAGE_POINTS_PROCESSOR_H_

#include <map>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

// Create image cuts through the points with pixels being 'pixel_size' big.
class ImagePointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_2d_image";
  ImagePointsProcessor(double pixel_size, const string& output_filename,
                      FileWriterFactory file_writer_factory,
                      PointsProcessor* next);

  static std::unique_ptr<ImagePointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ImagePointsProcessor() override {}

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

  Eigen::AlignedBox3i bounding_box() const { return bounding_box_; }

 private:
  struct GridData {
    double sum_r = 0.;
    double sum_g = 0.;
    double sum_b = 0.;
    uint32_t count = 0;
  };

  struct Aggregation {
    mapping_2d::ProbabilityGrid probability_grid;
    std::map<std::pair<int, int>, GridData> grid_data;
  };

  void WriteGrid(const Aggregation& aggregation,
                 FileWriter* const file_writer);
  void Insert(const PointsBatch& batch, Aggregation* aggregation);

  PointsProcessor* const next_;
  FileWriterFactory file_writer_factory_;

  const string output_filename_;

  Aggregation aggregation_;

  // Bounding box containing all cells with data in all 'aggregations_'.
  Eigen::AlignedBox3i bounding_box_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_IMAGE_POINTS_PROCESSOR_H_
