#include "cartographer/io/image_points_processor.h"

#include <cmath>
#include <string>

#include "Eigen/Core"
#include "cairo/cairo.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/cairo_types.h"

namespace cartographer {
namespace io {
namespace {

struct PixelData {
  int count = 0;
  double mean_r = 0.;
  double mean_g = 0.;
  double mean_b = 0.;
};

using PixelDataMatrix =
  Eigen::Matrix<PixelData, Eigen::Dynamic, Eigen::Dynamic>;

double Mix(const double a, const double b, const double t) {
  return a * (1. - t) + t * b;
}

cairo_status_t CairoWriteCallback(void* const closure,
                                  const unsigned char* data,
                                  const unsigned int length) {
  if (static_cast<FileWriter*>(closure)->Write(
        reinterpret_cast<const char*>(data), length)) {
    return CAIRO_STATUS_SUCCESS;
  }
  return CAIRO_STATUS_WRITE_ERROR;
}

// Write 'mat' as a pleasing-to-look-at PNG into 'filename'
void WritePng(const PixelDataMatrix& mat, FileWriter* const file_writer) {
  const int stride =
    cairo_format_stride_for_width(CAIRO_FORMAT_ARGB32, mat.cols());
  CHECK_EQ(stride % 4, 0);
  std::vector<uint32_t> pixels(stride / 4 * mat.rows(), 0.);

  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.count == 0.) {
        pixels[y * stride / 4 + x] =
          (255 << 24) | (255 << 16) | (255 << 8) | 255;
        continue;
      }

      const int r = common::RoundToInt(cell.mean_r);
      const int g = common::RoundToInt(cell.mean_g);
      const int b = common::RoundToInt(cell.mean_b);
      pixels[y * stride / 4 + x] = (255 << 24) | (r << 16) | (g << 8) | b;
    }
  }

  // TODO(hrapp): cairo_image_surface_create_for_data does not take ownership of
  // the data until the surface is finalized. Once it is finalized though,
  // cairo_surface_write_to_png fails, complaining that the surface is already
  // finalized. This makes it pretty hard to pass back ownership of the image to
  // the caller.
  cairo::UniqueSurfacePtr surface(
    cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(pixels.data()), CAIRO_FORMAT_ARGB32,
      mat.cols(), mat.rows(), stride),
    cairo_surface_destroy);
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
  CHECK_EQ(cairo_surface_write_to_png_stream(surface.get(), &CairoWriteCallback,
                                             file_writer),
           CAIRO_STATUS_SUCCESS);
  CHECK(file_writer->Close());
}

bool ContainedIn(const common::Time& time,
                 const std::vector<mapping::Timespan>& timespans) {
  for (const mapping::Timespan& timespan : timespans) {
    if (timespan.start <= time && time <= timespan.end) {
      return true;
    }
  }
  return false;
}

static mapping_2d::ProbabilityGrid Create(
  double resolution) {
  double half_length = 200;
  const int num_cells_per_dimension =
    cartographer::common::RoundToInt(2. * half_length / resolution) + 1;
  Eigen::Vector2d max = half_length * Eigen::Vector2d::Ones();
  return mapping_2d::ProbabilityGrid(
    cartographer::mapping_2d::MapLimits(
      resolution, max,
      mapping_2d::CellLimits(num_cells_per_dimension, num_cells_per_dimension)));
}

}  // namespace

ImagePointsProcessor::ImagePointsProcessor(
  const double resolution, const string& output_filename,
  FileWriterFactory file_writer_factory, PointsProcessor* const next)
  : next_(next),
    file_writer_factory_(file_writer_factory),
    output_filename_(output_filename),
    aggregation_{Create(resolution), {}} {
}

std::unique_ptr<ImagePointsProcessor> ImagePointsProcessor::FromDictionary(
  FileWriterFactory file_writer_factory,
  common::LuaParameterDictionary* const dictionary,
  PointsProcessor* const next) {
  return common::make_unique<ImagePointsProcessor>(
    dictionary->GetDouble("resolution"),
    dictionary->GetString("filename"), file_writer_factory, next);
}

void ImagePointsProcessor::WriteGrid(const Aggregation& aggregation,
                                     FileWriter* const file_writer) {
  Eigen::Array2i offset;
  mapping_2d::CellLimits cell_limits;
  const auto & probability_grid = aggregation.probability_grid;
  probability_grid.ComputeCroppedLimits(&offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0)
  {
    LOG(WARNING) << "Not writing output: empty probability grid";
    return;
  }
  // probability grid: X: forward Y: left?
  // image: X: left Y: backward
  const auto grid_index_to_pixel = [cell_limits](const Eigen::Array2i& index) {
    // We flip the y axis, since matrices rows are counted from the top.
    return Eigen::Array2i(cell_limits.num_x_cells - index(0),
                          cell_limits.num_y_cells - index(1));
  };
  int width = cell_limits.num_y_cells;
  int height = cell_limits.num_x_cells;
  PixelDataMatrix image(width, height);
  for (auto xy_index :
         cartographer::mapping_2d::XYIndexRangeIterator(cell_limits)) {
    auto key = std::make_pair(xy_index.x(), xy_index.y());
    if (aggregation.grid_data.find(key) != aggregation.grid_data.end()) {
      const auto& grid_data = aggregation.grid_data.at(key);
      if (grid_data.count) {
        const Eigen::Array2i pixel = grid_index_to_pixel(xy_index);
        PixelData& pixel_data = image(pixel.y(), pixel.x());
        pixel_data.mean_r = grid_data.sum_r / grid_data.count;
        pixel_data.mean_g = grid_data.sum_g / grid_data.count;
        pixel_data.mean_b = grid_data.sum_b / grid_data.count;
        ++pixel_data.count;
      }
    }
  }

  WritePng(image, file_writer);
}

void ImagePointsProcessor::Insert(const PointsBatch& batch,
  Aggregation * const aggregation) {
  constexpr Color kDefaultColor = {{0, 0, 0}};
  for (size_t i = 0; i < batch.points.size(); ++i) {
    auto point = batch.points[i];
    auto cell_index = aggregation->probability_grid.limits().GetXYIndexOfCellContainingPoint(
      point(0), point(1));
    if (!aggregation->probability_grid.IsKnown(cell_index)) {
      aggregation->probability_grid.GrowLimits(point(0), point(1));
      aggregation->probability_grid.SetProbability(
          cell_index, cartographer::mapping::kMaxProbability);
      GridData& grid_data =
          aggregation->grid_data[std::make_pair(cell_index(0), cell_index(1))];
      const auto& color =
          batch.colors.empty() ? kDefaultColor : batch.colors.at(i);
      grid_data.sum_r += color[0];
      grid_data.sum_g += color[1];
      grid_data.sum_b += color[2];
      ++grid_data.count;
    }
  }
}

void ImagePointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  Insert(*batch, &aggregation_);
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult ImagePointsProcessor::Flush() {
  WriteGrid(aggregation_,
            file_writer_factory_(output_filename_ + ".png").get());
  switch (next_->Flush()) {
  case FlushResult::kRestartStream:
    LOG(FATAL) << "Image generation must be configured to occur after any "
      "stages that require multiple passes.";

  case FlushResult::kFinished:
    return FlushResult::kFinished;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer
