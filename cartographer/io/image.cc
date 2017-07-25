#include "cartographer/io/image.h"

#include <memory>

#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

cairo_status_t CairoWriteCallback(void* const closure,
                                  const unsigned char* data,
                                  const unsigned int length) {
  if (static_cast<FileWriter*>(closure)->Write(
          reinterpret_cast<const char*>(data), length)) {
    return CAIRO_STATUS_SUCCESS;
  }
  return CAIRO_STATUS_WRITE_ERROR;
}

constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

int StrideForWidth(int width) {
  const int stride = cairo_format_stride_for_width(kCairoFormat, width);
  CHECK_EQ(stride % 4, 0);
  return stride;
}

}  // namespace

UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t* surface) {
  return UniqueCairoSurfacePtr(surface, cairo_surface_destroy);
}

UniqueCairoPtr MakeUniqueCairoPtr(cairo_t* surface) {
  return UniqueCairoPtr(surface, cairo_destroy);
}

Image::Image(int width, int height)
    : width_(width),
      height_(height),
      stride_(StrideForWidth(width)),
      pixels_(stride_ / 4 * height, 0) {}

void Image::WritePng(FileWriter* const file_writer) {
  // TODO(hrapp): cairo_image_surface_create_for_data does not take ownership of
  // the data until the surface is finalized. Once it is finalized though,
  // cairo_surface_write_to_png fails, complaining that the surface is already
  // finalized. This makes it pretty hard to pass back ownership of the image to
  // the caller.
  UniqueCairoSurfacePtr surface = GetCairoSurface();
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
  CHECK_EQ(cairo_surface_write_to_png_stream(surface.get(), &CairoWriteCallback,
                                             file_writer),
           CAIRO_STATUS_SUCCESS);
}

const Color Image::GetPixel(int x, int y) const {
  const uint32_t value = pixels_[y * stride_ / 4 + x];
  return {{static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8),
           static_cast<uint8_t>(value)}};
}

void Image::SetPixel(int x, int y, const Color& color) {
  pixels_[y * stride_ / 4 + x] =
      (255 << 24) | (color[0] << 16) | (color[1] << 8) | color[2];
}

UniqueCairoSurfacePtr Image::GetCairoSurface() {
  return MakeUniqueCairoSurfacePtr(cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(pixels_.data()), kCairoFormat, width_,
      height_, stride_));
}

}  // namespace io
}  // namespace cartographer
