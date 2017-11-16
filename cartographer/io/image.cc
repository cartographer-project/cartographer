#include "cartographer/io/image.h"

#include <memory>

#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

uint32 Uint8ColorToCairo(const Uint8Color& color) {
  return static_cast<uint32>(255) << 24 | static_cast<uint32>(color[0]) << 16 |
         static_cast<uint32>(color[1]) << 8 | color[2];
}

Uint8Color CairoToUint8Color(uint32 color) {
  uint8 r = color >> 16;
  uint8 g = color >> 8;
  uint8 b = color;
  return {{r, g, b}};
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

const Uint8Color Image::GetPixel(int x, int y) const {
  return CairoToUint8Color(pixels_[y * stride_ / 4 + x]);
}

void Image::SetPixel(int x, int y, const Uint8Color& color) {
  pixels_[y * stride_ / 4 + x] = Uint8ColorToCairo(color);
}

UniqueCairoSurfacePtr Image::GetCairoSurface() {
  return MakeUniqueCairoSurfacePtr(cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(pixels_.data()), kCairoFormat, width_,
      height_, stride_));
}

}  // namespace io
}  // namespace cartographer
