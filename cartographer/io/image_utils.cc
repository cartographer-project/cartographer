#include "cartographer/io/image_utils.h"

#include <memory>

#include "cairo/cairo.h"

#include "glog/logging.h"

#include "cartographer/io/file_writer.h"

namespace cartographer {
namespace io {
namespace {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

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

int stride_for_width(int width)
{
  const int stride = cairo_format_stride_for_width(kCairoFormat, width);
  CHECK_EQ(stride % 4, 0);
  return stride;
}

}  // namespace

Image CreateImage(int width, int height)
{
  int stride = stride_for_width(width);
  return { width, height, stride,
           std::vector<uint32_t>(stride / 4 * height, 0) };
}

void WritePng(Image * const image, FileWriter * const file_writer)
{
  // TODO(hrapp): cairo_image_surface_create_for_data does not take ownership of
  // the data until the surface is finalized. Once it is finalized though,
  // cairo_surface_write_to_png fails, complaining that the surface is already
  // finalized. This makes it pretty hard to pass back ownership of the image to
  // the caller.
  UniqueSurfacePtr surface(
    cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(image->pixels.data()), kCairoFormat,
      image->width, image->height, image->stride),
    cairo_surface_destroy);
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
  CHECK_EQ(cairo_surface_write_to_png_stream(surface.get(), &CairoWriteCallback,
                                             file_writer),
           CAIRO_STATUS_SUCCESS);
}

}  // namespace io
}  // namespace cartographer
