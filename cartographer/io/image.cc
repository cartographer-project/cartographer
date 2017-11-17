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

void CheckStrideIsAsExpected(int width) {
  const int stride = cairo_format_stride_for_width(kCairoFormat, width);
  CHECK_EQ(stride, width * 4);
}

}  // namespace

UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t* surface) {
  return UniqueCairoSurfacePtr(surface, cairo_surface_destroy);
}

UniqueCairoPtr MakeUniqueCairoPtr(cairo_t* surface) {
  return UniqueCairoPtr(surface, cairo_destroy);
}

Image::Image(int width, int height)
    : width_(width), height_(height), pixels_(width * height, 0) {}

Image::Image(UniqueCairoSurfacePtr surface)
    : width_(cairo_image_surface_get_width(surface.get())),
      height_(cairo_image_surface_get_height(surface.get())) {
  CHECK_EQ(cairo_image_surface_get_format(surface.get()), kCairoFormat);
  CheckStrideIsAsExpected(width_);

  const uint32* pixel_data =
      reinterpret_cast<uint32*>(cairo_image_surface_get_data(surface.get()));
  const int num_pixels = width_ * height_;
  pixels_.reserve(num_pixels);
  for (int i = 0; i < num_pixels; ++i) {
    pixels_.push_back(pixel_data[i]);
  }
}

void Image::Rotate90DegreesClockwise() {
  const auto old_pixels = pixels_;
  pixels_.clear();
  for (int x = 0; x < width_; ++x) {
    for (int y = height_ - 1; y >= 0; --y) {
      pixels_.push_back(old_pixels.at(y * width_ + x));
    }
  }
  std::swap(width_, height_);
}

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
  return CairoToUint8Color(pixels_[y * width_ + x]);
}

void Image::SetPixel(int x, int y, const Uint8Color& color) {
  pixels_[y * width_ + x] = Uint8ColorToCairo(color);
}

UniqueCairoSurfacePtr Image::GetCairoSurface() {
  return MakeUniqueCairoSurfacePtr(cairo_image_surface_create_for_data(
      reinterpret_cast<unsigned char*>(pixels_.data()), kCairoFormat, width_,
      height_, width_ * 4 /* stride */));
}

}  // namespace io
}  // namespace cartographer
