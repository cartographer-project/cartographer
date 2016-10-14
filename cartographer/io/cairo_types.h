#ifndef CARTOGRAPHER_IO_CAIRO_TYPES_H_
#define CARTOGRAPHER_IO_CAIRO_TYPES_H_

#include <memory>

#include "cairo/cairo.h"

namespace cartographer {
namespace io {
namespace cairo {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

// std::unique_ptr for Cairo contexts. The context is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueContextPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// std::unique_ptr for Cairo paths. The path is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniquePathPtr = std::unique_ptr<cairo_path_t, void (*)(cairo_path_t*)>;

}  // namespace cairo
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_CAIRO_TYPES_H_
