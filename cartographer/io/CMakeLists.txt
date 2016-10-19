google_library(io_cairo_types
  USES_CAIRO
  HDRS
    cairo_types.h
)

google_library(io_min_max_range_filtering_points_processor
  SRCS
    min_max_range_filtering_points_processor.cc
  HDRS
    min_max_range_filtering_points_processor.h
  DEPENDS
    io_points_batch
    io_points_processor
)

google_library(io_null_points_processor
  HDRS
    null_points_processor.h
  DEPENDS
    io_points_processor
)

google_library(io_ply_writing_points_processor
  USES_GLOG
  SRCS
    ply_writing_points_processor.cc
  HDRS
    ply_writing_points_processor.h
  DEPENDS
    io_points_processor
)

google_library(io_points_batch
  USES_EIGEN
  SRCS
    points_batch.cc
  HDRS
    points_batch.h
  DEPENDS
    common_time
)

google_library(io_points_processor
  HDRS
    points_processor.h
  DEPENDS
    io_points_batch
)

google_library(io_xray_points_processor
  USES_CAIRO
  USES_EIGEN
  SRCS
    xray_points_processor.cc
  HDRS
    xray_points_processor.h
  DEPENDS
    common_math
    io_cairo_types
    io_points_processor
    mapping_3d_hybrid_grid
    transform_rigid_transform
)

google_library(io_xyz_writing_points_processor
  USES_GLOG
  SRCS
    xyz_writing_points_processor.cc
  HDRS
    xyz_writing_points_processor.h
  DEPENDS
    io_points_processor
)
