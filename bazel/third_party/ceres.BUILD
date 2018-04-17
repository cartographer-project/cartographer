# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
#   Ceres Solver is an open source C++ library for modeling and solving large,
#   complicated optimization problems.

licenses(["notice"])  # New BSD, portions MIT.

CERES_DEFINES = [
    "CERES_USE_CXX11",
    "CERES_USE_EIGEN_SPARSE",
    "CERES_NO_SUITESPARSE",
    "CERES_NO_LAPACK",
    "CERES_NO_CXSPARSE",
    "CERES_STD_UNORDERED_MAP",
    "CERES_USE_CXX11_THREADS",

    # Use the internal mutex code. Not ideal, but it works.
    "CERES_HAVE_PTHREAD",
    "CERES_HAVE_RWLOCK",
]

cc_library(
    name = "ceres",
    srcs = [
        "internal/ceres/array_utils.cc",
        "internal/ceres/blas.cc",
        "internal/ceres/block_evaluate_preparer.cc",
        "internal/ceres/block_jacobi_preconditioner.cc",
        "internal/ceres/block_jacobian_writer.cc",
        "internal/ceres/block_random_access_dense_matrix.cc",
        "internal/ceres/block_random_access_diagonal_matrix.cc",
        "internal/ceres/block_random_access_matrix.cc",
        "internal/ceres/block_random_access_sparse_matrix.cc",
        "internal/ceres/block_sparse_matrix.cc",
        "internal/ceres/block_structure.cc",
        "internal/ceres/callbacks.cc",
        "internal/ceres/c_api.cc",
        "internal/ceres/canonical_views_clustering.cc",
        "internal/ceres/cgnr_solver.cc",
        "internal/ceres/coordinate_descent_minimizer.cc",
        "internal/ceres/compressed_col_sparse_matrix_utils.cc",
        "internal/ceres/compressed_row_jacobian_writer.cc",
        "internal/ceres/compressed_row_sparse_matrix.cc",
        "internal/ceres/conditioned_cost_function.cc",
        "internal/ceres/conjugate_gradients_solver.cc",
        "internal/ceres/context_impl.cc",
        "internal/ceres/corrector.cc",
        "internal/ceres/covariance.cc",
        "internal/ceres/covariance_impl.cc",
        "internal/ceres/dense_normal_cholesky_solver.cc",
        "internal/ceres/dense_qr_solver.cc",
        "internal/ceres/dense_sparse_matrix.cc",
        "internal/ceres/detect_structure.cc",
        "internal/ceres/dogleg_strategy.cc",
        "internal/ceres/dynamic_compressed_row_jacobian_writer.cc",
        "internal/ceres/dynamic_compressed_row_sparse_matrix.cc",
        "internal/ceres/dynamic_sparse_normal_cholesky_solver.cc",
        "internal/ceres/eigensparse.cc",
        "internal/ceres/evaluator.cc",
        "internal/ceres/file.cc",
        "internal/ceres/function_sample.cc",
        "internal/ceres/gradient_checker.cc",
        "internal/ceres/gradient_checking_cost_function.cc",
        "internal/ceres/gradient_problem.cc",
        "internal/ceres/gradient_problem_solver.cc",
        "internal/ceres/implicit_schur_complement.cc",
        "internal/ceres/inner_product_computer.cc",
        "internal/ceres/is_close.cc",
        "internal/ceres/iterative_refiner.cc",
        "internal/ceres/iterative_schur_complement_solver.cc",
        "internal/ceres/lapack.cc",
        "internal/ceres/levenberg_marquardt_strategy.cc",
        "internal/ceres/line_search.cc",
        "internal/ceres/line_search_direction.cc",
        "internal/ceres/line_search_minimizer.cc",
        "internal/ceres/line_search_preprocessor.cc",
        "internal/ceres/linear_least_squares_problems.cc",
        "internal/ceres/linear_operator.cc",
        "internal/ceres/linear_solver.cc",
        "internal/ceres/local_parameterization.cc",
        "internal/ceres/loss_function.cc",
        "internal/ceres/low_rank_inverse_hessian.cc",
        "internal/ceres/minimizer.cc",
        "internal/ceres/normal_prior.cc",
        "internal/ceres/parallel_for_cxx.cc",
        "internal/ceres/parameter_block_ordering.cc",
        "internal/ceres/partitioned_matrix_view.cc",
        "internal/ceres/polynomial.cc",
        "internal/ceres/preconditioner.cc",
        "internal/ceres/preprocessor.cc",
        "internal/ceres/problem.cc",
        "internal/ceres/problem_impl.cc",
        "internal/ceres/program.cc",
        "internal/ceres/reorder_program.cc",
        "internal/ceres/residual_block.cc",
        "internal/ceres/residual_block_utils.cc",
        "internal/ceres/schur_complement_solver.cc",
        "internal/ceres/schur_eliminator.cc",
        "internal/ceres/schur_jacobi_preconditioner.cc",
        "internal/ceres/schur_templates.cc",
        "internal/ceres/scratch_evaluate_preparer.cc",
        "internal/ceres/single_linkage_clustering.cc",
        "internal/ceres/solver.cc",
        "internal/ceres/solver_utils.cc",
        "internal/ceres/sparse_cholesky.cc",
        "internal/ceres/sparse_matrix.cc",
        "internal/ceres/sparse_normal_cholesky_solver.cc",
        "internal/ceres/split.cc",
        "internal/ceres/stringprintf.cc",
        "internal/ceres/suitesparse.cc",
        "internal/ceres/thread_pool.cc",
        "internal/ceres/thread_token_provider.cc",
        "internal/ceres/trust_region_minimizer.cc",
        "internal/ceres/trust_region_preprocessor.cc",
        "internal/ceres/trust_region_step_evaluator.cc",
        "internal/ceres/trust_region_strategy.cc",
        "internal/ceres/triplet_sparse_matrix.cc",
        "internal/ceres/types.cc",
        "internal/ceres/visibility_based_preconditioner.cc",
        "internal/ceres/visibility.cc",
        "internal/ceres/wall_time.cc",
    ] + glob([
        "internal/ceres/generated/schur_eliminator_*.cc",
        "internal/ceres/generated/partitioned_matrix_view_*.cc",
        "config/**/*.h",
        "internal/**/*.h",
    ]),
    hdrs = glob([
        "include/ceres/*.h",
        "include/ceres/internal/*.h",
    ]),
    copts = [
        "-fopenmp",
        "-Wno-sign-compare",
    ],
    defines = CERES_DEFINES,
    includes = [
        "config",
        "include",
        "internal",
    ],
    linkopts = [
        "-lgomp",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_glog//:glog",
        "@org_tuxfamily_eigen//:eigen",
    ],
)
