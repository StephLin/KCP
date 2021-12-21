#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "kcp/keypoint.hpp"
#include "kcp/solver.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(pykcp, m) {
  py::class_<kcp::Correspondences>(m, "Correspondences")
      .def(py::init<>())
      .def_readwrite("points", &kcp::Correspondences::points)
      .def_readwrite("indices", &kcp::Correspondences::indices);

  py::class_<kcp::keypoint::RangeImage>(m, "RangeImage")
      .def(py::init<Eigen::MatrixX3d, int, float, float, int>(),
           py::arg("cloud"),
           py::arg("n_channels")      = 32,
           py::arg("min_vfov_deg")    = -30.0,
           py::arg("max_vfov_deg")    = 10.0,
           py::arg("hfov_resolution") = 1800)
      .def("get_cloud", &kcp::keypoint::RangeImage::get_cloud, py::return_value_policy::copy)
      .def("get_n_channels", &kcp::keypoint::RangeImage::get_n_channels)
      .def("get_image_sequence_size", &kcp::keypoint::RangeImage::get_image_sequence_size)
      .def("get_image_indices", &kcp::keypoint::RangeImage::get_image_indices, py::return_value_policy::copy)
      .def("get_image_depth_sequence", &kcp::keypoint::RangeImage::get_image_depth_sequence, py::return_value_policy::copy)
      .def("get_image_point_indices_sequence", &kcp::keypoint::RangeImage::get_image_point_indices_sequence, py::return_value_policy::copy)
      .def("get_image_col_indices_sequence", &kcp::keypoint::RangeImage::get_image_col_indices_sequence, py::return_value_policy::copy)
      .def("get_channel_start_indices", &kcp::keypoint::RangeImage::get_channel_start_indices, py::return_value_policy::copy)
      .def("get_channel_end_indices", &kcp::keypoint::RangeImage::get_channel_end_indices, py::return_value_policy::copy);

  py::class_<kcp::keypoint::MultiScaleCurvature>(m, "MultiScaleCurvature")
      .def(py::init<kcp::keypoint::RangeImage, float, float>(),
           py::arg("range_image"),
           py::arg("corner_threshold") = 30.0,
           py::arg("plane_threshold")  = 0.1)
      .def(py::init<Eigen::MatrixX3d, int, float, float, int, float, float>(),
           py::arg("cloud"),
           py::arg("n_channels")       = 32,
           py::arg("min_vfov_deg")     = -30.0,
           py::arg("max_vfov_deg")     = 10.0,
           py::arg("hfov_resolution")  = 1800,
           py::arg("corner_threshold") = 30.0,
           py::arg("plane_threshold")  = 0.1)
      .def("get_range_image", &kcp::keypoint::MultiScaleCurvature::get_range_image, py::return_value_policy::copy)
      .def("get_corner_points", &kcp::keypoint::MultiScaleCurvature::get_corner_points, py::return_value_policy::copy)
      .def("get_plane_points", &kcp::keypoint::MultiScaleCurvature::get_plane_points, py::return_value_policy::copy)
      .def("get_corner_point_indices", &kcp::keypoint::MultiScaleCurvature::get_corner_point_indices, py::return_value_policy::copy)
      .def("get_plane_point_indices", &kcp::keypoint::MultiScaleCurvature::get_plane_point_indices, py::return_value_policy::copy)
      .def("get_curvature", &kcp::keypoint::MultiScaleCurvature::get_curvature, py::return_value_policy::copy);

  py::class_<kcp::KCP::TEASER::Params>(m, "TEASERParams")
      .def(py::init<>())
      .def_readwrite("noise_bound", &kcp::KCP::TEASER::Params::noise_bound)
      .def_readwrite("cbar2", &kcp::KCP::TEASER::Params::cbar2)
      .def_readwrite("estimate_scaling", &kcp::KCP::TEASER::Params::estimate_scaling)
      .def_readwrite("rotation_gnc_factor", &kcp::KCP::TEASER::Params::rotation_gnc_factor)
      .def_readwrite("rotation_estimation_algorithm", &kcp::KCP::TEASER::Params::rotation_estimation_algorithm)
      .def_readwrite("rotation_max_iterations", &kcp::KCP::TEASER::Params::rotation_max_iterations)
      .def_readwrite("rotation_cost_threshold", &kcp::KCP::TEASER::Params::rotation_cost_threshold)
      .def_readwrite("kcore_heuristic_threshold", &kcp::KCP::TEASER::Params::kcore_heuristic_threshold)
      .def_readwrite("use_max_clique", &kcp::KCP::TEASER::Params::use_max_clique)
      .def_readwrite("max_clique_exact_solution", &kcp::KCP::TEASER::Params::max_clique_exact_solution)
      .def_readwrite("max_clique_time_limit", &kcp::KCP::TEASER::Params::max_clique_time_limit);

  py::class_<kcp::KCP::Params>(m, "KCPParams")
      .def(py::init<>())
      .def_readwrite("k", &kcp::KCP::Params::k)
      .def_readwrite("teaser", &kcp::KCP::Params::teaser);

  py::class_<kcp::KCP>(m, "KCP")
      .def(py::init<kcp::KCP::Params>())
      .def("get_params", &kcp::KCP::get_params, py::return_value_policy::reference)
      .def("get_initial_correspondences", &kcp::KCP::get_initial_correspondences)
      .def("get_inlier_correspondence_indices", &kcp::KCP::get_inlier_correspondence_indices)
      .def("solve", &kcp::KCP::solve)
      .def("get_solution", &kcp::KCP::get_solution);
}