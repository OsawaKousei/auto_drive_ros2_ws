#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <utility>
#include "lib/spline.hpp"

namespace py = pybind11;

PYBIND11_MODULE(spline_module, m)
{
      m.doc() = "Spline interpolation module";

      // spline_by_num関数をエクスポート
      m.def("spline_by_num", &path_planning::spline_by_num,
            "Interpolate points using a spline with a specified number of points",
            py::arg("xs"), py::arg("ys"), py::arg("num_points"));

      // spline_by_min_max関数をエクスポート
      m.def("spline_by_min_max", &path_planning::spline_by_min_max,
            "Interpolate points using a spline with specified min and max distances",
            py::arg("xs"), py::arg("ys"), py::arg("l_min"), py::arg("l_max"), py::arg("d_max"));
}
