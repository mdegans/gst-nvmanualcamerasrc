#include <stdexcept>
#include <string>
// #include <Argus/Argus.h>
// #include <bits/stdint-uintn.h>

#include "gst/gst.h"

#include "gst/gstbuffer.h"
#include "gst/gstpad.h"
#include "pybind11/pybind11.h"
#include "pybind11/operators.h"
#include "pybind11/cast.h"
#include "pybind11/stl.h"
#include "pybind11/pytypes.h"

#include "metadata.hpp"

namespace py = pybind11;

// // https://stackoverflow.com/questions/47487888/pybind11-template-class-of-many-types/47749076
template <typename T>
void declare_rectangle(py::module& m, std::string typestr) {
  py::class_<Argus::Rectangle<T>>(m, (typestr + std::string("Rectangle")).c_str())
    .def(py::init<T, T, T, T>(), py::arg("left"), py::arg("top"), py::arg("right"), py::arg("bottom"))
    .def_property_readonly("left", py::overload_cast<>(&Argus::Rectangle<T>::left))
    .def_property_readonly("top", py::overload_cast<>(&Argus::Rectangle<T>::top))
    .def_property_readonly("right", py::overload_cast<>(&Argus::Rectangle<T>::right))
    .def_property_readonly("bottom", py::overload_cast<>(&Argus::Rectangle<T>::bottom));
}


// because pybind doesn't detect this correctly on ubuntu 20.04 (or 18.04)]
// (or I am doing something wrong). absl::optional or boost::optional can
// also work
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#c-17-library-containers
namespace pybind11 {
namespace detail {
template <typename T>
struct type_caster<std::experimental::optional<T>>
    : optional_caster<std::experimental::optional<T>> {};
}  // namespace detail
}  // namespace pybind11


PYBIND11_MODULE(nvmanual, m) {

  py::module m_argus = m.def_submodule("argus", "Unofficial Argus bindings");

  declare_rectangle<float>(m_argus, "Float");  // argus.FloatRectangle
  declare_rectangle<uint32_t>(m_argus, "Int");  // argus.IntRectangle

  py::class_<nvmanualcam::Metadata> c_metadata(m, "Metadata");
  c_metadata.def_static("from_buffer", [](py::object pygobject_buf) {
      // reinterpret_cast is now Nvidia does this in the `nvds` bindings.
      // They require the user call `__hash__`, but we can do that for them and
      // check we actually got a GstBuffer (as much as GObject can ensure that
      // anyhow)
      auto buf = reinterpret_cast<GstBuffer*>(py::hash(pygobject_buf));
      if (!GST_IS_BUFFER(buf)) {
        throw std::invalid_argument("Not a valid Gst.Buffer");
      }
      return nvmanualcam::Metadata::create(buf);
    }, py::arg("buf")).doc() = "create nvmanual.Metadata from a Gst.Buffer";
    // .def("bayer_histogram", &nvmanualcam::Metadata::getBayerHistogram)
    // .def("rgb_histogram", &nvmanualcam::Metadata::getRgbHistogram)
  c_metadata.def_property_readonly("scene_lux", &nvmanualcam::Metadata::getSceneLux);
  c_metadata.def(
    "sharpness_score",
    py::overload_cast<Argus::Rectangle<float>>(&nvmanualcam::Metadata::getSharpnessScore),
    py::arg("rect") = Argus::Rectangle<float>(0.0, 0.0, 1.0, 1.0)
  ).doc() = "Get the sharpness score for the image or a ROI within it.";
    // .def("tonemap_curve", &nvmanualcam::Metadata::getToneMapCurves)
    // .def("color_correction_matrix", &nvmanualcam::Metadata::getBayerHistogram);
}