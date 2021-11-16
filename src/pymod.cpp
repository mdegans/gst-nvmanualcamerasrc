#include <stdexcept>
#include <string>
// #include <Argus/Argus.h>
// #include <bits/stdint-uintn.h>

#include "gst/gst.h"
#include "gst/gstbuffer.h"

#include "pybind11/pybind11.h"
#include "pybind11/detail/common.h"
#include "pybind11/operators.h"
#include "pybind11/cast.h"
#include "pybind11/stl.h"
#include "pybind11/pytypes.h"

#include "metadata.hpp"

namespace py = pybind11;

// // https://stackoverflow.com/questions/47487888/pybind11-template-class-of-many-types/47749076
template <typename T>
void declare_rectangle(py::module& m, std::string typestr) {
  py::class_<Argus::Rectangle<T>>(m, (typestr + "Rectangle").c_str())
    .def(py::init<T, T, T, T>(), py::arg("left"), py::arg("top"), py::arg("right"), py::arg("bottom"))
    .def_property_readonly("left", py::overload_cast<>(&Argus::Rectangle<T>::left))
    .def_property_readonly("top", py::overload_cast<>(&Argus::Rectangle<T>::top))
    .def_property_readonly("right", py::overload_cast<>(&Argus::Rectangle<T>::right))
    .def_property_readonly("bottom", py::overload_cast<>(&Argus::Rectangle<T>::bottom));
}

template <typename T>
void declare_rgb_tuple(py::module& m, std::string typestr) {
  py::class_<Argus::RGBTuple<T>>(m, (typestr + "RGBTuple").c_str())
    .def(py::init<T, T, T>(), py::arg("r"), py::arg("g"), py::arg("b"))
    .def_property_readonly("r", py::overload_cast<>(&Argus::RGBTuple<T>::r))
    .def_property_readonly("g", py::overload_cast<>(&Argus::RGBTuple<T>::g))
    .def_property_readonly("b", py::overload_cast<>(&Argus::RGBTuple<T>::b));
}

template <typename T>
void declare_bayer_tuple(py::module& m, std::string typestr) {
  py::class_<Argus::BayerTuple<T>>(m, (typestr + "BayerTuple").c_str())
    .def(py::init<T, T, T, T>(), py::arg("r"), py::arg("g_even"), py::arg("g_odd"), py::arg("b"))
    .def_property_readonly("r", py::overload_cast<>(&Argus::BayerTuple<T>::r))
    .def_property_readonly("g_even", py::overload_cast<>(&Argus::BayerTuple<T>::gEven))
    .def_property_readonly("g_odd", py::overload_cast<>(&Argus::BayerTuple<T>::gOdd))
    .def_property_readonly("b", py::overload_cast<>(&Argus::BayerTuple<T>::b));
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
  declare_rgb_tuple<float>(m_argus, "Float");  // argus.FloatRGBTuple
  declare_rgb_tuple<uint32_t>(m_argus, "Int");  // argus.IntRGBTuple
  declare_bayer_tuple<float>(m_argus, "Float");  // argus.FloatBayerTuple
  declare_bayer_tuple<uint32_t>(m_argus, "Int");  // argus.IntBayerTuple

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
  c_metadata.def("bayer_histogram", &nvmanualcam::Metadata::getBayerHistogram)
    .doc() = "Get histogram of bayer data (if enabled, else returns None)";
  c_metadata.def("rgb_histogram", &nvmanualcam::Metadata::getRgbHistogram)
    .doc() = "Get rgb histogram (if enabled, else returns None)";
  c_metadata.def_property_readonly("scene_lux", &nvmanualcam::Metadata::getSceneLux)
    .doc() = "Get approximate scene illumination in lux.";
  c_metadata.def(
    "sharpness_score",
    py::overload_cast<Argus::Rectangle<float>>(&nvmanualcam::Metadata::getSharpnessScore),
    py::arg("rect") = Argus::Rectangle<float>(0.0, 0.0, 1.0, 1.0)
  ).doc() = "Get the sharpness score for the image or a ROI within it.";
    // .def("tonemap_curve", &nvmanualcam::Metadata::getToneMapCurves)
    // .def("color_correction_matrix", &nvmanualcam::Metadata::getBayerHistogram);
}