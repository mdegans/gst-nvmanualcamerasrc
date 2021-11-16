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
#include "pybind11/detail/common.h"

#include "metadata.hpp"
#include "pybind11/pytypes.h"

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


PYBIND11_MODULE(nvmanual, m) {

  py::module argus = m.def_submodule("argus", "Unofficial Argus bindings");

  declare_rectangle<float>(argus, "Float");  // argus.FloatRectangle
  declare_rectangle<uint32_t>(argus, "Int");  // argus.IntRectangle

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
  c_metadata.def("scene_lux", &nvmanualcam::Metadata::getSceneLux);
    // .def("sharpness_values", &nvmanualcam::Metadata::getSharpnessValues)
    // .def("tonemap_curve", &nvmanualcam::Metadata::getToneMapCurves)
    // .def("color_correction_matrix", &nvmanualcam::Metadata::getBayerHistogram);
}