#include <bits/stdint-uintn.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <memory>

#include <Argus/Argus.h>

#include "gst/gst.h"
#include "gst/gstbuffer.h"

// #include "pybind11/attr.h"
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

template <typename T>
void declare_array_2d(py::module& m, std::string typestr) {
    py::class_<Argus::Array2D<T>>(m, (typestr + "Array2D").c_str())
      .def_static("from_xy", [](uint32_t x, uint32_t y){
        return std::make_unique<Argus::Array2D<T>>(Argus::Size2D(x, y));
      })
      .def("__len__", [](const Argus::Array2D<T> &a){ return a.size().area(); })
      .def("__iter__", [](const Argus::Array2D<T> &a) {
        return py::make_iterator(a.begin(), a.end());
      }, py::keep_alive<0, 1>())
      .def("__getitem__", [](const Argus::Array2D<T> &a, uint32_t index) {
        // We need to raise a python error since the assert in Types.h can't be
        // caught.  checkIndex being public would have been handy here.
        if (index >= a.size().area()) {
          std::stringstream ss;
          ss << "Index " << index << " is out of range for this Array2D (max " << a.size().area() << ")";
          throw py::index_error(ss.str());
        }
        return a[index];
      })
      .def("at", [](const Argus::Array2D<T> a, uint32_t x, uint32_t y) {
        if (x >= a.size().width()) {
          std::stringstream ss;
          ss << "X index is out of range for this Array2d (max " << a.size().width() << ")";
          throw py::index_error(ss.str());
        }
        if (y >= a.size().height()) {
          std::stringstream ss;
          ss << "Y index is out of range for this Array2d (max " << a.size().height() <<  ")";
          throw py::index_error(ss.str());
        }
        return a(x, y);
      }, py::arg("x"), py::arg("y")).doc() = "Access elements by x, y";
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
  declare_array_2d<Argus::BayerTuple<float>>(m_argus, "FloatBayerTuple");  // argus.FloatBayerTupleArray2D

  // This isn't an enum because Nvidia rolled it's own C++ 03 thing.
  // FIXME(mdegans): figure out how to bind this
  // py::enum_<Argus::AeState>(m_argus, "AeState")
  //   .value("INACTIVE", Argus::AE_STATE_INACTIVE)
  //   .value("SEARCHING", Argus::AE_STATE_SEARCHING)
  //   .value("CONVERGED", Argus::AE_STATE_CONVERGED)
  //   .value("FLASH_REQUIRED", Argus::AE_STATE_FLASH_REQUIRED)
  //   .value("TIMEOUT", Argus::AE_STATE_TIMEOUT);  // argus.AeState

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
  c_metadata.def("sharpness_values", &nvmanualcam::Metadata::getSharpnessValues)
    .doc() = "Get 64x64 array of sharpness values.";
  c_metadata.def(
    "sharpness_score",
    py::overload_cast<Argus::Rectangle<float>>(&nvmanualcam::Metadata::getSharpnessScore),
    py::arg("rect") = Argus::Rectangle<float>(0.0, 0.0, 1.0, 1.0)
  ).doc() = "Get the sharpness score for the image or a ROI within it.";
  c_metadata.def("tonemap_curves", &nvmanualcam::Metadata::getToneMapCurves)
    .doc() = "Get tonemap curves (if enabled, else retursn None)";
  c_metadata.def_property_readonly("ae_locked", &nvmanualcam::Metadata::getAeLocked)
    .doc() = "Returns true if Auto Exposure was locked for this capture.";
  // c_metadata.def_property_readonly("ae_state", &nvmanualcam::Metadata::getAeState)
  //   .doc() = "Get Auto Exposure state for this capture";
  c_metadata.def_property_readonly("awb_cct", &nvmanualcam::Metadata::getAwbCct)
    .doc() = "Get Auto White Balance CCT (color temperature in Kelvin)";
  c_metadata.def_property_readonly("awb_gains", &nvmanualcam::Metadata::getAwbGains)
    .doc() = "Get Auto White Balance gains.";
  // TODO(AE Regions and Argus::AcRegion)
  // TODO(AWB Regions, AWB Mode, AWB State)
  c_metadata.def("awb_estimate", &nvmanualcam::Metadata::getAwbWbEstimate)
    .doc() = "Get Auto White Balance estimate (if available, else returns None)";
  c_metadata.def_property_readonly("capture_id", &nvmanualcam::Metadata::getCaptureId)
    .doc() = "Get capture id (close, but not quite the frame number)";
  c_metadata.def("color_correction_matrix", &nvmanualcam::Metadata::getColorCorrectionMatrix)
    .doc() = "Get color correction matrix for this capture (if available else None()";
  c_metadata.def_property_readonly("color_correction_matrix_enable", &nvmanualcam::Metadata::getColorCorrectionMatrixEnable)
    .doc() = "Returns true if color correction matrix is enabled for this capture.";
  c_metadata.def_property_readonly("saturation", &nvmanualcam::Metadata::getColorSaturation)
    .doc() = "Get saturation for this capture (0.0 - 1.0)";
  // TODO(Argus::AeFlickerState)
  c_metadata.def_property_readonly("focuser_position", &nvmanualcam::Metadata::getFocuserPosition)
    .doc() = "Get focuser position (if applicable, else value is useless)";
  c_metadata.def_property_readonly("frame_duration", &nvmanualcam::Metadata::getFrameDuration)
    .doc() = "Get the duration it took to generate the associated frame in NS";
  c_metadata.def_property_readonly("frame_readout_time", &nvmanualcam::Metadata::getFrameReadoutTime)
    .doc() = "Get the frame readout time in NS.";
  c_metadata.def_property_readonly("isp_digital_gain", &nvmanualcam::Metadata::getIspDigitalGain)
    .doc() = "Get the ISP digital gain for this capture.";
  c_metadata.def_property_readonly("sensor_analog_gain", &nvmanualcam::Metadata::getSensorAnalogGain)
    .doc() = "Get the analog gain for this capture.";
  c_metadata.def_property_readonly("sensor_exposure_time", &nvmanualcam::Metadata::getSensorExposureTime)
    .doc() = "Get the sensor exposure time (in NS)";
  c_metadata.def_property_readonly("sensor_sensitivity", &nvmanualcam::Metadata::getSensorSensitivity)
    .doc() = "Get the sensor sensitivity (ISO value)";
  c_metadata.def_property_readonly("sensor_timestamp", &nvmanualcam::Metadata::getSensorTimestamp)
    .doc() = "Get the sensor timestamp.";
}