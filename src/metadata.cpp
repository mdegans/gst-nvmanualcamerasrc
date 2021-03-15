/*
 * Copyright (c) 2021 Michael de Gans
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "metadata.hpp"

#include <gst/gst.h>

#ifdef HAS_BOOST_JSON
#include <boost/json/src.hpp>
#endif

#include <tuple>
using std::experimental::nullopt;
using std::experimental::optional;

// Setup logging. Use Gstreamer if available. Otherwise use printf.
// TODO: use boost log if available, since printf is slow.
#ifdef HAS_GSTREAMER
GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_metadata_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_metadata_debug
#define LOG GST_LOG
#define WARNING GST_WARNING
#define ERROR GST_ERROR
#else  // !HAS_GSTREAMER ... so we use printf family instead :(
#include <cstdio>
#define LOG printf
#define WARNING printf
#define ERROR(__VA_ARGS__...) fprintf(sterr, __VA_ARGS__)
#endif  // HAS_GSTREAMER

// static gstreamer tools be here.
#ifdef HAS_GSTREAMER
/**
 * @brief get the quark for nvcvcam::Metadata (internal)
 */
static G_DEFINE_QUARK(NvManualCamMetadata, manual_meta);
/**
 * @brief A GDestroyNotify callback to destroy the Metadata when a buffer it's
 * attached to reaches the end of it's life.
 *
 * @param meta to destroy
 */
static void destroy_metadata(nvmanualcam::Metadata* meta) {
  LOG("Freeing metadata.");
  delete meta;
}
#endif  // HAS_GSTREAMER

namespace nvmanualcam {

#ifdef HAS_GSTREAMER
void attachMetadata(std::unique_ptr<Metadata>&& meta, GstBuffer* buffer) {
  // attach the metadata to the buffer.
  gst_mini_object_set_qdata(GST_MINI_OBJECT_CAST(buffer),
                            nvmanualcam::Metadata::quark(), meta.release(),
                            (GDestroyNotify)destroy_metadata);
}
#endif  // HAS_GSTREAMER

std::unique_ptr<Metadata> Metadata::create(Argus::CaptureMetadata* meta) {
  // It is safe to call this multiple times. It checks if the category is
  // NULL.
#ifdef HAS_GSTREAMER
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
#endif  // HAS_GSTREAMER
  LOG("Creating unique metadata from Argus metadata.");
  if (!meta) {
    WARNING("Argus CaptureMetadata is NULL.");
    return nullptr;
  }

  auto imeta = Argus::interface_cast<Argus::ICaptureMetadata>(meta);
  if (!imeta) {
    WARNING("Could not get ICaptureMetadata interface.");
    return nullptr;
  }

  return std::make_unique<Metadata>(imeta);
}

#ifdef HAS_GSTREAMER
std::unique_ptr<Metadata> Metadata::create(GstPadProbeInfo* info) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
  LOG("Copying Metadata from GstPadProbeInfo.");
  GstBuffer* buf = nullptr;

  buf = gst_pad_probe_info_get_buffer(info);  // [transfer: none]
  if (!buf) {
    WARNING("Unable to get GstBuffer from GstPadProbeInfo.");
    return nullptr;
  }

  return Metadata::create(buf);
}

std::unique_ptr<Metadata> Metadata::create(GstBuffer* buf) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
  LOG("Copying Metadata from GstBuffer.");
  Metadata* raw = nullptr;

  // this does not take ownership, in contrast with `...steal_qdata`
  raw = (Metadata*)gst_mini_object_get_qdata(GST_MINI_OBJECT_CAST(buf),
                                             quark());  // [transfer: none]

  if (!raw) {
    WARNING("Metadata not attached to buffer.");
    return nullptr;
  }

  return std::make_unique<Metadata>(*raw);
}

std::unique_ptr<Metadata> Metadata::steal(GstPadProbeInfo* info) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
  LOG("Taking ownership of Metadata from GstPadProbeInfo.");
  GstBuffer* buf = nullptr;

  buf = gst_pad_probe_info_get_buffer(info);  // [transfer: none]
  if (!buf) {
    WARNING("Unable to get GstBuffer from GstPadProbeInfo.");
    return nullptr;
  }

  return Metadata::steal(buf);
}

std::unique_ptr<Metadata> Metadata::steal(GstBuffer* buf) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
  LOG("Taking ownership of Metadata from GstBuffer.");
  Metadata* raw = nullptr;

  // this takes ownership, so the deleter attached to the buffer is not called
  raw = (Metadata*)gst_mini_object_steal_qdata(GST_MINI_OBJECT_CAST(buf),
                                               quark());

  if (!raw) {
    WARNING("Metadata not attached to buffer.");
    return nullptr;
  }

  return std::unique_ptr<Metadata>(raw);
}
#endif  // HAS_GSTREAMER

#ifdef HAS_BOOST_JSON
std::unique_ptr<Metadata> Metadata::create(std::string json) {
  ERROR("create from json not implemented");
  return nullptr;
}
#endif  // HAS_BOOST_JSON

#ifdef HAS_GSTREAMER
GQuark Metadata::quark() {
  return manual_meta_quark();
}
#endif  // HAS_GSTREAMER

Metadata::Metadata(Argus::ICaptureMetadata* imeta)
    : aeLocked_(imeta->getAeLocked()),
      aeRegions_(nullopt),
      aeState_(imeta->getAeState()),
      awbCct_(imeta->getAwbCct()),
      awbGains_(imeta->getAwbGains()),
      awbMode_(imeta->getAwbMode()),
      awbRegions_(nullopt),
      awbState_(imeta->getAwbState()),
      awbWbEstimate_(nullopt),
      bayerHistogramBinCount_(nullopt),
      bayerHistogram_(nullopt),
      bayerHistogramRegion_(nullopt),
      captureId_(imeta->getCaptureId()),
      clientData_(imeta->getClientData()),
      colorCorrectionMatrix_(nullopt),
      colorCorrectionMatrixEnabled_(imeta->getColorCorrectionMatrixEnable()),
      colorSaturation_(imeta->getColorSaturation()),
      flickerState_(imeta->getFlickerState()),
      focuserPosition_(imeta->getFocuserPosition()),
      frameDuration_(imeta->getFrameDuration()),
      frameReadoutTime_(imeta->getFrameReadoutTime()),
      ispDigitalGain_(imeta->getIspDigitalGain()),
      rbgHistogramBinCount_(nullopt),
      rgbHistogram_(nullopt),
      sceneLux_(imeta->getSceneLux()),
      sensorAnalogGain_(imeta->getSensorAnalogGain()),
      sensorExposureTime_(imeta->getSensorExposureTime()),
      sensorSensitivity_(imeta->getSensorSensitivity()),
      sensorTimestamp_(imeta->getSensorTimestamp()),
      sharpnessScore_(nullopt),
      toneMapCurveR_(nullopt),
      toneMapCurveG_(nullopt),
      toneMapCurveB_(nullopt),
      toneMapCurveEnabled_(imeta->getToneMapCurveEnabled()) {
  assert(imeta);

  std::vector<Argus::AcRegion> aeRegions;
  if (Argus::Status::STATUS_OK == imeta->getAeRegions(&aeRegions)) {
    aeRegions_ = aeRegions;
  } else {
    LOG("AE regions not available.");
  }

  std::vector<Argus::AcRegion> awbRegions;
  if (Argus::Status::STATUS_OK == imeta->getAwbRegions(&awbRegions)) {
    awbRegions_ = awbRegions;
  } else {
    LOG("AWB regions not available.");
  }

  std::vector<float> awbWbEstimate;
  if (Argus::Status::STATUS_OK == imeta->getAwbWbEstimate(&awbWbEstimate)) {
    awbWbEstimate_ = awbWbEstimate;
  } else {
    LOG("AWB WB Estimate not available.");
  }

  auto ibayer = Argus::interface_cast<const Argus::IBayerHistogram>(
      imeta->getBayerHistogram());
  if (ibayer) {
    bayerHistogramBinCount_ = ibayer->getBinCount();
    std::vector<Argus::BayerTuple<uint32_t>> bayerHistogram;
    if (Argus::Status::STATUS_OK == ibayer->getHistogram(&bayerHistogram)) {
      bayerHistogram_ = bayerHistogram;
    } else {
      WARNING("Could not get Bayer histogram even though enabled.");
    }
  } else {
    LOG("Bayer histogram not available.");
  }

  if (colorCorrectionMatrixEnabled_) {
    std::vector<float> colorCorrectionMatrix;
    if (Argus::Status::STATUS_OK ==
        imeta->getColorCorrectionMatrix(&colorCorrectionMatrix)) {
      colorCorrectionMatrix_ = colorCorrectionMatrix;
    } else {
      WARNING("Could not get color correction matrix even though enabled.");
    }
  } else {
    LOG("Color correction matrix not available.");
  }

  auto irgbhist = Argus::interface_cast<const Argus::IRGBHistogram>(
      imeta->getRGBHistogram());
  if (irgbhist) {
    rbgHistogramBinCount_ = irgbhist->getBinCount();
    std::vector<Argus::RGBTuple<uint32_t>> rgbHistogram;
    if (Argus::Status::STATUS_OK == irgbhist->getHistogram(&rgbHistogram)) {
      rgbHistogram_ = rgbHistogram;
    } else {
      WARNING("Could not get RGB histogram even though enabled.");
    }
  } else {
    LOG("RGB histogram not available.");
  }

  std::vector<float> sharpnessScore;
  if (Argus::Status::STATUS_OK == imeta->getSharpnessScore(&sharpnessScore)) {
    sharpnessScore_ = sharpnessScore;
  } else {
    LOG("Sharpness score not available.");
  }

  if (toneMapCurveEnabled_) {
    std::vector<float> toneMapCurve;

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_R,
                               &toneMapCurve)) {
      toneMapCurveR_ = toneMapCurve;
    } else {
      WARNING("Could not get R tonemap curve even though enabled.");
    }

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_G,
                               &toneMapCurve)) {
      toneMapCurveG_ = toneMapCurve;
    } else {
      WARNING("Could not get G tonemap curve even though enabled.");
    }

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_B,
                               &toneMapCurve)) {
      toneMapCurveB_ = toneMapCurve;
    } else {
      WARNING("Could not get B tonemap curve even though enabled.");
    }
  } else {
    LOG("Tone map curve not available.");
  }
}

#ifdef HAS_BOOST_JSON
std::string to_json() const {
  ERROR("not implemented");
  return R"(["not implemented"])";
}
#endif

std::experimental::optional<RGBCurves> Metadata::getToneMapCurves() const {
  if (toneMapCurveEnabled_ && toneMapCurveR_ && toneMapCurveG_ &&
      toneMapCurveB_) {
    return std::make_tuple(toneMapCurveR_.value(), toneMapCurveG_.value(),
                           toneMapCurveB_.value());
  } else {
    return nullopt;
  }
}

}  // namespace nvmanualcam
