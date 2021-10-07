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
#define ERROR(__VA_ARGS__...) fprintf(stderr, __VA_ARGS__)
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

template <typename T>
static inline bool is_normalized(T val) {
  return val >= 0.0 && val <= 1.0;
}

template <typename T>
static bool is_valid_roi(Argus::Rectangle<T> roi) {
  g_return_val_if_fail(roi.left() < roi.right(), false);
  g_return_val_if_fail(roi.top() < roi.bottom(), false);
  g_return_val_if_fail(is_normalized(roi.left()), false);
  g_return_val_if_fail(is_normalized(roi.top()), false);
  g_return_val_if_fail(is_normalized(roi.right()), false);
  g_return_val_if_fail(is_normalized(roi.bottom()), false);
  return true;
}

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

  return std::make_unique<Metadata>(
      imeta, Argus::interface_cast<Argus::Ext::IBayerSharpnessMap>(meta));
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

Metadata::Metadata(Argus::ICaptureMetadata* imeta,
                   Argus::Ext::IBayerSharpnessMap* iBayerSharpnessMap)
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
#ifdef JETPACK_45
      flickerState_(imeta->getFlickerState()),
#endif  // JETPACK_45
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
      sharpnessValues_(nullopt),
#ifdef JETPACK_45
      sharpnessScore_(nullopt),
#endif  // JETPACK_45
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

  if (iBayerSharpnessMap) {
    Argus::Array2D<Argus::BayerTuple<float>> sharpnessValues;
    if (Argus::Status::STATUS_OK ==
        iBayerSharpnessMap->getSharpnessValues(&sharpnessValues)) {
      sharpnessValues_ = sharpnessValues;
    } else {
      WARNING(
          "IBayerSharpnessMap interface enabled but could not get sharpness "
          "values.");
    }

    // These are not very useful since they can be derived from the above and
    // image resolution, so disabling them:
    // sharpnessValuesBinCount_ = iBayerSharpnessMap->getBinCount();
    // sharpnessValuesBinInterval_ = iBayerSharpnessMap->getBinInterval();
    // sharpnessValuesBinSize_ = iBayerSharpnessMap->getBinSize();
    // sharpnessValuesBinStart_ = iBayerSharpnessMap->getBinStart();
  } else {
    LOG("IBayerSharpnessMap not available.");
  }

#ifdef JETPACK_45
  std::vector<float> sharpnessScore;
  if (Argus::Status::STATUS_OK == imeta->getSharpnessScore(&sharpnessScore)) {
    sharpnessScore_ = sharpnessScore;
  } else {
    LOG("Sharpness score not available.");
  }
#endif  // JETPACK_45

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

std::experimental::optional<float> Metadata::getSharpnessScore(
    Argus::Rectangle<float> roi) {
  // TODO(indra): use laplacian variance if sharpnessScore not available.
  //              possibly, use sharpness score together with variance for
  //              more accurate focus?
  // NOTE(mdegans): sharpnessScore will always be available. It's just "faked"
  //  as a mean of getSharpnessValues on the metadata in metadata.hpp (may
  //  move this to metadata.cpp). SharpnessValues may not be calculated at all
  //  if `bayer-sharpness-map` is set to false on `nvmanualcameasrc`.
  // get focus score from upstream, if it's available
  if (!is_valid_roi(roi)) {
    ERROR("Can't get sharpness score since supplied ROI is invalid.");
    return nullopt;
  }
  if (auto opt_values = sharpnessValues_) {
    // Bayer Sharpness Map
    Argus::Array2D<Argus::BayerTuple<float>> bsm = opt_values.value();
    // PRAISE(mdegans): to Nvidia for using top, left, width, and height instead
    // of x and y, which despite using for years I can never remember which is
    // which. I can recite my credit card number, but not that.
    // TODO(mdegans): linear filtering, since right now this is basically
    //  "nearest"
    uint32_t bsm_width = bsm.size().width();
    uint32_t bsm_height = bsm.size().height();
    Argus::Point2D<uint> tl(static_cast<uint>(roi.left() * bsm_width),
                            static_cast<uint>(roi.top() * bsm_height));
    Argus::Point2D<uint> br(static_cast<uint>(roi.right() * bsm_width),
                            static_cast<uint>(roi.bottom() * bsm_height));
    float sharpness_sum = 0.0f;
    Argus::BayerTuple<float> rggb = {};
    for (uint x = tl.x(); x < br.x(); x++) {
      for (uint y = tl.y(); y < br.y(); y++) {
        rggb = bsm[y * bsm_width + x];
        sharpness_sum +=
            (rggb.r() + rggb.gEven() + rggb.gOdd() + rggb.b()) / 4.0f;
      }
    }

    return sharpness_sum /
           static_cast<float>((tl.x() - br.x()) * (tl.y() - br.y()));
  } else {
    ERROR(
        "bayer-sharpness-map needs to be enabled on `nvmanualcameasrc` to use "
        "getSharpnessScore with a roi. If it is enabled, please report this.");
    return nullopt;
  }
}
#ifdef JETPACK_45
/**
 * @brief Get the sharpness score if available. With JetPack 4.5, this is
 * available even if `bayer-sharpness-map` isn't enabled. This will not
 * provide exactly the same score as < JetPack 4.5, but it should be similar.
 *
 * @return std::experimental::optional<std::vector<float>>
 */
std::experimental::optional<float> Metadata::getSharpnessScore() const {
  if (!sharpnessScore_) {
    return std::experimental::nullopt;
  }
  const auto ss = sharpnessScore_.value();
  g_assert(ss.size() == 1);  // in tests, it's always a vector of a single
  // float, which seems pointless
  return ss.at(0);
}
#else   // fallback < JETPACK_45
/**
 * @brief Get the mean of getSharpnessValues if available. This will not
 * provide exactly the same score as > JetPack 4.5 version but it should
 * be similiar. If you need a ROI, use `getSharpnessValues`.
 *
 * @return std::experimental::optional<std::vector<float>>
 */
std::experimental::optional<float> Metadata::getSharpnessScore() const {
  if (!sharpnessValues_) {
    return std::experimental::nullopt;
  }
  const auto& vals = sharpnessValues_.value();
  const auto len = vals.size().area();
  g_assert(len);  // should never be zero
  float sum = 0.0f;
  for (const auto& val : vals) {
    sum += val.r() + val.gEven() + val.gOdd() + val.b();
  }
  // approximately the same as JETPACK_45
  return (sum * 1000.0f) / (static_cast<float>(len) * 4.0f);
}
#endif  // JETPACK_45

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
