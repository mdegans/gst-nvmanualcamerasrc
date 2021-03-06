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

#include <tuple>

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_metadata_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_metadata_debug

using std::experimental::nullopt;
using std::experimental::optional;

namespace nvmanualcam {

optional<Metadata> Metadata::create(Argus::CaptureMetadata* meta) {
  // It is safe to call this multiple times. It checks if the category is NULL.
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_metadata_debug,
                          "nvmanualcamerasrc:metadata", 0,
                          "nvmanualcamerasrc metadata");
  if (!meta) {
    GST_WARNING("Could not create Metadata.");
    return nullopt;
  }
  auto imeta = Argus::interface_cast<Argus::ICaptureMetadata>(meta);
  if (!imeta) {
    GST_WARNING("Could not get ICaptureMetadata interface");
    return nullopt;
  }
  return Metadata(imeta);
}

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
  std::vector<Argus::AcRegion> aeRegions;
  if (Argus::Status::STATUS_OK == imeta->getAeRegions(&aeRegions)) {
    aeRegions_ = aeRegions;
  } else {
    GST_LOG("AE regions not available.");
  }

  std::vector<Argus::AcRegion> awbRegions;
  if (Argus::Status::STATUS_OK == imeta->getAwbRegions(&awbRegions)) {
    awbRegions_ = awbRegions;
  } else {
    GST_LOG("AWB regions not available.");
  }

  std::vector<float> awbWbEstimate;
  if (Argus::Status::STATUS_OK == imeta->getAwbWbEstimate(&awbWbEstimate)) {
    awbWbEstimate_ = awbWbEstimate;
  } else {
    GST_LOG("AWB WB Estimate not available.");
  }

  auto ibayer = Argus::interface_cast<const Argus::IBayerHistogram>(
      imeta->getBayerHistogram());
  if (ibayer) {
    bayerHistogramBinCount_ = ibayer->getBinCount();
    std::vector<Argus::BayerTuple<uint32_t>> bayerHistogram;
    if (Argus::Status::STATUS_OK == ibayer->getHistogram(&bayerHistogram)) {
      bayerHistogram_ = bayerHistogram;
    } else {
      GST_WARNING("Could not get Bayer histogram even though enabled.");
    }
  } else {
    GST_LOG("Bayer histogram not available.");
  }

  if (colorCorrectionMatrixEnabled_) {
    std::vector<float> colorCorrectionMatrix;
    if (Argus::Status::STATUS_OK ==
        imeta->getColorCorrectionMatrix(&colorCorrectionMatrix)) {
      colorCorrectionMatrix_ = colorCorrectionMatrix;
    } else {
      GST_WARNING("Could not get color correction matrix even though enabled.");
    }
  } else {
    GST_LOG("Color correction matrix not available.");
  }

  auto irgbhist = Argus::interface_cast<const Argus::IRGBHistogram>(
      imeta->getRGBHistogram());
  if (irgbhist) {
    rbgHistogramBinCount_ = irgbhist->getBinCount();
    std::vector<Argus::RGBTuple<uint32_t>> rgbHistogram;
    if (Argus::Status::STATUS_OK == irgbhist->getHistogram(&rgbHistogram)) {
      rgbHistogram_ = rgbHistogram;
    } else {
      GST_WARNING("Could not get RGB histogram even though enabled.");
    }
  } else {
    GST_LOG("RGB histogram not available.");
  }

  std::vector<float> sharpnessScore;
  if (Argus::Status::STATUS_OK == imeta->getSharpnessScore(&sharpnessScore)) {
    sharpnessScore_ = sharpnessScore;
  } else {
    GST_LOG("Sharpness score not available.");
  }

  if (toneMapCurveEnabled_) {
    std::vector<float> toneMapCurve;

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_R,
                               &toneMapCurve)) {
      toneMapCurveR_ = toneMapCurve;
    } else {
      GST_WARNING("Could not get R tonemap curve even though enabled.");
    }

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_G,
                               &toneMapCurve)) {
      toneMapCurveG_ = toneMapCurve;
    } else {
      GST_WARNING("Could not get G tonemap curve even though enabled.");
    }

    if (Argus::Status::STATUS_OK ==
        imeta->getToneMapCurve(Argus::RGBChannel::RGB_CHANNEL_B,
                               &toneMapCurve)) {
      toneMapCurveB_ = toneMapCurve;
    } else {
      GST_WARNING("Could not get B tonemap curve even though enabled.");
    }
  } else {
    GST_LOG("Tone map curve not available.");
  }
}

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
