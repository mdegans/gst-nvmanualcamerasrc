/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Argus/Argus.h>
#include <Argus/Ext/BayerSharpnessMap.h>
#include <Error.h>

#include <gst/gst.h>

#include <math.h>
#include <unistd.h>

#include "consumer.hpp"
#include "gstnvmanualcamera_utils.h"
#include "impl_data.hpp"
#include "logging.hpp"
#include "producer.hpp"

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_producer_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_producer_debug

using namespace Argus;

namespace nvmanualcam {

bool producer(int32_t cameraIndex,
              int32_t cameraMode,
              const Size2D<uint32_t>& streamSize,
              int32_t secToRun,
              std::shared_ptr<ArgusControls> ctrl) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_producer_debug,
                          "nvmanualcamerasrc:producer", 0,
                          "nvmanualcamerasrc producer");
  GST_INFO("starting producer thread");
  gfloat frameRate = 0;
  uint32_t index = 0;
  gint found = 0;
  gint best_match = -1;

  auto a_ctx = ArgusCtx::create(streamSize, ctrl, cameraIndex);

  if (ctrl->getBayerSharpnessMap()) {
    if (!ctrl->getMetaEnabled()) {
      ORIGINATE_ERROR("`metadata` must be true to use `bayer-sharpness-map`");
    }
    GST_INFO("Enabling BayerSharpnessMap metadata.");
    auto sharpMapSettings =
        interface_cast<Ext::IBayerSharpnessMapSettings>(request);
    if (!sharpMapSettings) {
      ORIGINATE_ERROR(
          "Failed to get Ext::IBayerSharpnessMapSettings interface.");
    }
    sharpMapSettings->setBayerSharpnessMapEnable(true);
  }

  if (cameraMode != nvmanualcam::defaults::SENSOR_MODE_STATE &&
      static_cast<uint32_t>(cameraMode) >= modes.size())
    ORIGINATE_ERROR("Invalid sensor mode %d selected %d present", cameraMode,
                    static_cast<int32_t>(modes.size()));

  src->total_sensor_modes = modes.size();

  GST_INFO("Available Sensor modes :");
  frameRate = src->info.fps_n / src->info.fps_d;
  src->frame_duration = get_frame_duration(src->info);
  ISensorMode* iSensorMode[modes.size()];
  Range<float> sensorModeAnalogGainRange;
  Range<float> ispDigitalGainRange;
  Range<uint64_t> limitExposureTimeRange;

  if (cameraMode == nvmanualcam::defaults::SENSOR_MODE_STATE) {
    if (0 == found) {
      /* As request resolution is not supported, switch to default
       * sensormode Index.
       */
      GST_INFO("Requested resolution W = %d H = %d not supported by Sensor.",
               streamSize.width(), streamSize.height());
      cameraMode = 0;
    } else {
      cameraMode = best_match;
    }
  }
  /* Update Sensor Mode*/
  src->sensor_mode = cameraMode;

  if (frameRate >
      round((1e9 / (iSensorMode[cameraMode]->getFrameDurationRange().min())))) {
    src->in_error = TRUE;
    GST_ERROR_OBJECT(src,
                     "Frame Rate specified (%d/%d) is greater than supported.",
                     src->info.fps_n, src->info.fps_d);
  }

  IDenoiseSettings* denoiseSettings = interface_cast<IDenoiseSettings>(request);
  if (!denoiseSettings)
    ORIGINATE_ERROR("Failed to get DenoiseSettings interface");
  src->iDenoiseSettings_ptr = denoiseSettings;

  IEdgeEnhanceSettings* eeSettings =
      interface_cast<IEdgeEnhanceSettings>(request);
  if (!eeSettings)
    ORIGINATE_ERROR("Failed to get EdgeEnhancementSettings interface");
  src->iEeSettings_ptr = eeSettings;

  /* Setting Noise Reduction Mode and Strength*/
  if (ctrl->isUpdated(Properties::ID::TNR_MODE)) {
    switch (ctrl->getTnrMode()) {
      case NvManualCamNoiseReductionMode_Off:
        denoiseSettings->setDenoiseMode(DENOISE_MODE_OFF);
        break;
      case NvManualCamNoiseReductionMode_Fast:
        denoiseSettings->setDenoiseMode(DENOISE_MODE_FAST);
        break;
      case NvManualCamNoiseReductionMode_HighQuality:
        denoiseSettings->setDenoiseMode(DENOISE_MODE_HIGH_QUALITY);
        break;
      default:
        denoiseSettings->setDenoiseMode(DENOISE_MODE_OFF);
        break;
    }
  }

  if (ctrl->isUpdated(Properties::ID::TNR_STRENGTH)) {
    denoiseSettings->setDenoiseStrength(ctrl->getTnrStrength());
  }

  if (ctrl->isUpdated(Properties::ID::EE_MODE)) {
    switch (ctrl->getEeMode()) {
      case NvManualCamEdgeEnhancementMode_Off:
        eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF);
        break;
      case NvManualCamEdgeEnhancementMode_Fast:
        eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST);
        break;
      case NvManualCamEdgeEnhancementMode_HighQuality:
        eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY);
        break;
      default:
        eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF);
        break;
    }
  }

  if (ctrl->isUpdated(Properties::ID::EE_STRENGTH)) {
    eeSettings->setEdgeEnhanceStrength(ctrl->getEeStrength());
  }

  if (ctrl->isUpdated(Properties::ID::AEANTIBANDING_MODE)) {
    switch (ctrl->getAeantibandingMode()) {
      case NvManualCamAeAntibandingMode_Off:
        iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF);
        break;
      case NvManualCamAeAntibandingMode_Auto:
        iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_AUTO);
        break;
      case NvManualCamAeAntibandingMode_50HZ:
        iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_50HZ);
        break;
      case NvManualCamAeAntibandingMode_60HZ:
        iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_60HZ);
        break;
      default:
        iAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF);
        break;
    }
  }

  if (ctrl->isUpdated(Properties::ID::EXP_COMPENSATION)) {
    iAutoControlSettings->setExposureCompensation(ctrl->getExpCompensation());
  }

  /* Setting auto white balance lock */
  if (ctrl->isUpdated(Properties::ID::AWB_LOCK)) {
    iAutoControlSettings->setAwbLock(ctrl->getAwbLock());
  }

  /* Setting auto exposure lock */
  if (ctrl->isUpdated(Properties::ID::AE_LOCK)) {
    iAutoControlSettings->setAeLock(ctrl->getAeLock());
  }

  GST_INFO(
      "Running with following settings:\n"
      "   Camera index = %d \n"
      "   Camera mode  = %d \n"
      "   Output Stream W = %d H = %d \n"
      "   seconds to Run    = %d \n"
      "   Frame Rate = %f",
      cameraIndex, cameraMode, iSensorMode[cameraMode]->getResolution().width(),
      iSensorMode[cameraMode]->getResolution().height(), secToRun,
      (1e9 / (iSensorMode[cameraMode]->getFrameDurationRange().min())));

  /* Setting white balance property */
  if (ctrl->isUpdated(Properties::ID::WHITE_BALANCE)) {
    switch (ctrl->getWbmode()) {
      case NvManualCamAwbMode_Off:
        iAutoControlSettings->setAwbMode(AWB_MODE_OFF);
        break;
      case NvManualCamAwbMode_Auto:
        iAutoControlSettings->setAwbMode(AWB_MODE_AUTO);
        break;
      case NvManualCamAwbMode_Incandescent:
        iAutoControlSettings->setAwbMode(AWB_MODE_INCANDESCENT);
        break;
      case NvManualCamAwbMode_Fluorescent:
        iAutoControlSettings->setAwbMode(AWB_MODE_FLUORESCENT);
        break;
      case NvManualCamAwbMode_WarmFluorescent:
        iAutoControlSettings->setAwbMode(AWB_MODE_WARM_FLUORESCENT);
        break;
      case NvManualCamAwbMode_Daylight:
        iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT);
        break;
      case NvManualCamAwbMode_CloudyDaylight:
        iAutoControlSettings->setAwbMode(AWB_MODE_CLOUDY_DAYLIGHT);
        break;
      case NvManualCamAwbMode_Twilight:
        iAutoControlSettings->setAwbMode(AWB_MODE_TWILIGHT);
        break;
      case NvManualCamAwbMode_Shade:
        iAutoControlSettings->setAwbMode(AWB_MODE_SHADE);
        break;
      case NvManualCamAwbMode_Manual:
        iAutoControlSettings->setAwbMode(AWB_MODE_MANUAL);
        break;
      default:
        iAutoControlSettings->setAwbMode(AWB_MODE_OFF);
        break;
    }
  }

  /* Setting color saturation property */
  if (ctrl->isUpdated(Properties::ID::SATURATION)) {
    iAutoControlSettings->setColorSaturationEnable(true);
    iAutoControlSettings->setColorSaturation(ctrl->getSaturation());
  }

  if (ctrl->isUpdated(Properties::ID::EXPOSURE_TIME)) {
    uint64_t exp_time = ctrl->getExposureReal();
    limitExposureTimeRange.min() = exp_time;
    limitExposureTimeRange.max() = exp_time;
    requestSourceSettings->setExposureTimeRange(limitExposureTimeRange);
  }

  if (ctrl->isUpdated(Properties::ID::GAIN)) {
    float gain = ctrl->getGain();
    sensorModeAnalogGainRange.min() = gain;
    sensorModeAnalogGainRange.max() = gain;
    requestSourceSettings->setGainRange(sensorModeAnalogGainRange);
  }

  if (ctrl->isUpdated(Properties::ID::DIGITAL_GAIN)) {
    float digital_gain = ctrl->getDigitalGain();
    ispDigitalGainRange.min() = digital_gain;
    ispDigitalGainRange.max() = digital_gain;
    iAutoControlSettings->setIspDigitalGainRange(ispDigitalGainRange);
  }

  requestSourceSettings->setSensorMode(modes[cameraMode]);
  if (!ctrl->info().fps_n) {
    frameRate = defaults::DEFAULT_FPS;
  }

  requestSourceSettings->setFrameDurationRange(
      Range<uint64_t>(1e9 / frameRate));

  GST_INFO("Setup Complete, Starting captures for %d seconds", secToRun);

  GST_INFO("Starting repeat capture requests.");
  Request* captureRequest = request.get();
  // src->request_ptr = captureRequest;
  iCaptureSession->capture(captureRequest);
  if (iCaptureSession->capture(captureRequest) == 0) {
    ORIGINATE_ERROR("Failed to start capture request");
  }

  if (src->in_error) {
    GST_ERROR("InvalidState.");
    iCaptureSession->cancelRequests();
    src->timeout = 1;
  } else if (secToRun != 0) {
    sleep(secToRun);
    iCaptureSession->cancelRequests();
  } else {
    if (src->stop_requested == false) {
      g_mutex_lock(&src->eos_lock);
      g_cond_wait(&src->eos_cond, &src->eos_lock);
      g_mutex_unlock(&src->eos_lock);
    }
  }

  GST_INFO("Cleaning up");

  iCaptureSession->stopRepeat();
  iCaptureSession->waitForIdle();

  // Destroy the output stream. This destroys the EGLStream which causes
  // the GL consumer acquire to fail and the consumer thread to end.
  outputStream.reset();

  // Manual execution completed, signal the buffer consumed cond.
  if (!src->is_manual_buffer_consumed) {
    g_mutex_lock(&src->manual_buffer_consumed_lock);
    g_cond_signal(&src->manual_buffer_consumed_cond);
    src->is_manual_buffer_consumed = TRUE;
    g_mutex_unlock(&src->manual_buffer_consumed_lock);
  }

  // Wait for the consumer thread to complete.
  PROPAGATE_ERROR(consumerThread.shutdown());

  if (src->in_error) {
    return false;
  }

  GST_INFO("Done Success");
  return true;
}

}  // namespace nvmanualcam
