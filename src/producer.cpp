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

#include "producer.hpp"
#include "consumer.hpp"
#include "gstnvmanualcamera_utils.h"
#include "gstnvmanualcamerasrc.hpp"
#include "logging.hpp"

#include <Argus/Argus.h>
#include <Argus/Ext/BayerSharpnessMap.h>
#include <Error.h>

#include <gst/gst.h>

#include <math.h>
#include <unistd.h>

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_producer_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_producer_debug

using namespace Argus;

namespace nvmanualcam {

bool producer(GstNvManualCameraSrc* src) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_producer_debug,
                          "nvmanualcamerasrc:producer", 0,
                          "nvmanualcamerasrc producer");
  Argus::Status err = Argus::Status::STATUS_OK;

  // Create our camera provier
  static UniqueObj<CameraProvider> provider(CameraProvider::create(&err));
  if (err) {
    ORIGINATE_ERROR("Failed to create CameraProvider.");
  }
  GST_INFO("Checking cameraProvider");
  auto iProvider = interface_cast<ICameraProvider>(provider);
  if (!iProvider) {
    ORIGINATE_ERROR("Failed to get ICameraProvider interface");
  }

  UniqueObj<CaptureSession> session(nullptr);
  ICaptureSession* iSession = nullptr;
  UniqueObj<OutputStreamSettings> streamSettings(nullptr);
  IEGLOutputStreamSettings* iStreamSettings = nullptr;
  UniqueObj<OutputStream> outputStream(nullptr);
  IEGLOutputStream* iStream = nullptr;
  UniqueObj<Request> request(nullptr);
  IRequest* iRequest = nullptr;                          // interface of Request
  IAutoControlSettings* iAutoControlSettings = nullptr;  // interface of Request
  ISourceSettings* iSourceSettings = nullptr;            // interface of Request
  IDenoiseSettings* iDenoiseSettings = nullptr;          // interface of Request
  IEdgeEnhanceSettings* iEeSettings = nullptr;           // interface of Request
  ICameraProperties* iCamProps = nullptr;  // interface of CameraDevice

  std::vector<CameraDevice*> devices;
  std::vector<SensorMode*> modes;

  GST_INFO("Getting CameraDevices");
  NONZERO_RETURN_FALSE(iProvider->getCameraDevices(&devices));
  if (devices.size() == 0) {
    ORIGINATE_ERROR("No cameras available");
  }

  GST_INFO("Checking cameraIndex is in range");
  src->total_sensor_modes = devices.size();
  if (src->sensor_id >= devices.size()) {
    ORIGINATE_ERROR(
        "Invalid camera device specified %zu specified, %zu max index",
        src->sensor_id, src->total_sensor_modes - 1);
  }

  GST_INFO("Creating CaptureSession");
  session.reset(iProvider->createCaptureSession(devices[src->sensor_id], &err));
  NONZERO_RETURN_FALSE(err);
  iSession = interface_cast<ICaptureSession>(session);
  if (!iSession)
    ORIGINATE_ERROR("Failed to create CaptureSession");
  // potential footgun here;
  // https://github.com/kareldonk/C-Posters/blob/master/cppcon1.jpg
  // "Object lifetimes man; that shit'll ruin your life"
  src->iCaptureSession_ptr = iSession;

  GST_INFO("Creating OutputStreamSettings");
  streamSettings.reset(
      iSession->createOutputStreamSettings(STREAM_TYPE_EGL, &err));
  NONZERO_RETURN_FALSE(err);
  iStreamSettings = interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (iStreamSettings) {
    GST_INFO("Setting pixelFormat to PIXEL_FMT_YCbCr_420_888");
    NONZERO_RETURN_FALSE(
        iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888));
    Argus::Size2D<uint32_t> stream_size(src->info.width, src->info.height);
    GST_INFO("Setting resolution to %d x %d", stream_size.width(),
             stream_size.height());
    NONZERO_RETURN_FALSE(iStreamSettings->setResolution(stream_size));
    if (src->controls.meta_enabled) {
      GST_INFO("Enabling metadata.");
      NONZERO_RETURN_FALSE(iStreamSettings->setMetadataEnable(true));
    } else {
      GST_INFO("Metadata not enabled.");
    }
  }

  GST_INFO("Creating OutputStream");
  outputStream.reset(iSession->createOutputStream(streamSettings.get(), &err));
  NONZERO_RETURN_FALSE(err);
  iStream = interface_cast<IEGLOutputStream>(outputStream);
  if (!iStream) {
    ORIGINATE_ERROR("Failed to create OutputStream");
  }

  GST_INFO("Setting up Consumer");
  utils::Consumer consumerThread(outputStream.get());
  PROPAGATE_ERROR(consumerThread.initialize(src));
  PROPAGATE_ERROR(consumerThread.waitRunning());

  GST_INFO("Creating Request");
  request.reset(iSession->createRequest(CAPTURE_INTENT_MANUAL, &err));
  NONZERO_RETURN_FALSE(err);
  iRequest = interface_cast<IRequest>(request);
  src->iRequest_ptr = iRequest;
  if (!iRequest) {
    ORIGINATE_ERROR("Failed to create Request");
  }

  GST_INFO("Enabling OutputStream on Request");
  NONZERO_RETURN_FALSE(iRequest->enableOutputStream(outputStream.get()));

  GST_INFO("Resurrecting ancient evil, iAutoControlSettings");
  iAutoControlSettings =
      interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
  // FIXME(mdegans): remove this (potential lifetime issue)
  src->iAutoControlSettings_ptr = iAutoControlSettings;

  GST_INFO("Getting CameraProperties");
  iCamProps = interface_cast<ICameraProperties>(devices[src->sensor_id]);
  if (!iCamProps) {
    ORIGINATE_ERROR("Failed to create camera properties");
  }

  NONZERO_RETURN_FALSE(iCamProps->getAllSensorModes(&modes));
  src->total_sensor_modes = modes.size();
  GST_INFO("Checking sensor-mode is valid");
  if (src->sensor_mode >= modes.size()) {
    ORIGINATE_ERROR("Invalid sensor mode %zu selected %zu present",
                    src->sensor_mode, modes.size());
  }

  // Get sensor mode we will use, and it's interface
  auto mode = modes[src->sensor_mode];
  auto iMode = interface_cast<ISensorMode>(mode);

  // get the resolution and supported frame duration range
  auto resolution = iMode->getResolution();
  auto f_duration = iMode->getFrameDurationRange();

  // update info, so caps are fixated properly
  src->info.width = resolution.width();
  src->info.height = resolution.height();
  src->info.fps_n = round(1000000000 / f_duration.min());

  GST_INFO("Getting SourceSettings from Request");
  iSourceSettings =
      interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  if (!iSourceSettings) {
    ORIGINATE_ERROR("Failed to get request source settings");
  }

  // set the selected mode
  iSourceSettings->setSensorMode(mode);

  // set best supported frame rate
  NONZERO_WARNING(iSourceSettings->setFrameDurationRange(
      Range<uint64_t>(f_duration.min())));

  // FIXME(mdegans): remove this (potential lifetime issue)
  src->iRequestSourceSettings_ptr = iSourceSettings;

  if (src->controls.bayer_sharpness_map) {
    if (!src->controls.meta_enabled) {
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

  iDenoiseSettings = interface_cast<IDenoiseSettings>(request);
  if (!iDenoiseSettings)
    ORIGINATE_ERROR("Failed to get DenoiseSettings interface");
  // FIXME(mdegans): remove this (potential lifetime issue)
  src->iDenoiseSettings_ptr = iDenoiseSettings;

  iEeSettings = interface_cast<IEdgeEnhanceSettings>(request);
  if (!iEeSettings)
    ORIGINATE_ERROR("Failed to get EdgeEnhancementSettings interface");
  // FIXME(mdegans): remove this (potential lifetime issue)
  src->iEeSettings_ptr = iEeSettings;

  if (src->tnrModePropSet) {
    switch (src->controls.NoiseReductionMode) {
      case NvManualCamNoiseReductionMode_Off:
        NONZERO_ERROR(iDenoiseSettings->setDenoiseMode(DENOISE_MODE_OFF));
        break;
      case NvManualCamNoiseReductionMode_Fast:
        NONZERO_ERROR(iDenoiseSettings->setDenoiseMode(DENOISE_MODE_FAST));
        break;
      case NvManualCamNoiseReductionMode_HighQuality:
        NONZERO_ERROR(
            iDenoiseSettings->setDenoiseMode(DENOISE_MODE_HIGH_QUALITY));
        break;
      default:
        GST_ERROR("src->controls.NoiseReductionMode invalid");
        break;
    }
    src->tnrModePropSet = FALSE;
  }

  if (src->tnrStrengthPropSet) {
    NONZERO_ERROR(iDenoiseSettings->setDenoiseStrength(
        src->controls.NoiseReductionStrength));
    src->tnrStrengthPropSet = FALSE;
  }

  if (src->edgeEnhancementModePropSet) {
    switch (src->controls.EdgeEnhancementMode) {
      case NvManualCamEdgeEnhancementMode_Off:
        NONZERO_ERROR(iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF));
        break;
      case NvManualCamEdgeEnhancementMode_Fast:
        NONZERO_ERROR(iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST));
        break;
      case NvManualCamEdgeEnhancementMode_HighQuality:
        NONZERO_ERROR(
            iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY));
        break;
      default:
        GST_ERROR("src->controls.EdgeEnhancementMode invalid");
        break;
    }
    src->edgeEnhancementModePropSet = FALSE;
  }

  if (src->edgeEnhancementStrengthPropSet) {
    NONZERO_ERROR(iEeSettings->setEdgeEnhanceStrength(
        src->controls.EdgeEnhancementStrength));
    src->edgeEnhancementStrengthPropSet = FALSE;
  }

  if (src->aeAntibandingPropSet) {
    switch (src->controls.AeAntibandingMode) {
      case NvManualCamAeAntibandingMode_Off:
        NONZERO_ERROR(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_OFF));
        break;
      case NvManualCamAeAntibandingMode_Auto:
        NONZERO_ERROR(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_AUTO));
        break;
      case NvManualCamAeAntibandingMode_50HZ:
        NONZERO_ERROR(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_50HZ));
        break;
      case NvManualCamAeAntibandingMode_60HZ:
        NONZERO_ERROR(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_60HZ));
        break;
      default:
        GST_ERROR("src->controls.AeAntibandingMode invalid");
        break;
    }
    src->aeAntibandingPropSet = FALSE;
  }

  if (src->exposureCompensationPropSet) {
    NONZERO_ERROR(iAutoControlSettings->setExposureCompensation(
        src->controls.ExposureCompensation));
    src->exposureCompensationPropSet = FALSE;
  }

  if (src->awbLockPropSet) {
    if (src->controls.AwbLock) {
      NONZERO_ERROR(iAutoControlSettings->setAwbLock(true));
    } else {
      NONZERO_ERROR(iAutoControlSettings->setAwbLock(false));
    }
    src->awbLockPropSet = FALSE;
  }

  if (src->aeLockPropSet) {
    if (src->controls.AeLock)
      NONZERO_ERROR(iAutoControlSettings->setAeLock(true));
    else
      NONZERO_ERROR(iAutoControlSettings->setAeLock(false));
    src->aeLockPropSet = FALSE;
  }

  GST_INFO(
      "Running with following settings:\n"
      "   Camera index = %zu \n"
      "   Camera mode = %zu \n"
      "   Output Stream W = %d H = %d \n"
      "   seconds to Run = %d \n"
      "   Frame Rate = %d",
      src->sensor_id, src->sensor_mode, resolution.width(), resolution.height(),
      src->timeout, src->info.fps_n);

  if (src->wbPropSet) {
    switch (src->controls.wbmode) {
      case NvManualCamAwbMode_Off:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_OFF));
        break;
      case NvManualCamAwbMode_Auto:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_AUTO));
        break;
      case NvManualCamAwbMode_Incandescent:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_INCANDESCENT));
        break;
      case NvManualCamAwbMode_Fluorescent:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_FLUORESCENT));
        break;
      case NvManualCamAwbMode_WarmFluorescent:
        NONZERO_ERROR(
            iAutoControlSettings->setAwbMode(AWB_MODE_WARM_FLUORESCENT));
        break;
      case NvManualCamAwbMode_Daylight:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT));
        break;
      case NvManualCamAwbMode_CloudyDaylight:
        NONZERO_ERROR(
            iAutoControlSettings->setAwbMode(AWB_MODE_CLOUDY_DAYLIGHT));
        break;
      case NvManualCamAwbMode_Twilight:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_TWILIGHT));
        break;
      case NvManualCamAwbMode_Shade:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_SHADE));
        break;
      case NvManualCamAwbMode_Manual:
        NONZERO_ERROR(iAutoControlSettings->setAwbMode(AWB_MODE_MANUAL));
        break;
      default:
        GST_ERROR("src->controls.wbmode invalid");
        break;
    }
    src->wbPropSet = FALSE;
  }

  if (src->saturationPropSet) {
    NONZERO_ERROR(iAutoControlSettings->setColorSaturationEnable(TRUE));
    NONZERO_ERROR(
        iAutoControlSettings->setColorSaturation(src->controls.saturation));
    src->saturationPropSet = false;
  }

  if (src->exposureTimePropSet) {
    f_duration.min() = src->controls.exposure_real;
    f_duration.max() = src->controls.exposure_real;
    NONZERO_ERROR(iSourceSettings->setExposureTimeRange(f_duration));
    src->exposureTimePropSet = false;
  }

  if (src->gainPropSet) {
    Argus::Range<float> gain_range(0);
    gain_range.min() = src->controls.gain;
    gain_range.max() = src->controls.gain;
    NONZERO_ERROR(iSourceSettings->setGainRange(gain_range));
    src->gainPropSet = false;
  }

  if (src->ispDigitalGainPropSet) {
    Argus::Range<float> digital_gain(0);
    digital_gain.min() = src->controls.digital_gain;
    digital_gain.max() = src->controls.digital_gain;
    NONZERO_ERROR(iAutoControlSettings->setIspDigitalGainRange(digital_gain));
    src->ispDigitalGainPropSet = false;
  }

  GST_INFO("Setup Complete, Starting captures for %d seconds", src->timeout);

  GST_INFO("Starting repeat capture requests.");
  Request* captureRequest = request.get();
  // FIXME(mdegans): remove this (potential lifetime issue)
  src->request_ptr = captureRequest;
  iSession->capture(captureRequest);
  if (iSession->capture(captureRequest) == 0) {
    ORIGINATE_ERROR("Failed to start capture request");
  }

  if (src->in_error) {
    GST_ERROR("InvalidState.");
    NONZERO_ERROR(iSession->cancelRequests());
    src->timeout = 1;
  } else if (src->timeout != 0) {
    sleep(src->timeout);
    NONZERO_ERROR(iSession->cancelRequests());
  } else {
    if (src->stop_requested == FALSE) {
      g_mutex_lock(&src->eos_lock);
      g_cond_wait(&src->eos_cond, &src->eos_lock);
      g_mutex_unlock(&src->eos_lock);
    }
  }

  GST_INFO("Cleaning up");

  iSession->stopRepeat();
  NONZERO_ERROR(iSession->waitForIdle());

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

  if (src->in_error)
    return false;

  GST_INFO("Done Success");

  return true;
}

}  // namespace nvmanualcam
