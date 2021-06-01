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

bool producer(int32_t cameraIndex,
              int32_t cameraMode,
              const Size2D<uint32_t>& streamSize,
              int32_t secToRun,
              GstNvManualCameraSrc* src) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_producer_debug,
                          "nvmanualcamerasrc:producer", 0,
                          "nvmanualcamerasrc producer");
  GST_INFO("starting producer thread");
  UniqueObj<CameraProvider> cameraProvider(nullptr);
  ICameraProvider* iCameraProvider = nullptr;
  UniqueObj<CaptureSession> captureSession(nullptr);
  ICaptureSession* iCaptureSession = nullptr;
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

  std::vector<CameraDevice*> cameraDevices;
  std::vector<SensorMode*> modes;

  gfloat frameRate = 0;
  uint32_t index = 0;
  gint found = 0;
  gint best_match = -1;
  Argus::Status err = Argus::Status::STATUS_OK;

  GST_INFO("Creating CameraProvider");
  cameraProvider.reset(CameraProvider::create(&err));
  NONZERO_RETURN_FALSE(err);
  iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
  if (!iCameraProvider) {
    ORIGINATE_ERROR("Failed to create CameraProvider");
  }

  GST_INFO("Getting cameraDevices");
  NONZERO_RETURN_FALSE(iCameraProvider->getCameraDevices(&cameraDevices));
  if (cameraDevices.size() == 0) {
    ORIGINATE_ERROR("No cameras available");
  }

  GST_INFO("Checking cameraIndex is in range");
  if (static_cast<uint32_t>(cameraIndex) >= cameraDevices.size()) {
    ORIGINATE_ERROR(
        "Invalid camera device specified %d specified, %d max index",
        cameraIndex, static_cast<int32_t>(cameraDevices.size()) - 1);
  }

  GST_INFO("Creating CaptureSession");
  captureSession.reset(
      iCameraProvider->createCaptureSession(cameraDevices[cameraIndex], &err));
  NONZERO_RETURN_FALSE(err);
  iCaptureSession = interface_cast<ICaptureSession>(captureSession);
  if (!iCaptureSession)
    ORIGINATE_ERROR("Failed to create CaptureSession");
  // potential footgun here;
  // https://github.com/kareldonk/C-Posters/blob/master/cppcon1.jpg
  // "Object lifetimes man; that shit'll ruin your life"
  src->iCaptureSession_ptr = iCaptureSession;

  GST_INFO("Creating OutputStreamSettings");
  streamSettings.reset(
      iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL, &err));
  NONZERO_RETURN_FALSE(err);
  iStreamSettings = interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (iStreamSettings) {
    GST_INFO("Setting pixelFormat to PIXEL_FMT_YCbCr_420_888");
    NONZERO_RETURN_FALSE(
        iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888));
    GST_INFO("Setting resolution to %d x %d", streamSize.width(),
             streamSize.height());
    NONZERO_RETURN_FALSE(iStreamSettings->setResolution(streamSize));
    if (src->controls.meta_enabled) {
      GST_INFO("Enabling metadata.");
      NONZERO_RETURN_FALSE(iStreamSettings->setMetadataEnable(true));
    } else {
      GST_INFO("Metadata not enabled.");
    }
  }

  GST_INFO("Creating OutputStream");
  outputStream.reset(
      iCaptureSession->createOutputStream(streamSettings.get(), &err));
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
  request.reset(iCaptureSession->createRequest(CAPTURE_INTENT_MANUAL, &err));
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
  iCamProps = interface_cast<ICameraProperties>(cameraDevices[cameraIndex]);
  if (!iCamProps) {
    ORIGINATE_ERROR("Failed to create camera properties");
  }
  NONZERO_RETURN_FALSE(iCamProps->getAllSensorModes(&modes));
  src->total_sensor_modes = modes.size();

  GST_INFO("Checking sensor-mode is valid");
  if (cameraMode != nvmanualcam::defaults::SENSOR_MODE_STATE &&
      static_cast<uint32_t>(cameraMode) >= modes.size()) {
    ORIGINATE_ERROR("Invalid sensor mode %d selected %d present", cameraMode,
                    static_cast<int32_t>(modes.size()));
  }

  GST_INFO("Getting SourceSettings from Request");
  iSourceSettings =
      interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  if (!iSourceSettings) {
    ORIGINATE_ERROR("Failed to get request source settings");
  }
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

  // FIXME(mdegans): move or delete this. it's too "helpful"
  GST_INFO("Available Sensor modes :");
  frameRate = src->info.fps_n / src->info.fps_d;
  src->frame_duration = get_frame_duration(src->info);
  ISensorMode* iSensorMode[modes.size()];
  Range<float> sensorModeAnalogGainRange;
  Range<float> ispDigitalGainRange;
  Range<uint64_t> limitExposureTimeRange;
  for (index = 0; index < modes.size(); index++) {
    iSensorMode[index] = interface_cast<ISensorMode>(modes[index]);
    sensorModeAnalogGainRange = iSensorMode[index]->getAnalogGainRange();
    limitExposureTimeRange = iSensorMode[index]->getExposureTimeRange();
    GST_INFO(
        "%d x %d FR = %f fps Duration = %lu ; Analog Gain range min %f, max "
        "%f; Exposure Range min %ju, max %ju;",
        iSensorMode[index]->getResolution().width(),
        iSensorMode[index]->getResolution().height(),
        (1e9 / (iSensorMode[index]->getFrameDurationRange().min())),
        iSensorMode[index]->getFrameDurationRange().min(),
        sensorModeAnalogGainRange.min(), sensorModeAnalogGainRange.max(),
        limitExposureTimeRange.min(), limitExposureTimeRange.max());

    if (cameraMode == nvmanualcam::defaults::SENSOR_MODE_STATE) {
      if (streamSize.width() <= iSensorMode[index]->getResolution().width() &&
          streamSize.height() <= iSensorMode[index]->getResolution().height() &&
          src->frame_duration >=
              (iSensorMode[index]->getFrameDurationRange().min())) {
        if (best_match == -1 ||
            ((streamSize.width() ==
              iSensorMode[index]->getResolution().width()) &&
             (streamSize.height() ==
              iSensorMode[index]->getResolution().height()) &&
             (iSensorMode[best_match]->getFrameDurationRange().min() >=
              iSensorMode[index]->getFrameDurationRange().min()))) {
          best_match = index;
        } else if ((iSensorMode[index]->getResolution().width()) <=
                   iSensorMode[best_match]->getResolution().width()) {
          best_match = index;
        }
        found = 1;
      }
    }
  }

  if (cameraMode == nvmanualcam::defaults::SENSOR_MODE_STATE) {
    if (0 == found) {
      /* As request resolution is not supported, this is fatal! */
      GST_ERROR("Requested resolution W = %d H = %d not supported by Sensor.",
                streamSize.width(), streamSize.height());
      src->in_error = true;
    } else {
      cameraMode = best_match;
    }
  }

  /* Update Sensor Mode*/
  src->sensor_mode = cameraMode;

  /* Check framerate is supported */
  if (frameRate >
      round((1e9 / (iSensorMode[cameraMode]->getFrameDurationRange().min())))) {
    src->in_error = true;
    GST_ERROR_OBJECT(src,
                     "Frame Rate specified (%d/%d) is greater than supported.",
                     src->info.fps_n, src->info.fps_d);
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
      "   Camera index = %d \n"
      "   Camera mode  = %d \n"
      "   Output Stream W = %d H = %d \n"
      "   seconds to Run    = %d \n"
      "   Frame Rate = %f",
      cameraIndex, cameraMode, iSensorMode[cameraMode]->getResolution().width(),
      iSensorMode[cameraMode]->getResolution().height(), secToRun,
      (1e9 / (iSensorMode[cameraMode]->getFrameDurationRange().min())));

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
    limitExposureTimeRange.min() = src->controls.exposure_real;
    limitExposureTimeRange.max() = src->controls.exposure_real;
    NONZERO_ERROR(
        iSourceSettings->setExposureTimeRange(limitExposureTimeRange));
    src->exposureTimePropSet = false;
  }

  if (src->gainPropSet) {
    sensorModeAnalogGainRange.min() = src->controls.gain;
    sensorModeAnalogGainRange.max() = src->controls.gain;
    NONZERO_ERROR(iSourceSettings->setGainRange(sensorModeAnalogGainRange));
    src->gainPropSet = false;
  }

  if (src->ispDigitalGainPropSet) {
    ispDigitalGainRange.min() = src->controls.digital_gain;
    ispDigitalGainRange.max() = src->controls.digital_gain;
    NONZERO_ERROR(
        iAutoControlSettings->setIspDigitalGainRange(ispDigitalGainRange));
    src->ispDigitalGainPropSet = false;
  }

  NONZERO_WARNING(iSourceSettings->setSensorMode(modes[cameraMode]));

  if (!src->info.fps_n) {
    GST_INFO("Using default frameRate");
    frameRate = defaults::DEFAULT_FPS;
  }

  NONZERO_WARNING(iSourceSettings->setFrameDurationRange(
      Range<uint64_t>(static_cast<uint64_t>(1e9 / frameRate))));

  GST_INFO("Setup Complete, Starting captures for %d seconds", secToRun);

  GST_INFO("Starting repeat capture requests.");
  Request* captureRequest = request.get();
  // FIXME(mdegans): remove this (potential lifetime issue)
  src->request_ptr = captureRequest;
  iCaptureSession->capture(captureRequest);
  if (iCaptureSession->capture(captureRequest) == 0) {
    ORIGINATE_ERROR("Failed to start capture request");
  }

  if (src->in_error) {
    GST_ERROR("InvalidState.");
    NONZERO_ERROR(iCaptureSession->cancelRequests());
    src->timeout = 1;
  } else if (secToRun != 0) {
    sleep(secToRun);
    NONZERO_ERROR(iCaptureSession->cancelRequests());
  } else {
    if (src->stop_requested == FALSE) {
      g_mutex_lock(&src->eos_lock);
      g_cond_wait(&src->eos_cond, &src->eos_lock);
      g_mutex_unlock(&src->eos_lock);
    }
  }

  GST_INFO("Cleaning up");

  iCaptureSession->stopRepeat();
  NONZERO_ERROR(iCaptureSession->waitForIdle());

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
