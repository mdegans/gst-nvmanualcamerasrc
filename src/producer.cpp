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

// https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api
#define argusCheck(code) \
  { argusAssert((code), __FILE__, __LINE__); }
inline void argusAssert(Argus::Status code, const char* file, int line) {
  if (code != Argus::Status::STATUS_OK) {
    GST_ERROR("Argus Error: %d %s:%d\n", code, file, line);
  }
}

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
  gfloat frameRate = 0;
  uint32_t index = 0;
  gint found = 0;
  gint best_match = -1;
  Argus::Status err = Argus::Status::STATUS_OK;

  // Create the CameraProvider object
  static UniqueObj<CameraProvider> cameraProvider(CameraProvider::create());
  ICameraProvider* iCameraProvider =
      interface_cast<ICameraProvider>(cameraProvider);
  if (!iCameraProvider)
    ORIGINATE_ERROR("Failed to create CameraProvider");

  // Get the camera devices.
  std::vector<CameraDevice*> cameraDevices;
  iCameraProvider->getCameraDevices(&cameraDevices);
  if (cameraDevices.size() == 0)
    ORIGINATE_ERROR("No cameras available");

  if (static_cast<uint32_t>(cameraIndex) >= cameraDevices.size())
    ORIGINATE_ERROR(
        "Invalid camera device specified %d specified, %d max index",
        cameraIndex, static_cast<int32_t>(cameraDevices.size()) - 1);

  // Create the capture session using the specified device.
  UniqueObj<CaptureSession> captureSession(
      iCameraProvider->createCaptureSession(cameraDevices[cameraIndex], &err));
  argusCheck(err);
  ICaptureSession* iCaptureSession =
      interface_cast<ICaptureSession>(captureSession);
  if (!iCaptureSession)
    ORIGINATE_ERROR("Failed to create CaptureSession");

  src->iCaptureSession_ptr = iCaptureSession;
  GST_INFO("Creating output stream");
  UniqueObj<OutputStreamSettings> streamSettings(
      iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL, &err));
  argusCheck(err);
  IEGLOutputStreamSettings* iStreamSettings =
      interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (iStreamSettings) {
    argusCheck(iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888));
    argusCheck(iStreamSettings->setResolution(streamSize));
    if (src->controls.meta_enabled) {
      GST_INFO("Enabling metadata.");
      argusCheck(iStreamSettings->setMetadataEnable(true));
    } else {
      GST_INFO("Metadata not enabled.");
    }
  }
  UniqueObj<OutputStream> outputStream(
      iCaptureSession->createOutputStream(streamSettings.get(), &err));
  argusCheck(err);
  IEGLOutputStream* iStream = interface_cast<IEGLOutputStream>(outputStream);
  if (!iStream)
    ORIGINATE_ERROR("Failed to create OutputStream");

  utils::Consumer consumerThread(outputStream.get());
  PROPAGATE_ERROR(consumerThread.initialize(src));

  // Wait until the consumer is connected to the stream.
  PROPAGATE_ERROR(consumerThread.waitRunning());

  // Create capture request and enable output stream.
  UniqueObj<Request> request(
      iCaptureSession->createRequest(CAPTURE_INTENT_MANUAL, &err));
  argusCheck(err);
  IRequest* iRequest = interface_cast<IRequest>(request);
  src->iRequest_ptr = iRequest;
  if (!iRequest)
    ORIGINATE_ERROR("Failed to create Request");
  argusCheck(iRequest->enableOutputStream(outputStream.get()));

  IAutoControlSettings* iAutoControlSettings =
      interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
  src->iAutoControlSettings_ptr = iAutoControlSettings;
  std::vector<SensorMode*> modes;
  ICameraProperties* camProps =
      interface_cast<ICameraProperties>(cameraDevices[cameraIndex]);
  if (!camProps)
    ORIGINATE_ERROR("Failed to create camera properties");
  argusCheck(camProps->getAllSensorModes(&modes));

  ISourceSettings* requestSourceSettings =
      interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  if (!requestSourceSettings)
    ORIGINATE_ERROR("Failed to get request source settings");
  src->iRequestSourceSettings_ptr = requestSourceSettings;

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
  if (src->tnrModePropSet) {
    switch (src->controls.NoiseReductionMode) {
      case NvManualCamNoiseReductionMode_Off:
        argusCheck(denoiseSettings->setDenoiseMode(DENOISE_MODE_OFF));
        break;
      case NvManualCamNoiseReductionMode_Fast:
        argusCheck(denoiseSettings->setDenoiseMode(DENOISE_MODE_FAST));
        break;
      case NvManualCamNoiseReductionMode_HighQuality:
        argusCheck(denoiseSettings->setDenoiseMode(DENOISE_MODE_HIGH_QUALITY));
        break;
      default:
        GST_ERROR("src->controls.NoiseReductionMode invalid");
        break;
    }
    src->tnrModePropSet = FALSE;
  }

  if (src->tnrStrengthPropSet) {
    denoiseSettings->setDenoiseStrength(src->controls.NoiseReductionStrength);
    src->tnrStrengthPropSet = FALSE;
  }

  /* Setting Edge Enhancement Mode and Strength*/
  if (src->edgeEnhancementModePropSet) {
    switch (src->controls.EdgeEnhancementMode) {
      case NvManualCamEdgeEnhancementMode_Off:
        argusCheck(eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF));
        break;
      case NvManualCamEdgeEnhancementMode_Fast:
        argusCheck(eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST));
        break;
      case NvManualCamEdgeEnhancementMode_HighQuality:
        argusCheck(
            eeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY));
        break;
      default:
        GST_ERROR("src->controls.EdgeEnhancementMode invalid");
        break;
    }
    src->edgeEnhancementModePropSet = FALSE;
  }

  if (src->edgeEnhancementStrengthPropSet) {
    argusCheck(eeSettings->setEdgeEnhanceStrength(
        src->controls.EdgeEnhancementStrength));
    src->edgeEnhancementStrengthPropSet = FALSE;
  }

  /* Setting AE Antibanding Mode */
  if (src->aeAntibandingPropSet) {
    switch (src->controls.AeAntibandingMode) {
      case NvManualCamAeAntibandingMode_Off:
        argusCheck(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_OFF));
        break;
      case NvManualCamAeAntibandingMode_Auto:
        argusCheck(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_AUTO));
        break;
      case NvManualCamAeAntibandingMode_50HZ:
        argusCheck(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_50HZ));
        break;
      case NvManualCamAeAntibandingMode_60HZ:
        argusCheck(iAutoControlSettings->setAeAntibandingMode(
            AE_ANTIBANDING_MODE_60HZ));
        break;
      default:
        GST_ERROR("src->controls.AeAntibandingMode invalid");
        break;
    }
    src->aeAntibandingPropSet = FALSE;
  }

  if (src->exposureCompensationPropSet) {
    err = iAutoControlSettings->setExposureCompensation(
        src->controls.ExposureCompensation);
    if (err) {
      GST_WARNING_OBJECT(
          src, "unable to set exposure compensation to %ld (status %d)",
          src->controls.exposure_real, err);
    }
    src->exposureCompensationPropSet = FALSE;
  }

  /* Setting auto white balance lock */
  if (src->awbLockPropSet) {
    if (src->controls.AwbLock) {
      argusCheck(iAutoControlSettings->setAwbLock(true));
    } else {
      argusCheck(iAutoControlSettings->setAwbLock(false));
    }
    src->awbLockPropSet = FALSE;
  }

  /* Setting auto exposure lock */
  if (src->aeLockPropSet) {
    if (src->controls.AeLock)
      err = iAutoControlSettings->setAeLock(true);
    else
      err = iAutoControlSettings->setAeLock(false);
    if (err) {
      GST_WARNING_OBJECT(src, "unable to set ae lock (status %d)", err);
    }
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

  /* Setting white balance property */
  if (src->wbPropSet) {
    switch (src->controls.wbmode) {
      case NvManualCamAwbMode_Off:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_OFF));
        break;
      case NvManualCamAwbMode_Auto:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_AUTO));
        break;
      case NvManualCamAwbMode_Incandescent:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_INCANDESCENT));
        break;
      case NvManualCamAwbMode_Fluorescent:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_FLUORESCENT));
        break;
      case NvManualCamAwbMode_WarmFluorescent:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_WARM_FLUORESCENT));
        break;
      case NvManualCamAwbMode_Daylight:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT));
        break;
      case NvManualCamAwbMode_CloudyDaylight:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_CLOUDY_DAYLIGHT));
        break;
      case NvManualCamAwbMode_Twilight:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_TWILIGHT));
        break;
      case NvManualCamAwbMode_Shade:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_SHADE));
        break;
      case NvManualCamAwbMode_Manual:
        argusCheck(iAutoControlSettings->setAwbMode(AWB_MODE_MANUAL));
        break;
      default:
        GST_ERROR("src->controls.wbmode invalid");
        break;
    }
    src->wbPropSet = FALSE;
  }

  /* Setting color saturation property */
  if (src->saturationPropSet) {
    argusCheck(iAutoControlSettings->setColorSaturationEnable(TRUE));
    argusCheck(
        iAutoControlSettings->setColorSaturation(src->controls.saturation));
    src->saturationPropSet = false;
  }

  if (src->exposureTimePropSet) {
    limitExposureTimeRange.min() = src->controls.exposure_real;
    limitExposureTimeRange.max() = src->controls.exposure_real;
    err = requestSourceSettings->setExposureTimeRange(limitExposureTimeRange);
    if (err) {
      GST_WARNING_OBJECT(src, "unable to set exposure time to %ld (status %d)",
                         src->controls.exposure_real, err);
    }
    src->exposureTimePropSet = false;
  }

  if (src->gainPropSet) {
    sensorModeAnalogGainRange.min() = src->controls.gain;
    sensorModeAnalogGainRange.max() = src->controls.gain;
    err = requestSourceSettings->setGainRange(sensorModeAnalogGainRange);
    if (err) {
      GST_WARNING_OBJECT(src, "unable to gain to %.4f (status %d)",
                         src->controls.gain, err);
    }
    src->gainPropSet = false;
  }

  if (src->ispDigitalGainPropSet) {
    ispDigitalGainRange.min() = src->controls.digital_gain;
    ispDigitalGainRange.max() = src->controls.digital_gain;
    err = iAutoControlSettings->setIspDigitalGainRange(ispDigitalGainRange);
    if (err) {
      GST_WARNING_OBJECT(src, "unable to digital gain to %.4f (status %d)",
                         src->controls.gain, err);
    }
    src->ispDigitalGainPropSet = false;
  }

  err = requestSourceSettings->setSensorMode(modes[cameraMode]);
  if (err) {
    GST_WARNING_OBJECT(src, "unable to sensor mode to %d (status %d)",
                       cameraMode, err);
  }
  if (!src->info.fps_n) {
    frameRate = defaults::DEFAULT_FPS;
  }

  err = requestSourceSettings->setFrameDurationRange(
      Range<uint64_t>(static_cast<uint64_t>(1e9 / frameRate)));
  if (err != Argus::Status::STATUS_OK) {
    GST_WARNING_OBJECT(src, "unable to frame duration to %ld (status %d)",
                       static_cast<uint64_t>(1e9 / frameRate), err);
  }

  GST_INFO("Setup Complete, Starting captures for %d seconds", secToRun);

  GST_INFO("Starting repeat capture requests.");
  Request* captureRequest = request.get();
  src->request_ptr = captureRequest;
  iCaptureSession->capture(captureRequest);
  if (iCaptureSession->capture(captureRequest) == 0)
    ORIGINATE_ERROR("Failed to start capture request");

  if (src->in_error) {
    GST_ERROR("InvalidState.");
    argusCheck(iCaptureSession->cancelRequests());
    src->timeout = 1;
  } else if (secToRun != 0) {
    sleep(secToRun);
    argusCheck(iCaptureSession->cancelRequests());
  } else {
    if (src->stop_requested == FALSE) {
      g_mutex_lock(&src->eos_lock);
      g_cond_wait(&src->eos_cond, &src->eos_lock);
      g_mutex_unlock(&src->eos_lock);
    }
  }

  GST_INFO("Cleaning up");

  iCaptureSession->stopRepeat();
  argusCheck(iCaptureSession->waitForIdle());

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
