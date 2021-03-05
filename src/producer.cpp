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
              GstNvManualCameraSrc* self) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_producer_debug,
                          "nvmanualcamerasrc:producer", 0,
                          "nvmanualcamerasrc producer");
  GST_INFO("starting producer thread");
  gfloat frameRate = 0;
  uint32_t index = 0;
  gint found = 0;
  gint best_match = -1;

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
      iCameraProvider->createCaptureSession(cameraDevices[cameraIndex]));
  ICaptureSession* iCaptureSession =
      interface_cast<ICaptureSession>(captureSession);
  if (!iCaptureSession)
    ORIGINATE_ERROR("Failed to create CaptureSession");

  self->iCaptureSession_ptr = iCaptureSession;
  PRODUCER_PRINT("Creating output stream");
  UniqueObj<OutputStreamSettings> streamSettings(
      iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
  IEGLOutputStreamSettings* iStreamSettings =
      interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (iStreamSettings) {
    iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iStreamSettings->setResolution(streamSize);
    if (self->controls.meta_enabled) {
      iStreamSettings->setMetadataEnable(true);
    }
  }
  UniqueObj<OutputStream> outputStream(
      iCaptureSession->createOutputStream(streamSettings.get()));
  IEGLOutputStream* iStream = interface_cast<IEGLOutputStream>(outputStream);
  if (!iStream)
    ORIGINATE_ERROR("Failed to create OutputStream");

  utils::Consumer consumerThread(outputStream.get());
  PROPAGATE_ERROR(consumerThread.initialize(self));

  // Wait until the consumer is connected to the stream.
  PROPAGATE_ERROR(consumerThread.waitRunning());

  // Create capture request and enable output stream.
  UniqueObj<Request> request(iCaptureSession->createRequest());
  IRequest* iRequest = interface_cast<IRequest>(request);
  self->iRequest_ptr = iRequest;
  if (!iRequest)
    ORIGINATE_ERROR("Failed to create Request");
  iRequest->enableOutputStream(outputStream.get());

  IAutoControlSettings* iAutoControlSettings =
      interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
  self->iAutoControlSettings_ptr = iAutoControlSettings;
  std::vector<SensorMode*> modes;
  ICameraProperties* camProps =
      interface_cast<ICameraProperties>(cameraDevices[cameraIndex]);
  if (!camProps)
    ORIGINATE_ERROR("Failed to create camera properties");
  camProps->getAllSensorModes(&modes);

  ISourceSettings* requestSourceSettings =
      interface_cast<ISourceSettings>(iRequest->getSourceSettings());
  if (!requestSourceSettings)
    ORIGINATE_ERROR("Failed to get request source settings");
  self->iRequestSourceSettings_ptr = requestSourceSettings;

  if (cameraMode != nvmanualcam::defaults::SENSOR_MODE_STATE &&
      static_cast<uint32_t>(cameraMode) >= modes.size())
    ORIGINATE_ERROR("Invalid sensor mode %d selected %d present", cameraMode,
                    static_cast<int32_t>(modes.size()));

  self->total_sensor_modes = modes.size();

  PRODUCER_PRINT("Available Sensor modes :");
  frameRate = self->info.fps_n / self->info.fps_d;
  self->frame_duration = get_frame_duration(self->info);
  ISensorMode* iSensorMode[modes.size()];
  Range<float> sensorModeAnalogGainRange;
  Range<float> ispDigitalGainRange;
  Range<uint64_t> limitExposureTimeRange;
  for (index = 0; index < modes.size(); index++) {
    iSensorMode[index] = interface_cast<ISensorMode>(modes[index]);
    sensorModeAnalogGainRange = iSensorMode[index]->getAnalogGainRange();
    limitExposureTimeRange = iSensorMode[index]->getExposureTimeRange();
    PRODUCER_PRINT(
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
          self->frame_duration >=
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
      PRODUCER_PRINT(
          "Requested resolution W = %d H = %d not supported by Sensor.",
          streamSize.width(), streamSize.height());
      cameraMode = 0;
    } else {
      cameraMode = best_match;
    }
  }
  /* Update Sensor Mode*/
  self->sensor_mode = cameraMode;

  if (frameRate >
      round((1e9 / (iSensorMode[cameraMode]->getFrameDurationRange().min())))) {
    self->in_error = TRUE;
    GST_ERROR_OBJECT(self,
                     "Frame Rate specified (%d/%d) is greater than supported.",
                     self->info.fps_n, self->info.fps_d);
  }

  IDenoiseSettings* denoiseSettings = interface_cast<IDenoiseSettings>(request);
  if (!denoiseSettings)
    ORIGINATE_ERROR("Failed to get DenoiseSettings interface");
  self->iDenoiseSettings_ptr = denoiseSettings;

  IEdgeEnhanceSettings* eeSettings =
      interface_cast<IEdgeEnhanceSettings>(request);
  if (!eeSettings)
    ORIGINATE_ERROR("Failed to get EdgeEnhancementSettings interface");
  self->iEeSettings_ptr = eeSettings;

  /* Setting Noise Reduction Mode and Strength*/
  if (self->tnrModePropSet) {
    switch (self->controls.NoiseReductionMode) {
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
    self->tnrModePropSet = FALSE;
  }

  if (self->tnrStrengthPropSet) {
    denoiseSettings->setDenoiseStrength(self->controls.NoiseReductionStrength);
    self->tnrStrengthPropSet = FALSE;
  }

  /* Setting Edge Enhancement Mode and Strength*/
  if (self->edgeEnhancementModePropSet) {
    switch (self->controls.EdgeEnhancementMode) {
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
    self->edgeEnhancementModePropSet = FALSE;
  }

  if (self->edgeEnhancementStrengthPropSet) {
    eeSettings->setEdgeEnhanceStrength(self->controls.EdgeEnhancementStrength);
    self->edgeEnhancementStrengthPropSet = FALSE;
  }

  /* Setting AE Antibanding Mode */
  if (self->aeAntibandingPropSet) {
    switch (self->controls.AeAntibandingMode) {
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
    self->aeAntibandingPropSet = FALSE;
  }

  if (self->exposureCompensationPropSet) {
    iAutoControlSettings->setExposureCompensation(
        self->controls.ExposureCompensation);
    self->exposureCompensationPropSet = FALSE;
  }

  /* Setting auto white balance lock */
  if (self->awbLockPropSet) {
    if (self->controls.AwbLock)
      iAutoControlSettings->setAwbLock(true);
    else
      iAutoControlSettings->setAwbLock(false);
    self->awbLockPropSet = FALSE;
  }

  /* Setting auto exposure lock */
  if (self->aeLockPropSet) {
    if (self->controls.AeLock)
      iAutoControlSettings->setAeLock(true);
    else
      iAutoControlSettings->setAeLock(false);
    self->aeLockPropSet = FALSE;
  }

  PRODUCER_PRINT(
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
  if (self->wbPropSet) {
    switch (self->controls.wbmode) {
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
    self->wbPropSet = FALSE;
  }

  /* Setting color saturation property */
  if (self->saturationPropSet) {
    iAutoControlSettings->setColorSaturationEnable(TRUE);
    iAutoControlSettings->setColorSaturation(self->controls.saturation);
    self->saturationPropSet = false;
  }

  if (self->exposureTimePropSet) {
    limitExposureTimeRange.min() = self->controls.exposure_real;
    limitExposureTimeRange.max() = self->controls.exposure_real;
    requestSourceSettings->setExposureTimeRange(limitExposureTimeRange);
    self->exposureTimePropSet = false;
  }

  if (self->gainPropSet) {
    sensorModeAnalogGainRange.min() = self->controls.gain;
    sensorModeAnalogGainRange.max() = self->controls.gain;
    requestSourceSettings->setGainRange(sensorModeAnalogGainRange);
    self->gainPropSet = false;
  }

  if (self->ispDigitalGainPropSet) {
    ispDigitalGainRange.min() = self->controls.digital_gain;
    ispDigitalGainRange.max() = self->controls.digital_gain;
    iAutoControlSettings->setIspDigitalGainRange(ispDigitalGainRange);
    self->ispDigitalGainPropSet = false;
  }

  requestSourceSettings->setSensorMode(modes[cameraMode]);
  if (!self->info.fps_n) {
    frameRate = defaults::DEFAULT_FPS;
  }

  requestSourceSettings->setFrameDurationRange(
      Range<uint64_t>(1e9 / frameRate));

  PRODUCER_PRINT("Setup Complete, Starting captures for %d seconds", secToRun);

  PRODUCER_PRINT("Starting repeat capture requests.");
  Request* captureRequest = request.get();
  self->request_ptr = captureRequest;
  iCaptureSession->capture(captureRequest);
  if (iCaptureSession->capture(captureRequest) == 0)
    ORIGINATE_ERROR("Failed to start capture request");

  if (self->in_error) {
    PRODUCER_ERROR("InvalidState.");
    iCaptureSession->cancelRequests();
    self->timeout = 1;
  } else if (secToRun != 0) {
    sleep(secToRun);
    iCaptureSession->cancelRequests();
  } else {
    if (self->stop_requested == FALSE) {
      g_mutex_lock(&self->eos_lock);
      g_cond_wait(&self->eos_cond, &self->eos_lock);
      g_mutex_unlock(&self->eos_lock);
    }
  }

  PRODUCER_PRINT("Cleaning up");

  iCaptureSession->stopRepeat();
  iCaptureSession->waitForIdle();

  // Destroy the output stream. This destroys the EGLStream which causes
  // the GL consumer acquire to fail and the consumer thread to end.
  outputStream.reset();

  // Manual execution completed, signal the buffer consumed cond.
  if (!self->is_manual_buffer_consumed) {
    g_mutex_lock(&self->manual_buffer_consumed_lock);
    g_cond_signal(&self->manual_buffer_consumed_cond);
    self->is_manual_buffer_consumed = TRUE;
    g_mutex_unlock(&self->manual_buffer_consumed_lock);
  }
  // Wait for the consumer thread to complete.
  PROPAGATE_ERROR(consumerThread.shutdown());

  if (self->in_error)
    return false;

  PRODUCER_PRINT("Done Success");

  return true;
}

}  // namespace nvmanualcam
