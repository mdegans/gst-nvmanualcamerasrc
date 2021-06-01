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

#include "consumer.hpp"
#include "logging.hpp"
#include "metadata.hpp"

#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <Error.h>

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_consumer_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_consumer_debug

using namespace Argus;
using namespace EGLStream;

namespace nvmanualcam::utils {

bool Consumer::threadInitialize(GstNvManualCameraSrc* _) {
  (void)_;
  // Create the FrameConsumer.
  m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
  if (!m_consumer)
    ORIGINATE_ERROR("Failed to create FrameConsumer");

  return true;
}

bool Consumer::threadExecute(GstNvManualCameraSrc* src) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_consumer_debug,
                          "nvmanualcamerasrc:consumer", 0,
                          "nvmanualcamerasrc consumer");
  IEGLOutputStream* iStream = interface_cast<IEGLOutputStream>(m_stream);
  Size2D<uint32_t> streamSize(src->info.width, src->info.height);
  IFrameConsumer* iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);

  // Wait until the producer has connected to the stream.
  GST_INFO("Waiting until producer is connected...");
  if (iStream->waitUntilConnected() != STATUS_OK)
    ORIGINATE_ERROR("Stream failed to connect.");
  GST_INFO("Producer has connected; continuing.");
  // FIXME(mdegans): lifetime issues, above and below
  IAutoControlSettings* l_iAutoControlSettings_ptr =
      (IAutoControlSettings*)src->iAutoControlSettings_ptr;
  ICaptureSession* l_iCaptureSession =
      (ICaptureSession*)src->iCaptureSession_ptr;
  IDenoiseSettings* l_iDenoiseSettings_ptr =
      (IDenoiseSettings*)src->iDenoiseSettings_ptr;
  IEdgeEnhanceSettings* l_iEeSettings_ptr =
      (IEdgeEnhanceSettings*)src->iEeSettings_ptr;
  ISourceSettings* l_iRequestSourceSettings_ptr =
      (ISourceSettings*)src->iRequestSourceSettings_ptr;
  Request* l_captureRequest = (Request*)src->request_ptr;
  Range<float> sensorModeAnalogGainRange;
  Range<float> ispDigitalGainRange;
  Range<uint64_t> limitExposureTimeRange;
  NONZERO_RETURN_FALSE(l_iCaptureSession->repeat(l_captureRequest));

  // FIXME(mdegans): this works, but unsure with every compiler
  src->frameInfo = g_slice_new0(NvManualFrameInfo);
  src->frameInfo->fd = -1;
  src->frameInfo->metadata.reset();
  Argus::Status err = Argus::STATUS_OK;
  while (true) {
    UniqueObj<Frame> frame(
        iFrameConsumer->acquireFrame(Argus::TIMEOUT_INFINITE, &err));
    NONZERO_STOP_SRC(err);
    if (src->stop_requested == TRUE) {
      break;
    }
    if (err || !frame) {
      g_mutex_lock(&src->manual_buffers_queue_lock);
      STOP_SRC("Could not acquireFrame");
      g_mutex_unlock(&src->manual_buffers_queue_lock);
      break;
    }

    if (src->wbPropSet) {
      switch (src->controls.wbmode) {
        case NvManualCamAwbMode_Off:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_OFF));
          break;
        case NvManualCamAwbMode_Auto:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_AUTO));
          break;
        case NvManualCamAwbMode_Incandescent:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_INCANDESCENT));
          break;
        case NvManualCamAwbMode_Fluorescent:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_FLUORESCENT));
          break;
        case NvManualCamAwbMode_WarmFluorescent:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAwbMode(
              AWB_MODE_WARM_FLUORESCENT));
          break;
        case NvManualCamAwbMode_Daylight:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_DAYLIGHT));
          break;
        case NvManualCamAwbMode_CloudyDaylight:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_CLOUDY_DAYLIGHT));
          break;
        case NvManualCamAwbMode_Twilight:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_TWILIGHT));
          break;
        case NvManualCamAwbMode_Shade:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_SHADE));
          break;
        case NvManualCamAwbMode_Manual:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_MANUAL));
          break;
        default:
          NONZERO_STOP_SRC(
              l_iAutoControlSettings_ptr->setAwbMode(AWB_MODE_OFF));
          break;
      }
      src->wbPropSet = FALSE;
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
    }

    if (src->saturationPropSet) {
      NONZERO_STOP_SRC(
          l_iAutoControlSettings_ptr->setColorSaturationEnable(TRUE));
      NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setColorSaturation(
          src->controls.saturation));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->saturationPropSet = FALSE;
    }

    if (src->exposureCompensationPropSet) {
      NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setExposureCompensation(
          src->controls.ExposureCompensation));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->exposureCompensationPropSet = FALSE;
    }

    if (src->aeLockPropSet) {
      if (src->controls.AeLock)
        NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeLock(true));
      else
        NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeLock(false));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->aeLockPropSet = FALSE;
    }

    if (src->awbLockPropSet) {
      if (src->controls.AwbLock)
        NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAwbLock(true));
      else
        NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAwbLock(false));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->awbLockPropSet = FALSE;
    }

    if (src->tnrModePropSet) {
      switch (src->controls.NoiseReductionMode) {
        case NvManualCamNoiseReductionMode_Off:
          NONZERO_STOP_SRC(
              l_iDenoiseSettings_ptr->setDenoiseMode(DENOISE_MODE_OFF));
          break;
        case NvManualCamNoiseReductionMode_Fast:
          NONZERO_STOP_SRC(
              l_iDenoiseSettings_ptr->setDenoiseMode(DENOISE_MODE_FAST));
          break;
        case NvManualCamNoiseReductionMode_HighQuality:
          NONZERO_STOP_SRC(l_iDenoiseSettings_ptr->setDenoiseMode(
              DENOISE_MODE_HIGH_QUALITY));
          break;
        default:
          NONZERO_STOP_SRC(
              l_iDenoiseSettings_ptr->setDenoiseMode(DENOISE_MODE_OFF));
          break;
      }
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->tnrModePropSet = FALSE;
    }

    if (src->tnrStrengthPropSet) {
      NONZERO_STOP_SRC(l_iDenoiseSettings_ptr->setDenoiseStrength(
          src->controls.NoiseReductionStrength));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->tnrStrengthPropSet = FALSE;
    }

    if (src->edgeEnhancementModePropSet) {
      switch (src->controls.EdgeEnhancementMode) {
        case NvManualCamEdgeEnhancementMode_Off:
          NONZERO_STOP_SRC(
              l_iEeSettings_ptr->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF));
          break;
        case NvManualCamEdgeEnhancementMode_Fast:
          NONZERO_STOP_SRC(
              l_iEeSettings_ptr->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST));
          break;
        case NvManualCamEdgeEnhancementMode_HighQuality:
          NONZERO_STOP_SRC(l_iEeSettings_ptr->setEdgeEnhanceMode(
              EDGE_ENHANCE_MODE_HIGH_QUALITY));
          break;
        default:
          NONZERO_STOP_SRC(
              l_iEeSettings_ptr->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF));
          break;
      }
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->edgeEnhancementModePropSet = FALSE;
    }

    if (src->edgeEnhancementStrengthPropSet) {
      NONZERO_STOP_SRC(l_iEeSettings_ptr->setEdgeEnhanceStrength(
          src->controls.EdgeEnhancementStrength));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->edgeEnhancementStrengthPropSet = FALSE;
    }

    if (src->aeAntibandingPropSet) {
      switch (src->controls.AeAntibandingMode) {
        case NvManualCamAeAntibandingMode_Off:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeAntibandingMode(
              AE_ANTIBANDING_MODE_OFF));
          break;
        case NvManualCamAeAntibandingMode_Auto:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeAntibandingMode(
              AE_ANTIBANDING_MODE_AUTO));
          break;
        case NvManualCamAeAntibandingMode_50HZ:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeAntibandingMode(
              AE_ANTIBANDING_MODE_50HZ));
          break;
        case NvManualCamAeAntibandingMode_60HZ:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeAntibandingMode(
              AE_ANTIBANDING_MODE_60HZ));
          break;
        default:
          NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setAeAntibandingMode(
              AE_ANTIBANDING_MODE_OFF));
          break;
      }
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->aeAntibandingPropSet = FALSE;
    }

    if (src->gainPropSet) {
      sensorModeAnalogGainRange.min() = src->controls.gain;
      sensorModeAnalogGainRange.max() = src->controls.gain;
      NONZERO_STOP_SRC(l_iRequestSourceSettings_ptr->setGainRange(
          sensorModeAnalogGainRange));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->gainPropSet = false;
    }

    if (src->ispDigitalGainPropSet) {
      ispDigitalGainRange.min() = src->controls.digital_gain;
      ispDigitalGainRange.max() = src->controls.digital_gain;
      NONZERO_STOP_SRC(l_iAutoControlSettings_ptr->setIspDigitalGainRange(
          ispDigitalGainRange));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->ispDigitalGainPropSet = false;
    }

    if (src->exposureTimePropSet == TRUE) {
      limitExposureTimeRange.min() = src->controls.exposure_real;
      limitExposureTimeRange.max() = src->controls.exposure_real;
      NONZERO_STOP_SRC(l_iRequestSourceSettings_ptr->setExposureTimeRange(
          limitExposureTimeRange));
      NONZERO_STOP_SRC(l_iCaptureSession->repeat(l_captureRequest));
      src->exposureTimePropSet = false;
    }

    // Use the IFrame interface to print out the frame number/timestamp, and
    // to provide access to the Image in the Frame.
    IFrame* iFrame = interface_cast<IFrame>(frame);
    if (!iFrame) {
      STOP_SRC("Failed to get IFrame interface.");
      break;
    }

    // Get the IImageNativeBuffer extension interface and create the fd.
    NV::IImageNativeBuffer* iNativeBuffer =
        Argus::interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
    if (!iNativeBuffer) {
      STOP_SRC("IImageNativeBuffer not supported by Image.");
      break;
    }

    if (src->frameInfo->fd < 0) {
      src->frameInfo->fd = iNativeBuffer->createNvBuffer(
          streamSize, NvBufferColorFormat_YUV420, NvBufferLayout_BlockLinear,
          EGLStream::NV::ROTATION_0, &err);
      if (err) {
        STOP_SRC("Could not createNvBuffer");
        break;
      } else if (!src->silent) {
        GST_INFO("Acquired Frame. %d", src->frameInfo->fd);
      }
    } else if (iNativeBuffer->copyToNvBuffer(src->frameInfo->fd) != STATUS_OK) {
      STOP_SRC("IImageNativeBuffer not supported by Image.");
      break;
    }

    if (!src->silent) {
      GST_INFO("Acquired Frame: %llu, time %llu",
               static_cast<unsigned long long>(iFrame->getNumber()),
               static_cast<unsigned long long>(iFrame->getTime()));
    }

    src->frameInfo->frameNum = iFrame->getNumber();
    src->frameInfo->frameTime = iFrame->getTime();

    // Attach capture metadata to frameInfo
    // TODO(mdegans): research gstreamer frame level metadata to see if there is
    //  a better way to do this.
    if (src->controls.meta_enabled) {
      auto iArgusCaptureMetadata =
          Argus::interface_cast<IArgusCaptureMetadata>(frame);
      if (!iArgusCaptureMetadata) {
        STOP_SRC("IArgusCaptureMetadata not supported by Frame.");
        break;
      }
      // lifetime tied to frame
      auto meta = iArgusCaptureMetadata->getMetadata();
      if (!meta) {
        STOP_SRC("IArgusCaptureMetadata not supported by Frame.");
        break;
      }
      src->frameInfo->metadata = nvmanualcam::Metadata::create(meta);
      if (!src->frameInfo->metadata) {
        STOP_SRC("Could not create Metadata from Argus Metadata");
        break;
      }
    }

    g_mutex_lock(&src->manual_buffers_queue_lock);
    g_queue_push_tail(src->manual_buffers, (src->frameInfo));
    g_cond_signal(&src->manual_buffers_queue_cond);
    g_mutex_unlock(&src->manual_buffers_queue_lock);

    g_mutex_lock(&src->manual_buffer_consumed_lock);
    while (!src->is_manual_buffer_consumed)
      g_cond_wait(&src->manual_buffer_consumed_cond,
                  &src->manual_buffer_consumed_lock);
    src->is_manual_buffer_consumed = FALSE;
    g_mutex_unlock(&src->manual_buffer_consumed_lock);
  }

  g_slice_free(NvManualFrameInfo, src->frameInfo);
  if (!src->in_error) {
    GST_INFO("Done Success");
  }
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool Consumer::threadShutdown(GstNvManualCameraSrc* src) {
  (void)src;
  return true;
}

}  // namespace nvmanualcam::utils