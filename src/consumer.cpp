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
#include "impl_data.hpp"
#include "logging.hpp"
#include "metadata.hpp"

#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <Error.h>

#include <inttypes.h>

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_consumer_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_consumer_debug

using namespace Argus;
using namespace EGLStream;

namespace nvmanualcam::consumer {

bool execute(std::shared_ptr<ArgusCtx> ctx,
             std::shared_ptr<ArgusControls> ctrl) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_consumer_debug,
                          "nvmanualcamerasrc:consumer", 0,
                          "nvmanualcamerasrc consumer");

  Argus::Status err = Argus::Status::STATUS_OK; /** an error if nonzero */
  NvManualFrameInfo frameInfo = {};
  frameInfo.fd = -1;

  // start repeated requests
  ctx->repeat_request();

  while ((!ctx->stop_requested) && (err != Argus::Status::STATUS_OK)) {
    // set properites if any have been updated, then update the request.
    if (ctrl->isUpdated(Properties::ID::WHITE_BALANCE)) {
      switch (ctrl->getWbmode()) {
        case NvManualCamAwbMode_Off:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_OFF);
          break;
        case NvManualCamAwbMode_Auto:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_AUTO);
          break;
        case NvManualCamAwbMode_Incandescent:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_INCANDESCENT);
          break;
        case NvManualCamAwbMode_Fluorescent:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_FLUORESCENT);
          break;
        case NvManualCamAwbMode_WarmFluorescent:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_WARM_FLUORESCENT);
          break;
        case NvManualCamAwbMode_Daylight:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_DAYLIGHT);
          break;
        case NvManualCamAwbMode_CloudyDaylight:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_CLOUDY_DAYLIGHT);
          break;
        case NvManualCamAwbMode_Twilight:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_TWILIGHT);
          break;
        case NvManualCamAwbMode_Shade:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_SHADE);
          break;
        case NvManualCamAwbMode_Manual:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_MANUAL);
          break;
        default:
          ctx->iAutoSettings->setAwbMode(AWB_MODE_OFF);
          break;
      }
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::SATURATION)) {
      ctx->iAutoSettings->setColorSaturationEnable(true);
      ctx->iAutoSettings->setColorSaturation(ctrl->getSaturation());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::EXP_COMPENSATION)) {
      ctx->iAutoSettings->setExposureCompensation(ctrl->getExpCompensation());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::AE_LOCK)) {
      ctx->iAutoSettings->setAeLock(ctrl->getAeLock());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::AWB_LOCK)) {
      ctx->iAutoSettings->setAwbLock(ctrl->getAwbLock());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::TNR_MODE)) {
      switch (ctrl->getTnrMode()) {
        case NvManualCamNoiseReductionMode_Off:
          ctx->iDenoiseSettings->setDenoiseMode(DENOISE_MODE_OFF);
          break;
        case NvManualCamNoiseReductionMode_Fast:
          ctx->iDenoiseSettings->setDenoiseMode(DENOISE_MODE_FAST);
          break;
        case NvManualCamNoiseReductionMode_HighQuality:
          ctx->iDenoiseSettings->setDenoiseMode(DENOISE_MODE_HIGH_QUALITY);
          break;
        default:
          ctx->iDenoiseSettings->setDenoiseMode(DENOISE_MODE_OFF);
          break;
      }
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::TNR_STRENGTH)) {
      ctx->iDenoiseSettings->setDenoiseStrength(ctrl->getTnrStrength());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::EE_MODE)) {
      switch (ctrl->getEeMode()) {
        case NvManualCamEdgeEnhancementMode_Off:
          ctx->iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_OFF);
          break;
        case NvManualCamEdgeEnhancementMode_Fast:
          ctx->iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_FAST);
          break;
        case NvManualCamEdgeEnhancementMode_HighQuality:
          ctx->iEeSettings->setEdgeEnhanceMode(EDGE_ENHANCE_MODE_HIGH_QUALITY);
          break;
        default:
          GST_ERROR("INVALID EE MODE!");
          break;
      }
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::EE_STRENGTH)) {
      ctx->iEeSettings->setEdgeEnhanceStrength(ctrl->getEeStrength());
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::AEANTIBANDING_MODE)) {
      switch (ctrl->getAeantibandingMode()) {
        case NvManualCamAeAntibandingMode_Off:
          ctx->iAutoSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF);
          break;
        case NvManualCamAeAntibandingMode_Auto:
          ctx->iAutoSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_AUTO);
          break;
        case NvManualCamAeAntibandingMode_50HZ:
          ctx->iAutoSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_50HZ);
          break;
        case NvManualCamAeAntibandingMode_60HZ:
          ctx->iAutoSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_60HZ);
          break;
        default:
          GST_ERROR("INVALID AE MODE!");
          break;
      }
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::GAIN)) {
      thread_local Argus::Range<float> range;
      range.min() = ctrl->getGain();
      range.max() = range.min();
      ctx->iSourceSettings->setGainRange(range);
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::DIGITAL_GAIN)) {
      thread_local Argus::Range<float> range;
      range.min() = ctrl->getDigitalGain();
      range.max() = range.min();
      ctx->iAutoSettings->setIspDigitalGainRange(range);
      ctx->repeat_request();
    }

    if (ctrl->isUpdated(Properties::ID::EXPOSURE_REAL)) {
      thread_local Argus::Range<uint64_t> range;
      range.min() = ctrl->getExposureReal();
      range.max() = range.min();
      ctx->iSourceSettings->setExposureTimeRange(range);
      ctx->repeat_request();
    }

    // Get a frame
    UniqueObj<Frame> frame(ctx->iFrameConsumer.acquireFrame(-1, &err));
    IFrame* iFrame = interface_cast<IFrame>(frame);
    if (!iFrame) {
      ORIGINATE_ERROR("Failed to get Frame (Status %d).", err);
    }

    if (ctrl->isUpdated(Properties::ID::META_ENABLED)) {
      auto iArgusCaptureMetadata =
          Argus::interface_cast<IArgusCaptureMetadata>(frame);
      if (!iArgusCaptureMetadata) {
        ORIGINATE_ERROR("IArgusCaptureMetadata not supported by Frame.");
      }
      // lifetime tied to frame
      auto meta = iArgusCaptureMetadata->getMetadata();
      if (!meta) {
        ORIGINATE_ERROR("Could not get Argus::CaptureMetadata");
      }
      frameInfo.metadata = nvmanualcam::Metadata::create(meta);
    }

    // Get the IImageNativeBuffer extension interface and create the fd.
    NV::IImageNativeBuffer* iNativeBuffer =
        Argus::interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
    if (!iNativeBuffer) {
      ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
    }

    if (frameInfo.fd < 0) {
      frameInfo.fd = iNativeBuffer->createNvBuffer(
          resolution, NvBufferColorFormat_YUV420, NvBufferLayout_BlockLinear);
      if (!ctrl->getSilent()) {
        GST_INFO("Acquired Frame. %d", src->frameInfo->fd);
      }
    } else if (iNativeBuffer->copyToNvBuffer(frameInfo.fd) != STATUS_OK) {
      ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
    }

    frameInfo.frameNum = iFrame->getNumber();
    frameInfo.frameTime = iFrame->getTime();

    if (!ctrl->getSilent()) {
      GST_INFO("Acquired Frame: %" PRIu64 ", time %" PRIu64, frameInfo.frameNum,
               frameInfo.frameTime);
    }

    // Attach capture metadata to frameInfo
    // TODO(mdegans): research gstreamer frame level metadata to see if there is
    //  a better way to do this.

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

  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool Consumer::threadShutdown(GstNvManualCameraSrc* src) {
  (void)src;
  return true;
}

}  // namespace nvmanualcam::consumer