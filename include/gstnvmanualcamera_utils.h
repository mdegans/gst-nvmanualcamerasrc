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

#ifndef B61F7423_3531_4016_9B09_EA5EDB1BB49D
#define B61F7423_3531_4016_9B09_EA5EDB1BB49D

#include <gst/gst.h>
#include <gst/video/video.h>

#ifdef __cplusplus
extern "C" {
#endif

namespace nvmanualcam {

/**
 * @brief Get the frame duration in nanoseconds from a GstVideoInfo.
 *
 * @param info a GstVideoInfo structure
 * @return duration of one frame in nanoseconds
 */
guint64 get_frame_duration(const GstVideoInfo& info);

}  // namespace nvcvcam

typedef enum {
  NvManualCamAwbMode_Off = 0,
  NvManualCamAwbMode_Auto,
  NvManualCamAwbMode_Incandescent,
  NvManualCamAwbMode_Fluorescent,
  NvManualCamAwbMode_WarmFluorescent,
  NvManualCamAwbMode_Daylight,
  NvManualCamAwbMode_CloudyDaylight,
  NvManualCamAwbMode_Twilight,
  NvManualCamAwbMode_Shade,
  NvManualCamAwbMode_Manual,

} NvManualCamAwbMode;

typedef enum {
  NvManualCamNoiseReductionMode_Off = 0,
  NvManualCamNoiseReductionMode_Fast,
  NvManualCamNoiseReductionMode_HighQuality

} NvManualCamNoiseReductionMode;

typedef enum {
  NvManualCamEdgeEnhancementMode_Off = 0,
  NvManualCamEdgeEnhancementMode_Fast,
  NvManualCamEdgeEnhancementMode_HighQuality

} NvManualCamEdgeEnhancementMode;

typedef enum {
  NvManualCamAeAntibandingMode_Off = 0,
  NvManualCamAeAntibandingMode_Auto,
  NvManualCamAeAntibandingMode_50HZ,
  NvManualCamAeAntibandingMode_60HZ

} NvManualCamAeAntibandingMode;

GType gst_nvmanualcam_white_balance_mode_get_type(void);
#define GST_TYPE_NVMANUALCAM_WB_MODE \
  (gst_nvmanualcam_white_balance_mode_get_type())

GType gst_nvmanualcam_tnr_mode_get_type(void);
#define GST_TYPE_NVMANUALCAM_TNR_MODE (gst_nvmanualcam_tnr_mode_get_type())

GType gst_nvmanualcam_edge_enhancement_mode_get_type(void);
#define GST_TYPE_NVMANUALCAM_EDGE_ENHANCEMENT_MODE \
  (gst_nvmanualcam_edge_enhancement_mode_get_type())

GType gst_nvmanualcam_aeantibanding_mode_get_type(void);
#define GST_TYPE_NVMANUALCAM_AEANTIBANDING_MODE \
  (gst_nvmanualcam_aeantibanding_mode_get_type())

#ifdef __cplusplus
}
#endif

#endif /* B61F7423_3531_4016_9B09_EA5EDB1BB49D */
