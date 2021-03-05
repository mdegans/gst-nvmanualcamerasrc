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

#ifndef A96C5A45_3E9B_445B_B3AD_E5A1F2FA13C8
#define A96C5A45_3E9B_445B_B3AD_E5A1F2FA13C8

#include "gstnvdsbufferpool.h"
#include "gstnvmanualcamera_utils.h"
#include "nvbuf_utils.h"
#include "nvbufsurface.h"

#include <gst/base/gstbasesrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

G_BEGIN_DECLS

/* #defines don't like whitespacey bits */
#define GST_TYPE_NVMANUALCAMERASRC (gst_nv_manual_camera_src_get_type())
#define GST_NVMANUALCAMERASRC(obj)                               \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_NVMANUALCAMERASRC, \
                              GstNvManualCameraSrc))
#define GST_NVMANUALCAMERASRC_CLASS(klass)                      \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_NVMANUALCAMERASRC, \
                           GstNvManualCameraSrcClass))
#define GST_IS_NVMANUALCAMERASRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_NVMANUALCAMERASRC))
#define GST_IS_NVMANUALCAMERASRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_NVMANUALCAMERASRC))

namespace nvmanualcam::defaults {

const auto AEANTIBANDING_MODE = NvManualCamAeAntibandingMode_Off;
const auto EE_MODE = NvManualCamEdgeEnhancementMode_Off;
const auto TNR_MODE = NvManualCamNoiseReductionMode_Off;
const auto WB_MODE = NvManualCamAwbMode_Auto;
const bool AE_LOCK = false;
const bool AWB_LOCK = false;
const bool METADATA = true;
const bool BUFAPI = false;
const bool SILENT = false;
const float DIGITAL_GAIN = 1.0f;
const float EE_STRENGTH = -1.0;
const float EXP_COMPENSATION = 0.0;
const float EXPOSURE_TIME = 1.0;
const float GAIN = 1.0f;
const float SATURATION = 1.0;
const float TNR_STRENGTH = -1.0;
const int SENSOR_MODE_STATE = -1;
const uint SENSOR_ID = 0;
const uint TIMEOUT = 0;
const uint TOTAL_SENSOR_MODES = 0;

}  // namespace nvmanualcam::defaults

typedef struct _GstNvManualCameraSrc GstNvManualCameraSrc;
typedef struct _GstNvManualCameraSrcClass GstNvManualCameraSrcClass;

typedef struct _GstNvManualCameraSrcBuffer GstNvManualCameraSrcBuffer;

/* NvManualCameraSrc Controls */
typedef struct NvManualCamControls {
  NvManualCamAwbMode wbmode;
  float saturation;
  float exposure_time;     // in frames
  uint64_t exposure_real;  // in nanoseconds
  float gain;              // min: 1, max: 16
  float digital_gain;      // min 1, max: 256
  gboolean meta_enabled;   // enable metadata generation
  NvManualCamNoiseReductionMode NoiseReductionMode;
  NvManualCamEdgeEnhancementMode EdgeEnhancementMode;
  NvManualCamAeAntibandingMode AeAntibandingMode;
  float NoiseReductionStrength;
  float EdgeEnhancementStrength;
  float ExposureCompensation;
  bool AeLock;
  bool AwbLock;
} NvManualCamControls;

/* NvManualCameraSrc buffer */
struct _GstNvManualCameraSrcBuffer {
  gint dmabuf_fd;
  gboolean bufApi;
  GstBuffer* gst_buf;
  NvBufSurface* surf;
};

typedef struct NvManualFrameInfo {
  gint fd;
  guint64 frameNum;
  guint64 frameTime;
  void* captureMeta;  // Argus::CaptureMetadata
} NvManualFrameInfo;

struct _GstNvManualCameraSrc {
  GstBaseSrc base_nvmanualcamera;

  GstPad* srcpad;

  GThread* consumer_thread;
  GThread* manual_thread;

  gboolean silent;

  GstBufferPool* pool;

  GstCaps* outcaps;

  GstVideoInfo info;
  guint64 frame_duration;  // in nanoseconds
  gint sensor_id;
  gint sensor_mode;

  guint total_sensor_modes;
  guint timeout;

  GQueue* nvmm_buffers;
  GMutex nvmm_buffers_queue_lock;
  GCond nvmm_buffers_queue_cond;

  gboolean stop_requested;
  gboolean unlock_requested;

  NvBufferTransformParams transform_params;

  GQueue* manual_buffers;
  GMutex manual_buffers_queue_lock;
  GCond manual_buffers_queue_cond;

  GMutex manual_buffer_consumed_lock;
  GCond manual_buffer_consumed_cond;
  gboolean is_manual_buffer_consumed;

  GMutex eos_lock;
  GCond eos_cond;

  NvManualCamControls controls;
  gboolean wbPropSet;
  gboolean saturationPropSet;
  gboolean exposureTimePropSet;
  gboolean gainPropSet;
  gboolean ispDigitalGainPropSet;
  gboolean tnrStrengthPropSet;
  gboolean tnrModePropSet;
  gboolean edgeEnhancementStrengthPropSet;
  gboolean edgeEnhancementModePropSet;
  gboolean aeAntibandingPropSet;
  gboolean exposureCompensationPropSet;
  gboolean aeLockPropSet;
  gboolean awbLockPropSet;
  gboolean bufApi;
  gboolean in_error;
  void* iRequest_ptr;
  void* iCaptureSession_ptr;
  void* iAutoControlSettings_ptr;
  void* request_ptr;
  void* outRequest_ptr;
  void* iDenoiseSettings_ptr;
  void* iEeSettings_ptr;
  void* iRequestSourceSettings_ptr;
  NvManualFrameInfo* frameInfo;
};

struct _GstNvManualCameraSrcClass {
  GstBaseSrcClass base_nvmanualcamera_class;
};

GType gst_nv_manual_camera_src_get_type(void);

G_END_DECLS

#endif /* A96C5A45_3E9B_445B_B3AD_E5A1F2FA13C8 */
