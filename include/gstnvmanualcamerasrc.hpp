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
#include "metadata.hpp"
#include "nvbuf_utils.h"
#include "nvbufsurface.h"

#include <Argus/Argus.h>

#include <gst/base/gstbasesrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <memory>

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

typedef struct _GstNvManualCameraSrc GstNvManualCameraSrc;
typedef struct _GstNvManualCameraSrcClass GstNvManualCameraSrcClass;
typedef struct _GstNvManualCameraSrcBuffer GstNvManualCameraSrcBuffer;

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
  std::unique_ptr<nvmanualcam::Metadata> metadata;
} NvManualFrameInfo;

struct ArgusControls;
struct ArgusCtx;

struct _GstNvManualCameraSrc {
  GstBaseSrc base_nvmanualcamera;

  GstPad* srcpad;

  GThread* consumer_thread;
  GThread* manual_thread;

  GstBufferPool* pool;

  GstCaps* outcaps;

  GstVideoInfo info;
  guint64 frame_duration;  // in nanoseconds
  guint timeout;

  GQueue* nvmm_buffers;
  GMutex nvmm_buffers_queue_lock;
  GCond nvmm_buffers_queue_cond;

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

  gboolean in_error;

  std::shared_ptr<ArgusControls> controls;

  NvManualFrameInfo* frameInfo;
};

struct _GstNvManualCameraSrcClass {
  GstBaseSrcClass base_nvmanualcamera_class;
};

GType gst_nv_manual_camera_src_get_type(void);

G_END_DECLS

#endif /* A96C5A45_3E9B_445B_B3AD_E5A1F2FA13C8 */
