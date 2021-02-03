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

#include <gst/base/gstbasesrc.h>
#include <gst/gst.h>

#include "gstnvdsbufferpool.h"
#include "gstnvmanualcamera_utils.h"
#include "nvbuf_utils.h"
#include "nvbufsurface.h"

#include "Ordered.h"

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

// clang-format off

#define NVMANUALCAM_DEFAULT_WB_MODE                   NvManualCamAwbMode_Auto
#define NVMANUALCAM_DEFAULT_SATURATION                1.0
#define NVMANUALCAM_DEFAULT_EXPOSURE_TIME             "10000 358733000"
#define NVMANUALCAM_DEFAULT_SENSOR_ID                 0
#define NVMANUALCAM_DEFAULT_SENSOR_MODE_STATE         -1
#define NVMANUALCAM_DEFAULT_TOTAL_SENSOR_MODES        0
#define NVMANUALCAM_DEFAULT_GAIN_RANGE                "1 16"
#define NVMANUALCAM_DEFAULT_DIGITAL_GAIN_RANGE        "1 256"
#define NVMANUALCAM_DEFAULT_TNR_MODE                  NvManualCamNoiseReductionMode_Fast
#define NVMANUALCAM_DEFAULT_TNR_STRENGTH              -1.0
#define NVMANUALCAM_DEFAULT_EE_MODE                   NvManualCamEdgeEnhancementMode_Fast
#define NVMANUALCAM_DEFAULT_EE_STRENGTH               -1.0
#define NVMANUALCAM_DEFAULT_AEANTIBANDING_MODE        NvManualCamAeAntibandingMode_Auto
#define NVMANUALCAM_DEFAULT_EXP_COMPENSATION          0.0
#define NVMANUALCAM_DEFAULT_AE_LOCK                   FALSE
#define NVMANUALCAM_DEFAULT_AWB_LOCK                  FALSE

// clang-format on

typedef struct _GstNvManualCameraSrc GstNvManualCameraSrc;
typedef struct _GstNvManualCameraSrcClass GstNvManualCameraSrcClass;

typedef struct _GstNvManualCameraSrcBuffer GstNvManualCameraSrcBuffer;

typedef struct NvManualCameraRangeRec {
  /**  Lower limit for the range. */
  gfloat low;
  /**  Upper limit for the range. */
  gfloat high;
} NvManualCameraRange;

/* NvManualCameraSrc Controls */
typedef struct NvManualCamControls {
  NvManualCamAwbMode wbmode;
  gfloat saturation;
  NvManualCameraRange exposureTimeRange;
  NvManualCameraRange gainRange;
  NvManualCameraRange ispDigitalGainRange;
  NvManualCamNoiseReductionMode NoiseReductionMode;
  NvManualCamEdgeEnhancementMode EdgeEnhancementMode;
  NvManualCamAeAntibandingMode AeAntibandingMode;
  gfloat NoiseReductionStrength;
  gfloat EdgeEnhancementStrength;
  gfloat ExposureCompensation;
  gboolean AeLock;
  gboolean AwbLock;
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
} NvManualFrameInfo;

struct _GstNvManualCameraSrc {
  GstBaseSrc base_nvmanualcamera;

  GstPad* srcpad;

  GThread* consumer_thread;
  GThread* manual_thread;

  gboolean silent;

  GstBufferPool* pool;

  GstCaps* outcaps;

  gint width;
  gint height;
  gint fps_n;
  gint fps_d;
  gint sensor_id;
  gint sensor_mode;

  guint total_sensor_modes;
  guint timeout;
  gchar* exposureTimeString;
  gchar* gainRangeString;
  gchar* ispDigitalGainRangeString;

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
  gboolean gainRangePropSet;
  gboolean ispDigitalGainRangePropSet;
  gboolean tnrStrengthPropSet;
  gboolean tnrModePropSet;
  gboolean edgeEnhancementStrengthPropSet;
  gboolean edgeEnhancementModePropSet;
  gboolean aeAntibandingPropSet;
  gboolean exposureCompensationPropSet;
  gboolean aeLockPropSet;
  gboolean awbLockPropSet;
  gboolean bufApi;
  gboolean manual_in_error;
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

namespace ArgusSamples {

/**
 * Base class for threads. Derived classes need to implement 'threadInitialize',
 * 'threadExecute' and 'threadShutdown'. This class handles the transition
 * between the thread states.
 */
class ThreadManual {
 public:
  ThreadManual();
  virtual ~ThreadManual();

  /**
   * Initialize
   */
  bool initialize(GstNvManualCameraSrc*);
  /**
   * Shutdown
   */
  bool shutdown();

  /**
   * Wait until the thread is in 'running' state
   *
   * @param timeout [in] timeout in us
   */
  bool waitRunning(useconds_t timeoutUs = 5 * 1000 * 1000);

  GstNvManualCameraSrc* src;

 protected:
  virtual bool threadInitialize(GstNvManualCameraSrc*) = 0;
  virtual bool threadExecute(GstNvManualCameraSrc*) = 0;
  virtual bool threadShutdown(GstNvManualCameraSrc*) = 0;

  /**
   * Request thread shutdown
   */
  bool requestShutdown() {
    m_doShutdown = true;
    return true;
  }

  Ordered<bool> m_doShutdown;  ///< set to request shutdown of the thread

 private:
  pthread_t m_threadID;  ///< thread ID

  /**
   * Thread states
   */
  enum ThreadState {
    THREAD_INACTIVE,      ///< is inactive
    THREAD_INITIALIZING,  ///< is initializing
    THREAD_RUNNING,       ///< is running
    THREAD_FAILED,        ///< has failed
    THREAD_DONE,          ///< execution done
  };
  Ordered<ThreadState> m_threadState;

  bool threadFunction(GstNvManualCameraSrc*);

  static void* threadFunctionStub(void* dataPtr);
};

}  // namespace ArgusSamples

G_END_DECLS

#endif /* A96C5A45_3E9B_445B_B3AD_E5A1F2FA13C8 */
