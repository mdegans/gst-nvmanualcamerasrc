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

/**
 * Sample pipeline
 *
 * gst-launch-1.0
 * nvmanualcamerasrc !
 * "video/x-raw(memory:NVMM), width=640, height=480, format=NV12,
 * framerate=30/1" ! nvoverlaysink -e -v
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "consumer.hpp"
#include "gstnvmanualcamerasrc.hpp"
#include "logging.hpp"
#include "metadata.hpp"
#include "producer.hpp"

#include "impl_data.hpp"

#include <Argus/Argus.h>

#include <gst/base/gstbasesrc.h>
#include <gst/gst.h>

static const char* CAPTURE_CAPS =
    "video/x-raw(memory:NVMM), "
    "width = (int) [ 1, MAX ], "
    "height = (int) [ 1, MAX ], "
    "format = (string) { NV12 }, "
    "framerate = (fraction) [ 0, MAX ];";

static const guint MIN_BUFFERS = 6;
static const guint MAX_BUFFERS = 8;

static const gfloat MIN_EXPOSURE_TIME = 0.0f;
static const gfloat MAX_EXPOSURE_TIME = 1.0f;

static const float MIN_GAIN = 1.0f;
static const float MAX_GAIN = 16.0f;

static const float MIN_DIGITAL_GAIN = 1.0f;
static const float MAX_DIGITAL_GAIN = 256.0f;

static const char* GST_NVMANUAL_MEMORY_TYPE = "nvmanualcam";

GST_DEBUG_CATEGORY_STATIC(gst_nvmanualcamerasrc_debug);
#define GST_CAT_DEFAULT gst_nvmanualcamerasrc_debug

G_BEGIN_DECLS

struct _GstNVManualMemory {
  GstMemory mem;
  GstNvManualCameraSrcBuffer* nvcam_buf;
};

struct _GstNVManualMemoryAllocator {
  GstAllocator parent;
  GstNvManualCameraSrc* owner;
};

struct _GstNVManualMemoryAllocatorClass {
  GstAllocatorClass parent_class;
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */

static GstStaticPadTemplate src_factory =
    GST_STATIC_PAD_TEMPLATE("src",
                            GST_PAD_SRC,
                            GST_PAD_ALWAYS,
                            GST_STATIC_CAPS(CAPTURE_CAPS));

typedef struct _GstNVManualMemory GstNVManualMemory;
typedef struct _GstNVManualMemoryAllocator GstNVManualMemoryAllocator;
typedef struct _GstNVManualMemoryAllocatorClass GstNVManualMemoryAllocatorClass;

GType gst_nv_memory_allocator_get_type(void);
#define GST_TYPE_NV_MEMORY_ALLOCATOR (gst_nv_memory_allocator_get_type())

#define gst_nv_manual_camera_src_parent_class parent_class
G_DEFINE_TYPE(GstNvManualCameraSrc,
              gst_nv_manual_camera_src,
              GST_TYPE_BASE_SRC);
G_DEFINE_TYPE(GstNVManualMemoryAllocator,
              gst_nv_memory_allocator,
              GST_TYPE_ALLOCATOR);

#define GST_NVMEMORY_ALLOCATOR(obj)                                \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_NV_MEMORY_ALLOCATOR, \
                              GstNVManualMemoryAllocator))

static gpointer consumer_thread(gpointer base);

static gpointer manual_thread(gpointer base);

static gpointer gst_nv_memory_map(GstMemory* mem,
                                  gsize maxsize,
                                  GstMapFlags flags) {
  (void)flags;
  (void)maxsize;
  int err = 0;
  GstNVManualMemory* nvmm_mem = (GstNVManualMemory*)mem;
  NvBufferParams params{};

  err = NvBufferGetParams(nvmm_mem->nvcam_buf->dmabuf_fd, &params);
  if (err) {
    GST_ERROR("NvBufferGetParams Failed");
    return nullptr;
  }

  return (gpointer)(params.nv_buffer);
}

static void gst_nv_memory_unmap(GstMemory* mem) {
  (void)mem;
  /* Nothing needs to be done */
}

static GstMemory* gst_nv_memory_share(GstMemory* mem,
                                      gssize offset,
                                      gssize size) {
  (void)mem;
  (void)offset;
  (void)size;
  g_assert_not_reached();
  return nullptr;
}

static GstMemory* gst_nv_memory_allocator_alloc(GstAllocator* allocator,
                                                gsize size,
                                                GstAllocationParams* params) {
  (void)size;
  (void)params;
  int err = 0;
  GstNVManualMemory* mem = nullptr;
  GstNvManualCameraSrcBuffer* nvbuf = nullptr;
  GstMemoryFlags flags = GST_MEMORY_FLAG_NO_SHARE;
  NvBufferParams param{};
  NvBufferCreateParams input_params{};

  GstNVManualMemoryAllocator* nvmm_allocator =
      GST_NVMEMORY_ALLOCATOR(allocator);
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)nvmm_allocator->owner;

  mem = g_slice_new0(GstNVManualMemory);
  // FIXME(mdegans): research whether it's ok to construct a struct with a
  //  unique_ptr and reset it like this.
  nvbuf = g_slice_new0(GstNvManualCameraSrcBuffer);

  {
    input_params.width = self->info.width;
    input_params.height = self->info.height;
    input_params.layout = NvBufferLayout_BlockLinear;
    input_params.colorFormat = NvBufferColorFormat_NV12;
    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.nvbuf_tag = NvBufferTag_CAMERA;

    err = NvBufferCreateEx(&nvbuf->dmabuf_fd, &input_params);
    if (err) {
      GST_ERROR("NvBufferCreateEx Failed");
      goto error;
    }

    err = NvBufferGetParams(nvbuf->dmabuf_fd, &param);
    if (err) {
      GST_ERROR("NvBufferGetParams Failed");
      goto getparam_failed;
    }

    gst_memory_init(GST_MEMORY_CAST(mem), flags, allocator, nullptr,
                    param.nv_buffer_size, 1 /* Alignment */, 0,
                    param.nv_buffer_size);
    mem->nvcam_buf = nvbuf;
  }
  return GST_MEMORY_CAST(mem);

getparam_failed :

{
  err = NvBufferDestroy(nvbuf->dmabuf_fd);
  if (err) {
    GST_ERROR("NvBufferDestroy Failed");
  }
}
error:
  g_slice_free(GstNvManualCameraSrcBuffer, nvbuf);
  g_slice_free(GstNVManualMemory, mem);

  return nullptr;
}

static void gst_nv_memory_allocator_free(GstAllocator* allocator,
                                         GstMemory* mem) {
  (void)allocator;
  GstNVManualMemory* nv_mem = (GstNVManualMemory*)mem;
  GstNvManualCameraSrcBuffer* nvbuf = nv_mem->nvcam_buf;

  int err = NvBufferDestroy(nvbuf->dmabuf_fd);
  if (err) {
    GST_ERROR("NvBufferDestroy Failed");
  }

  g_slice_free(GstNvManualCameraSrcBuffer, nvbuf);
  g_slice_free(GstNVManualMemory, nv_mem);
}

static void gst_nv_memory_allocator_class_init(
    GstNVManualMemoryAllocatorClass* klass) {
  GST_DEBUG_CATEGORY_INIT(gst_nvmanualcamerasrc_debug, "nvmanualcamerasrc", 0,
                          "nvmanualcamerasrc");

  GstAllocatorClass* allocator_class;
  allocator_class = (GstAllocatorClass*)klass;

  allocator_class->alloc = gst_nv_memory_allocator_alloc;
  allocator_class->free = gst_nv_memory_allocator_free;
}

static void gst_nv_memory_allocator_init(
    GstNVManualMemoryAllocator* allocator) {
  GstAllocator* alloc = GST_ALLOCATOR_CAST(allocator);

  alloc->mem_type = GST_NVMANUAL_MEMORY_TYPE;
  alloc->mem_map = gst_nv_memory_map;
  alloc->mem_unmap = gst_nv_memory_unmap;
  alloc->mem_share = gst_nv_memory_share;

  /* default copy & is_span */
  GST_OBJECT_FLAG_SET(allocator, GST_ALLOCATOR_FLAG_CUSTOM_ALLOC);
}

static void gst_nv_manual_camera_src_set_property(GObject* object,
                                                  guint prop_id,
                                                  const GValue* value,
                                                  GParamSpec* pspec);
static void gst_nv_manual_camera_src_get_property(GObject* object,
                                                  guint prop_id,
                                                  GValue* value,
                                                  GParamSpec* pspec);
static void gst_nv_manual_camera_src_finalize(GObject* object);

/* GObject vmethod implementations */

static GstCaps* gst_nv_manual_camera_fixate(GstBaseSrc* base, GstCaps* caps) {
  GstStructure* structure = nullptr;

  caps = gst_caps_make_writable(caps);

  structure = gst_caps_get_structure(caps, 0);

  gst_structure_fixate_field_nearest_int(structure, "width",
                                         nvmanualcam::defaults::DEFAULT_WIDTH);
  gst_structure_fixate_field_nearest_int(structure, "height",
                                         nvmanualcam::defaults::DEFAULT_HEIGHT);
  gst_structure_fixate_field_nearest_fraction(
      structure, "framerate", nvmanualcam::defaults::DEFAULT_FPS, 1);
  caps = GST_BASE_SRC_CLASS(gst_nv_manual_camera_src_parent_class)
             ->fixate(base, caps);

  return caps;
}

static gboolean gst_nv_manual_camera_set_caps(GstBaseSrc* base, GstCaps* caps) {
  GstCaps* old;
  GstNvManualCameraSrc* self = GST_NVMANUALCAMERASRC(base);
  // write own allocator here

  GST_INFO_OBJECT(self, "Received caps %" GST_PTR_FORMAT, caps);

  if (!gst_video_info_from_caps(&self->info, caps)) {
    GST_ERROR_OBJECT(self, "could not get GstVideoInfo from GstCaps");
    return FALSE;
  }

  GST_INFO_OBJECT(self, "CAPS: res: %dx%d fps: %d/%d", self->info.width,
                  self->info.height, self->info.fps_n, self->info.fps_d);

  // recalculate frame duration
  self->frame_duration = nvmanualcam::get_frame_duration(self->info);

  if ((old = self->outcaps) != caps) {
    if (caps)
      self->outcaps = gst_caps_copy(caps);
    else
      self->outcaps = nullptr;
    if (old)
      gst_caps_unref(old);
  }

  if (!self->controls->getBufApi()) {
    self->pool = gst_buffer_pool_new();
    GstNVManualMemoryAllocator* allocator =
        (GstNVManualMemoryAllocator*)g_object_new(
            gst_nv_memory_allocator_get_type(), nullptr);
    allocator->owner = self;
    GstStructure* config = gst_buffer_pool_get_config(self->pool);
    gst_buffer_pool_config_set_allocator(config, GST_ALLOCATOR(allocator),
                                         nullptr);

    gst_buffer_pool_config_set_params(config, nullptr, NvBufferGetSize(),
                                      MIN_BUFFERS, MAX_BUFFERS);
    gst_buffer_pool_set_config(self->pool, config);

    self->manual_buffers = g_queue_new();
    self->nvmm_buffers = g_queue_new();

    gst_buffer_pool_set_active(self->pool, TRUE);
  } else {
    self->pool = gst_nvds_buffer_pool_new();
    GstStructure* config = gst_buffer_pool_get_config(self->pool);
    gst_buffer_pool_config_set_params(
        config, self->outcaps, sizeof(NvBufSurface), MIN_BUFFERS, MAX_BUFFERS);
    gst_structure_set(config, "memtype", G_TYPE_INT, NVBUF_MEM_DEFAULT,
                      "gpu-id", G_TYPE_UINT, 0, "batch-size", G_TYPE_UINT, 1,
                      nullptr);
    gst_buffer_pool_set_config(self->pool, config);
    self->manual_buffers = g_queue_new();
    self->nvmm_buffers = g_queue_new();
    gst_buffer_pool_set_active(self->pool, TRUE);
  }

  self->consumer_thread =
      g_thread_new("consumer_thread", consumer_thread, self);

  self->manual_thread = g_thread_new("manual_thread", manual_thread, self);

  if (self->in_error) {
    return FALSE;
  }

  return TRUE;
}

static gboolean gst_nv_manual_camera_start(GstBaseSrc* base) {
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;
  self->stop_requested = FALSE;

  return TRUE;
}

static gboolean gst_nv_manual_camera_unlock(GstBaseSrc* base) {
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;

  self->unlock_requested = TRUE;

  return TRUE;
}

static gboolean gst_nv_manual_camera_unlock_stop(GstBaseSrc* base) {
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;

  self->unlock_requested = FALSE;

  return TRUE;
}

static gboolean gst_nv_manual_camera_stop(GstBaseSrc* base) {
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;
  bool success = true;

  self->stop_requested = true;
  if (!self->timeout) {
    Argus::ICaptureSession* sess =
        Argus::interface_cast<Argus::ICaptureSession*>(
            self->actx->iCaptureSession);
    if (sess) {
      sess->cancelRequests();
      sess->stopRepeat();
      sess->waitForIdle();
    } else {
      GST_ERROR_OBJECT(self, "Could not get iCaptureSession");
      success = false;
    }
  }
  g_mutex_lock(&self->eos_lock);
  g_cond_signal(&self->eos_cond);
  g_mutex_unlock(&self->eos_lock);
  gst_buffer_pool_set_active(self->pool, false);
  g_thread_join(self->manual_thread);
  return success;
}

static gpointer manual_thread(gpointer base) {
  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;

  int32_t cameraIndex = self->sensor_id;
  int32_t cameraMode = self->sensor_mode;
  int32_t secToRun = self->timeout;
  Argus::Size2D<uint32_t> streamSize(self->info.width, self->info.height);

  nvmanualcam::producer(cameraIndex, cameraMode, streamSize, secToRun,
                        controls);

  self->stop_requested = true;

  g_mutex_lock(&self->manual_buffers_queue_lock);
  g_cond_signal(&self->manual_buffers_queue_cond);
  g_mutex_unlock(&self->manual_buffers_queue_lock);

  g_mutex_lock(&self->nvmm_buffers_queue_lock);
  g_cond_signal(&self->nvmm_buffers_queue_cond);
  g_mutex_unlock(&self->nvmm_buffers_queue_lock);

  GST_DEBUG_OBJECT(self, "%s: stop_requested=%d", __func__,
                   self->stop_requested);
  return base;
}

static gpointer consumer_thread(gpointer base) {
  int err = 0;
  GstBuffer* buffer;
  GstMemory* mem;
  NvManualFrameInfo* consumerFrameInfo;
  GstFlowReturn ret;
  GstNVManualMemory* nv_mem = nullptr;

  GstNvManualCameraSrc* self = (GstNvManualCameraSrc*)base;

  while (!self->a_ctx->stop_requested) {
    g_mutex_lock(&self->manual_buffers_queue_lock);
    if (self->stop_requested) {
      g_mutex_unlock(&self->manual_buffers_queue_lock);
      goto done;
    }
    while (g_queue_is_empty(self->manual_buffers)) {
      g_cond_wait(&self->manual_buffers_queue_cond,
                  &self->manual_buffers_queue_lock);
    }

    consumerFrameInfo =
        (NvManualFrameInfo*)g_queue_pop_head(self->manual_buffers);
    g_mutex_unlock(&self->manual_buffers_queue_lock);
    if (&consumerFrameInfo->fd == nullptr) {
      goto done;
    }
    ret = gst_buffer_pool_acquire_buffer(self->pool, &buffer, nullptr);
    if (ret != GST_FLOW_OK) {
      if (!self->stop_requested) {
        GST_ERROR_OBJECT(self, "Error in pool acquire buffer");
      }
      goto done;
    }

    if (self->bufApi == FALSE) {
      mem = gst_buffer_peek_memory(buffer, 0);
      if (!mem) {
        GST_ERROR_OBJECT(self, "no memory block");
        goto done;
      }
      nv_mem = (GstNVManualMemory*)mem;
      if (self->controls->getMetaEnabled()) {
        g_assert(consumerFrameInfo->metadata);
        nvmanualcam::attachMetadata(std::move(consumerFrameInfo->metadata),
                                    buffer);
      }
      err =
          NvBufferTransform(consumerFrameInfo->fd, nv_mem->nvcam_buf->dmabuf_fd,
                            &self->transform_params);
      g_mutex_lock(&self->manual_buffer_consumed_lock);
      g_cond_signal(&self->manual_buffer_consumed_cond);
      self->is_manual_buffer_consumed = TRUE;
      g_mutex_unlock(&self->manual_buffer_consumed_lock);
      if (err) {
        GST_ERROR_OBJECT(self, "NvBufferTransform Failed");
        /* TODO: Check if need to set ->stop_requested flag in error condition
         */
        goto done;
      }
    } else {
      mem = gst_buffer_peek_memory(buffer, 0);
      GstMapInfo outmap = GST_MAP_INFO_INIT;
      if (!mem) {
        GST_ERROR_OBJECT(self, "no memory block");
        goto done;
      }
      gst_buffer_map(buffer, &outmap, GST_MAP_WRITE);
      NvBufSurface* surf = (NvBufSurface*)outmap.data;

      err = NvBufferTransform(consumerFrameInfo->fd,
                              (gint)surf->surfaceList[0].bufferDesc,
                              &self->transform_params);
      g_mutex_lock(&self->manual_buffer_consumed_lock);
      g_cond_signal(&self->manual_buffer_consumed_cond);
      self->is_manual_buffer_consumed = TRUE;
      g_mutex_unlock(&self->manual_buffer_consumed_lock);
      surf->numFilled = 1;
      if (err) {
        GST_ERROR_OBJECT(self, "NvBufferTransform Failed");
        /* TODO: Check if need to set ->stop_requested flag in error condition
         */
        goto done;
      }

      gst_buffer_unmap(buffer, &outmap);
    }

    g_queue_push_tail(self->nvmm_buffers, buffer);

    g_mutex_lock(&self->nvmm_buffers_queue_lock);
    g_cond_signal(&self->nvmm_buffers_queue_cond);
    g_mutex_unlock(&self->nvmm_buffers_queue_lock);
  }
done:
  GST_DEBUG_OBJECT(self, "%s: stop_requested=%d", __func__,
                   self->stop_requested);
  if (buffer) {
    gst_mini_object_set_qdata(GST_MINI_OBJECT_CAST(buffer),
                              nvmanualcam::Metadata::quark(), nullptr, nullptr);
  }
  return nullptr;
}

static GstFlowReturn gst_nv_manual_camera_create(GstBaseSrc* base,
                                                 guint64 offset,
                                                 guint size,
                                                 GstBuffer** buf) {
  (void)offset;
  (void)size;
  GstNvManualCameraSrc* self = GST_NVMANUALCAMERASRC(base);
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer* gst_buf = nullptr;

  if (self->stop_requested || self->unlock_requested)
    return GST_FLOW_EOS;

  g_mutex_lock(&self->nvmm_buffers_queue_lock);

  while (!self->stop_requested && !self->unlock_requested &&
         g_queue_is_empty(self->nvmm_buffers)) {
    g_cond_wait(&self->nvmm_buffers_queue_cond, &self->nvmm_buffers_queue_lock);
  }

  if (self->stop_requested || self->unlock_requested) {
    g_mutex_unlock(&self->nvmm_buffers_queue_lock);
    return GST_FLOW_EOS;
  }

  gst_buf = (GstBuffer*)g_queue_pop_head(self->nvmm_buffers);
  *buf = gst_buf;

  g_mutex_unlock(&self->nvmm_buffers_queue_lock);

  return ret;
}

/* initialize the nvmanualcamerasrc's class */
static void gst_nv_manual_camera_src_class_init(
    GstNvManualCameraSrcClass* klass) {
  GObjectClass* gobject_class;
  GstElementClass* gstelement_class;
  GstBaseSrcClass* base_src_class;

  gobject_class = (GObjectClass*)klass;
  gstelement_class = (GstElementClass*)klass;
  base_src_class = (GstBaseSrcClass*)klass;

  gobject_class->set_property = gst_nv_manual_camera_src_set_property;
  gobject_class->get_property = gst_nv_manual_camera_src_get_property;
  gobject_class->finalize = gst_nv_manual_camera_src_finalize;

  base_src_class->set_caps = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_set_caps);
  base_src_class->fixate = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_fixate);
  base_src_class->start = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_start);
  base_src_class->stop = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_stop);
  base_src_class->create = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_create);
  base_src_class->unlock = GST_DEBUG_FUNCPTR(gst_nv_manual_camera_unlock);
  base_src_class->unlock_stop =
      GST_DEBUG_FUNCPTR(gst_nv_manual_camera_unlock_stop);

  Properties::install(gobject_class);

  g_object_class_install_property(
      gobject_class, PROP_WHITE_BALANCE,
      g_param_spec_enum(
          "wbmode", "white balance mode", "Argus white balance preset.",
          GST_TYPE_NVMANUALCAM_WB_MODE, nvmanualcam::defaults::WB_MODE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_SATURATION,
      g_param_spec_float(
          "saturation", "saturation",
          "0.0 is b&w, 1.0 == normal, 2.0 == \"Fujify\" me.", 0.0, 2.0,
          nvmanualcam::defaults::SATURATION,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_SILENT,
      g_param_spec_boolean(
          "silent", "Silent", "Less logging when true. Mostly unused.",
          nvmanualcam::defaults::SILENT,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_TIMEOUT,
      g_param_spec_uint(
          "timeout", "Timeout",
          "timeout to capture in seconds (Either specify timeout "
          "or num-buffers, not both)",
          0, G_MAXINT, nvmanualcam::defaults::TIMEOUT,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_SENSOR_ID,
      g_param_spec_int(
          "sensor-id", "Sensor ID", "Set the id of camera sensor to use.", 0,
          G_MAXUINT8, nvmanualcam::defaults::SENSOR_ID,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_SENSOR_MODE,
      g_param_spec_int(
          "sensor-mode", "Sensor Mode",
          "Camera sensor mode to use. (-1 uses the default, which is \?\?\?)",
          -1, G_MAXUINT8, nvmanualcam::defaults::SENSOR_MODE_STATE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_TOTAL_SENSOR_MODES,
      g_param_spec_int(
          "total-sensor-modes", "Total Sensor Modes",
          "Query the number of sensor modes available.", 0, G_MAXUINT8,
          nvmanualcam::defaults::TOTAL_SENSOR_MODES,
          (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_EXPOSURE_TIME,
      g_param_spec_float(
          "exposuretime", "Exposure Time", "Exposure time in *frames*",
          MIN_EXPOSURE_TIME, MAX_EXPOSURE_TIME,
          nvmanualcam::defaults::EXPOSURE_TIME,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_EXPOSURE_REAL,
      g_param_spec_uint64(
          "exposurereal", "Real Exposure Time", "Actual exposure time in ns.",
          0, G_MAXUINT64, 0,  // default is 0 becase this is a read-only prop
          (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_GAIN,
      g_param_spec_float(
          "gain", "Analog Gain", "Set/get analog gain.", MIN_GAIN, MAX_GAIN,
          nvmanualcam::defaults::GAIN,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_DIGITAL_GAIN,
      g_param_spec_float(
          "digitalgain", "Digital Gain", "Set/get ISP digital gain.",
          MIN_DIGITAL_GAIN, MAX_DIGITAL_GAIN,
          nvmanualcam::defaults::DIGITAL_GAIN,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_TNR_STRENGTH,
      g_param_spec_float(
          "tnr-strength", "TNR Strength",
          "property to adjust temporal noise reduction strength", -1.0, 1.0,
          nvmanualcam::defaults::TNR_STRENGTH,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_TNR_MODE,
      g_param_spec_enum(
          "tnr-mode", "TNR mode",
          "property to select temporal noise reduction mode",
          GST_TYPE_NVMANUALCAM_TNR_MODE, nvmanualcam::defaults::TNR_MODE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_EDGE_ENHANCEMENT_STRENGTH,
      g_param_spec_float(
          "ee-strength", "TNR Strength",
          "property to adjust edge enhancement strength", -1.0, 1.0,
          nvmanualcam::defaults::TNR_STRENGTH,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_EDGE_ENHANCEMENT_MODE,
      g_param_spec_enum(
          "ee-mode", "Edge Enhancement",
          "property to select edge enhnacement mode",
          GST_TYPE_NVMANUALCAM_EDGE_ENHANCEMENT_MODE,
          nvmanualcam::defaults::EE_MODE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_AEANTIBANDING_MODE,
      g_param_spec_enum(
          "aeantibanding", "Auto Exposure Antibanding Mode",
          "property to set the auto exposure antibanding mode",
          GST_TYPE_NVMANUALCAM_AEANTIBANDING_MODE,
          nvmanualcam::defaults::AEANTIBANDING_MODE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_EXPOSURE_COMPENSATION,
      g_param_spec_float(
          "exposurecompensation", "Exposure Compensation",
          "property to adjust exposure compensation", -2.0, 2.0,
          nvmanualcam::defaults::EXP_COMPENSATION,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_AE_LOCK,
      g_param_spec_boolean(
          "aelock", "AE Lock", "set or unset the auto exposure lock",
          nvmanualcam::defaults::AE_LOCK, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(
      gobject_class, PROP_AWB_LOCK,
      g_param_spec_boolean(
          "awblock", "AWB Lock", "set or unset the auto white balance lock",
          nvmanualcam::defaults::AWB_LOCK, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(
      gobject_class, PROP_METADATA,
      g_param_spec_boolean(
          "metadata", "Generate metadata",
          "Generate and attach Argus::CaptureMetadata.",
          nvmanualcam::defaults::METADATA,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property(
      gobject_class, PROP_BAYER_SHARPNESS_MAP,
      g_param_spec_boolean(
          "bayer-sharpness-map", "Bayer Sharpness Map",
          "Generate and attach Argus::IBayerSharpnessMap metadata.",
          nvmanualcam::defaults::BAYER_SHARPNESS_MAP,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY)));

  gst_element_class_set_details_simple(
      gstelement_class, "NvManualCameraSrc", "Video/Capture",
      "nVidia MANUAL Camera Source",
      "Viranjan Pagar <vpagar@nvidia.com>, Amit Pandya <apandya@nvidia.com>");

  gst_element_class_add_pad_template(gstelement_class,
                                     gst_static_pad_template_get(&src_factory));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void gst_nv_manual_camera_src_init(GstNvManualCameraSrc* self) {
  self->stop_requested = false;
  self->unlock_requested = false;
  self->outcaps = nullptr;

  self->in_error = false;

  self->controls = std::make_shared<_GstNvManualCameraSrc::Controls>();
  self->controls->setFrameDuration(nvmanualcam::get_frame_duration(self->info));

  g_mutex_init(&self->manual_buffers_queue_lock);
  g_cond_init(&self->manual_buffers_queue_cond);

  g_mutex_init(&self->manual_buffer_consumed_lock);
  g_cond_init(&self->manual_buffer_consumed_cond);

  g_mutex_init(&self->nvmm_buffers_queue_lock);
  g_cond_init(&self->nvmm_buffers_queue_cond);

  memset(&self->transform_params, 0, sizeof(NvBufferTransformParams));

  g_mutex_init(&self->eos_lock);
  g_cond_init(&self->eos_cond);

  gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp(GST_BASE_SRC(self), TRUE);
}

static void gst_nv_manual_camera_src_finalize(GObject* object) {
  GstNvManualCameraSrc* self = GST_NVMANUALCAMERASRC(object);
  GST_DEBUG_OBJECT(self, "cleaning up");

  g_mutex_clear(&self->nvmm_buffers_queue_lock);
  g_cond_clear(&self->nvmm_buffers_queue_cond);

  g_mutex_clear(&self->manual_buffers_queue_lock);
  g_cond_clear(&self->manual_buffers_queue_cond);

  g_mutex_clear(&self->manual_buffer_consumed_lock);
  g_cond_clear(&self->manual_buffer_consumed_cond);

  g_mutex_clear(&self->eos_lock);
  g_cond_clear(&self->eos_cond);

  self->controls.reset();
}

static bool valid_id(GObject* object, guint id, GParamSpec* pspec) {
  if (id >= static_cast<guint>(Properties::ID::SENTINEL)) {
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, id, pspec);
    return false;
  }
  return true;
}

static void gst_nv_manual_camera_src_set_property(GObject* object,
                                                  guint id,
                                                  const GValue* value,
                                                  GParamSpec* pspec) {
  auto self = GST_NVMANUALCAMERASRC(object);

  g_assert(valid_id(object, id, pspec));

  g_assert(self->controls->set_property(static_cast<Properties::ID>(id), value,
                                        pspec));
}

static void gst_nv_manual_camera_src_get_property(GObject* object,
                                                  guint id,
                                                  GValue* value,
                                                  GParamSpec* pspec) {
  auto self = GST_NVMANUALCAMERASRC(object);

  g_assert(valid_id(object, id, pspec));

  g_assert(self->controls->get_property(static_cast<Properties::ID>(id), value,
                                        pspec));
}

G_END_DECLS
