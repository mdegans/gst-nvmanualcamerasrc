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

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */

#include "gstnvmanualcamera_config.h"
#include "gstnvmanualcamerasrc.hpp"

#include <gst/base/gstbasesrc.h>
#include <gst/gst.h>

static gboolean nvmanualcamerasrc_init(GstPlugin* nvmanualcamerasrc) {
  /* debug category for fltering log messages
   *
   * exchange the string 'Template nvmanualcamerasrc' with your description
   */
  return gst_element_register(nvmanualcamerasrc, "nvmanualcamerasrc",
                              GST_RANK_PRIMARY, GST_TYPE_NVMANUALCAMERASRC);
}

/* gstreamer looks for this structure to register nvmanualcamerasrcs
 *
 * to change this:
 *  * Gnu Make: modify include/gstnvmanualcamera_config_make.h
 *  * Meson: change the VERSION file to set the version and the root meson.build
 *    for the rest.
 *
 */
GST_PLUGIN_DEFINE(GST_VERSION_MAJOR,
                  GST_VERSION_MINOR,
                  nvmanualcamerasrc,
                  PROJ_DESCRIPTION,
                  nvmanualcamerasrc_init,
                  PROJ_VER,
                  LICENSE,
                  BINARY_PACKAGE,
                  PROJ_URL)
