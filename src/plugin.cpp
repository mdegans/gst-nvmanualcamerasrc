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

#include "gstnvarguscamera_config.h"
#include "gstnvarguscamerasrc.hpp"

#include <gst/gst.h>
#include <gst/base/gstbasesrc.h>

GST_DEBUG_CATEGORY_STATIC(gst_nv_argus_camera_src_debug);
#define GST_CAT_DEFAULT gst_nv_argus_camera_src_debug

static gboolean
nvarguscamerasrc_init(GstPlugin *nvarguscamerasrc)
{
    /* debug category for fltering log messages
   *
   * exchange the string 'Template nvarguscamerasrc' with your description
   */
    GST_DEBUG_CATEGORY_INIT(gst_nv_argus_camera_src_debug, "nvarguscamerasrc",
                            0, "nvarguscamerasrc");

    return gst_element_register(nvarguscamerasrc, "nvarguscamerasrc", GST_RANK_PRIMARY,
                                GST_TYPE_NVARGUSCAMERASRC);
}

/* gstreamer looks for this structure to register nvarguscamerasrcs
 *
 * to change this:
 *  * Gnu Make: modify include/gstnvarguscamera_config_make.h
 *  * Meson: change the VERSION file to set the version and the root meson.build
 *    for the rest.
 * 
 */
GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    nvarguscamerasrc,
    PROJ_DESCRIPTION,
    nvarguscamerasrc_init,
    PROJ_VER,
    LICENSE,
    BINARY_PACKAGE,
    PROJ_URL)
