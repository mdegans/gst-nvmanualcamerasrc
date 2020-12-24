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

#include "gstnvarguscamera_utils.h"
#include <string.h>

GType gst_nvarguscam_white_balance_mode_get_type(void)
{
  static gsize white_balance_type = 0;
  static const GEnumValue white_balance_mode[] = {
      {NvArgusCamAwbMode_Off, "auto white balance off", "off"},
      {NvArgusCamAwbMode_Auto, "auto white balance on", "auto"},
      {NvArgusCamAwbMode_Incandescent, "incandescent white balance preset",
       "incandescent"},
      {NvArgusCamAwbMode_Fluorescent, "fluorescent white balance preset", "fluorescent"},
      {NvArgusCamAwbMode_WarmFluorescent, "warm-fluorescent white balance preset",
       "warm-fluorescent"},
      {NvArgusCamAwbMode_Daylight, "foo white balance preset", "daylight"},
      {NvArgusCamAwbMode_CloudyDaylight, "cloudy-daylight white balance preset",
       "cloudy-daylight"},
      {NvArgusCamAwbMode_Twilight, "twilight white balance preset", "twilight"},
      {NvArgusCamAwbMode_Shade, "shade white balance preset", "shade"},
      {NvArgusCamAwbMode_Manual, "manual white balance preset", "manual"},
      {0, NULL, NULL}};

  if (g_once_init_enter(&white_balance_type))
  {
    GType tmp = g_enum_register_static("GstNvArgusCamWBMode", white_balance_mode);
    g_once_init_leave(&white_balance_type, tmp);
  }

  return (GType)white_balance_type;
}

GType gst_nvarguscam_tnr_mode_get_type(void)
{
  static gsize tnr_type = 0;
  static const GEnumValue tnr_mode[] = {
      {NvArgusCamNoiseReductionMode_Off, "Noise reduction off", "off"},
      {NvArgusCamNoiseReductionMode_Fast, "Fast noise reduction", "fast"},
      {NvArgusCamNoiseReductionMode_HighQuality, "HQ noise reduction", "hq"},
      {0, NULL, NULL}};

  if (g_once_init_enter(&tnr_type))
  {
    GType tmp = g_enum_register_static("GstNvArgusCamTNRMode", tnr_mode);
    g_once_init_leave(&tnr_type, tmp);
  }
  return (GType)tnr_type;
}

GType gst_nvarguscam_edge_enhancement_mode_get_type(void)
{
  static gsize edge_enhancement_type = 0;
  static const GEnumValue edge_enhancement_mode[] = {
      {NvArgusCamEdgeEnhancementMode_Off, "Edge enhancement off", "off"},
      {NvArgusCamEdgeEnhancementMode_Fast, "Fast edge enhancement", "fast"},
      {NvArgusCamEdgeEnhancementMode_HighQuality, "HQ edge enhancement", "hq"},
      {0, NULL, NULL}};

  if (g_once_init_enter(&edge_enhancement_type))
  {
    GType tmp = g_enum_register_static("GstNvArgusCamEEMode", edge_enhancement_mode);
    g_once_init_leave(&edge_enhancement_type, tmp);
  }
  return (GType)edge_enhancement_type;
}

GType gst_nvarguscam_aeantibanding_mode_get_type(void)
{
  static gsize aeantibanding_type = 0;
  static const GEnumValue aeantibanding_mode[] = {
      {NvArgusCamAeAntibandingMode_Off, "Anti-banding off", "off"},
      {NvArgusCamAeAntibandingMode_Auto, "Automatic anti-banding", "auto"},
      {NvArgusCamAeAntibandingMode_50HZ, "50hz anti-banding", "50hz"},
      {NvArgusCamAeAntibandingMode_60HZ, "60hz anti-banding", "60hz"},
      {0, NULL, NULL}};

  if (g_once_init_enter(&aeantibanding_type))
  {
    GType tmp = g_enum_register_static("GstNvArgusCamAeAntiBandingMode", aeantibanding_mode);
    g_once_init_leave(&aeantibanding_type, tmp);
  }
  return (GType)aeantibanding_type;
}