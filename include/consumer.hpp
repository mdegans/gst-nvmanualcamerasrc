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

#ifndef BA611933_ED6C_4DCF_9AF1_2FFAA6CFCA31
#define BA611933_ED6C_4DCF_9AF1_2FFAA6CFCA31

#include "stoppable_thread.hpp"

#include <Argus/Argus.h>
#include <EGLStream/FrameConsumer.h>

namespace nvmanualcam::utils {

/*******************************************************************************
 * Consumer thread:
 *   Creates a Consumer object to read frames from the OutputStream just
 *tests for sanity.
 ******************************************************************************/
class Consumer : public nvmanualcam::utils::StoppableThread {
 public:
  explicit Consumer(Argus::OutputStream* stream) : m_stream(stream) {}
  ~Consumer() {}

 private:
  /** @name Thread methods */
  /**@{*/
  virtual bool threadInitialize(GstNvManualCameraSrc*);
  virtual bool threadExecute(GstNvManualCameraSrc*);
  virtual bool threadShutdown(GstNvManualCameraSrc*);
  /**@}*/

  Argus::OutputStream* m_stream;
  // GstNvManualCameraSrc *manual_src;
  Argus::UniqueObj<EGLStream::FrameConsumer> m_consumer;
};

}  // namespace nvmanualcam::utils

#endif /* BA611933_ED6C_4DCF_9AF1_2FFAA6CFCA31 */
