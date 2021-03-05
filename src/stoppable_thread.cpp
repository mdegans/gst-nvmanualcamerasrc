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

#include "stoppable_thread.hpp"

#include "Error.h"

#include <unistd.h>  // for useconds_t

namespace nvmanualcam::utils {

StoppableThread::StoppableThread()
    : m_doShutdown(false),
      m_threadID(0),
      m_threadState(THREAD_INACTIVE)

{}

StoppableThread::~StoppableThread() {
  (void)shutdown();
}

bool StoppableThread::initialize(GstNvManualCameraSrc* src) {
  if (m_threadID)
    return true;

  this->src = src;

  if (pthread_create(&m_threadID, nullptr, threadFunctionStub, this) != 0)
    ORIGINATE_ERROR("Failed to create thread.");

  // wait for the thread to start up
  while (m_threadState == THREAD_INACTIVE)
    usleep(100);

  return true;
}

bool StoppableThread::shutdown() {
  if (m_threadID) {
    m_doShutdown = true;
    if (pthread_join(m_threadID, nullptr) != 0)
      ORIGINATE_ERROR("Failed to join thread");
    m_threadID = 0;
    m_doShutdown = false;
    m_threadState = THREAD_INACTIVE;
  }

  return true;
}

bool StoppableThread::waitRunning(useconds_t timeoutUs) {
  // Can only wait for a thread which is initializing or already running
  if ((m_threadState != THREAD_INITIALIZING) &&
      (m_threadState != THREAD_RUNNING))
    ORIGINATE_ERROR("Invalid thread state %d", m_threadState.get());

  // wait for the thread to run
  const useconds_t sleepTimeUs = 100;
  while (m_threadState != THREAD_RUNNING) {
    usleep(sleepTimeUs);
#ifdef DEBUG
    // in debug mode wait indefinitely
#else
    if (timeoutUs < sleepTimeUs)
      return false;
    timeoutUs -= sleepTimeUs;
#endif
  }

  return true;
}

/**
 * Thread function stub, calls the real thread function.
 *
 * @param [in] dataPtr  Pointer to user data
 */
/* static */ void* StoppableThread::threadFunctionStub(void* dataPtr) {
  StoppableThread* thread = static_cast<StoppableThread*>(dataPtr);

  if (!thread->threadFunction(thread->src))
    thread->m_threadState = StoppableThread::THREAD_FAILED;
  else
    thread->m_threadState = StoppableThread::THREAD_DONE;

  return nullptr;
}

/**
 * Thread function
 */
bool StoppableThread::threadFunction(GstNvManualCameraSrc* src) {
  m_threadState = THREAD_INITIALIZING;

  PROPAGATE_ERROR(threadInitialize(src));

  m_threadState = THREAD_RUNNING;

  while (!m_doShutdown) {
    PROPAGATE_ERROR(threadExecute(src));
  }

  PROPAGATE_ERROR(threadShutdown(src));

  return true;
}

};  // namespace nvmanualcam::utils