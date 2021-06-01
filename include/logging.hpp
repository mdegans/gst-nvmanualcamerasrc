/*
 * Copyright (c) 2021 Michael de Gans
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef C7D133B0_CD0B_4AE7_98E3_DBA51825D323
#define C7D133B0_CD0B_4AE7_98E3_DBA51825D323

#include <gst/gst.h>

/**
 * @brief If the return of `stmt` is nonzero, logs the returned code with
 * GST_ERROR.
 *
 */
#define NONZERO_ERROR(stmt)                                                \
  do {                                                                     \
    auto code = (stmt);                                                    \
    while (code) {                                                         \
      GST_ERROR("\"%s\" returned code:%d", #stmt, static_cast<int>(code)); \
      break;                                                               \
    }                                                                      \
  } while (0)

/**
 * @brief If `stmt` evaluates nonzero, sets src->stop_requested to
 * TRUE, src->in_error to TRUE.
 *
 * Only works in a context with a GstNvManualCamera* named `src`.
 *
 */
#define NONZERO_STOP_SRC(stmt)                                     \
  do {                                                             \
    auto code = (stmt);                                            \
    while (code) {                                                 \
      GST_ERROR("STOPPING:because:\"%s\" returned code:%d", #stmt, \
                static_cast<int>(code));                           \
      src->stop_requested = TRUE;                                  \
      src->in_error = TRUE;                                        \
      break;                                                       \
    }                                                              \
  } while (0)

/**
 * @brief logs to GST_ERROR_OBJ, sets src->stop_requested to true, and
 * src->in_error to true;
 *
 * Only works in a context with a GstNvManualCamera* named `src`.
 *
 */
#define STOP_SRC(...)                   \
  do {                                  \
    GST_ERROR_OBJECT(src, __VA_ARGS__); \
    src->stop_requested = TRUE;         \
    src->in_error = TRUE;               \
  } while (0)

/**
 * @brief If the return of `stmt` is nonzero, logs the returned code with
 * GST_ERROR and returns false.
 *
 */
#define NONZERO_RETURN_FALSE(stmt)                                         \
  do {                                                                     \
    auto code = (stmt);                                                    \
    while (code) {                                                         \
      GST_ERROR("\"%s\" returned code:%d", #stmt, static_cast<int>(code)); \
      return false;                                                        \
    }                                                                      \
  } while (0)

/**
 * @brief If the return of `stmt` is nonzero, logs the returned code with
 * GST_ERROR and calls std::terminate().
 *
 */
#define NONZERO_PANIC(stmt)                              \
  do {                                                   \
    auto code = (stmt);                                  \
    while (code) {                                       \
      GST_ERROR("FATAL: \"%s\" returned code:%d", #stmt, \
                static_cast<int>(code));                 \
      std::terminate();                                  \
      break;                                             \
    }                                                    \
  } while (0)

/**
 * @brief If the return of `stmt` is nonzero, logs the returned code with
 * GST_WARNING.
 *
 */
#define NONZERO_WARNING(stmt)                                                \
  do {                                                                       \
    auto code = (stmt);                                                      \
    while (code) {                                                           \
      GST_WARNING("\"%s\" returned code:%d", #stmt, static_cast<int>(code)); \
      break;                                                                 \
    }                                                                        \
  } while (0)

#endif /* C7D133B0_CD0B_4AE7_98E3_DBA51825D323 */
