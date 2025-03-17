/*
 * Copyright (c) 2024 Simone Rossetto <simros85@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__ESP_WIREGUARD_LOG__H__)
#define __ESP_WIREGUARD_LOG__H__

#if defined(ESP8266) && !defined(IDF_VER)

// do nothing, simply prevent compilation errors if ESP_* macros are missing
#define _noop(x, ...) do {} while(0)

#ifndef ESP_LOGE
#define ESP_LOGE(tag, ...) _noop(tag, __VA_ARGS__)
#endif

#ifndef ESP_LOGW
#define ESP_LOGW(tag, ...) _noop(tag, __VA_ARGS__)
#endif

#ifndef ESP_LOGI
#define ESP_LOGI(tag, ...) _noop(tag, __VA_ARGS__)
#endif

#ifndef ESP_LOGD
#define ESP_LOGD(tag, ...) _noop(tag, __VA_ARGS__)
#endif

#ifndef ESP_LOGV
#define ESP_LOGV(tag, ...) _noop(tag, __VA_ARGS__)
#endif

#elif defined(LIBRETINY)
#include <libretiny.h>
#else // defined(LIBRETINY)
#include <esp_log.h>
#endif // defined(ESP8266) && !defined(IDF_VER)

#endif  // __ESP_WIREGUARD_LOG__H__
