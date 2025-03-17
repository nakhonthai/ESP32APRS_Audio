/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
 * Copyright (c) 2023-2024 Simone Rossetto <simros85@gmail.com>
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

#include "wireguard-platform.h"

#include <stdlib.h>
#include <time.h>
#include <inttypes.h>

#include "lwip/sys.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/version.h"

#if defined(ESP8266) && !defined(IDF_VER)
#include <osapi.h>
#define esp_fill_random(out, size) os_get_random(out, size)
#elif defined(LIBRETINY)
#include <libretiny.h>
#define esp_fill_random(out, size) lt_rand_bytes(out, size)
#else // defined(LIBRETINY)
#include <esp_system.h>
#endif // defined(ESP8266) && !defined(IDF_VER)

#include "esp_wireguard_err.h"
#include "esp_wireguard_log.h"
#include "crypto.h"

#define ENTROPY_MINIMUM_REQUIRED_THRESHOLD	(134)
#define ENTROPY_FUNCTION_DATA	NULL
#define ENTROPY_CUSTOM_DATA		NULL
#define ENTROPY_CUSTOM_DATA_LENGTH (0)
#define TAG "wireguard-platform"

#if MBEDTLS_VERSION_NUMBER >= 0x020D0000
static struct mbedtls_ctr_drbg_context random_context;
static struct mbedtls_entropy_context entropy_context;
#else
static mbedtls_ctr_drbg_context random_context;
static mbedtls_entropy_context entropy_context;
#endif

static int entropy_hw_random_source( void *data, unsigned char *output, size_t len, size_t *olen ) {
	esp_fill_random(output, len);
	*olen = len;
	return 0;
}

esp_err_t wireguard_platform_init() {
	int mbedtls_err;
	esp_err_t err;

	mbedtls_entropy_init(&entropy_context);
	mbedtls_ctr_drbg_init(&random_context);
	mbedtls_err = mbedtls_entropy_add_source(
			&entropy_context,
			entropy_hw_random_source,
			ENTROPY_FUNCTION_DATA,
			ENTROPY_MINIMUM_REQUIRED_THRESHOLD,
			MBEDTLS_ENTROPY_SOURCE_STRONG);
	if (mbedtls_err != 0) {
		ESP_LOGE(TAG, "mbedtls_entropy_add_source: %i", mbedtls_err);
		err = ESP_ERR_HW_CRYPTO_BASE;
		goto fail;
	}
	mbedtls_err = mbedtls_ctr_drbg_seed(
			&random_context,
			mbedtls_entropy_func,
			&entropy_context,
			ENTROPY_CUSTOM_DATA,
			ENTROPY_CUSTOM_DATA_LENGTH);
	if (mbedtls_err != 0) {
		ESP_LOGE(TAG, "mbedtls_ctr_drbg_seed: %i", mbedtls_err);
		err = ESP_ERR_INVALID_CRC;
		goto fail;
	}
	err = ESP_OK;
fail:
	return err;
}

void wireguard_random_bytes(void *bytes, size_t size) {
	mbedtls_ctr_drbg_random(&random_context, bytes, size);
}

uint32_t wireguard_sys_now() {
	// Default to the LwIP system time
	return sys_now();
}

void wireguard_tai64n_now(uint8_t *output) {
	// See https://cr.yp.to/libtai/tai64.html
	// 64 bit seconds from 1970 = 8 bytes
	// 32 bit nano seconds from current second

	struct timeval tv;
	gettimeofday(&tv, NULL);

	uint64_t seconds = 0x400000000000000aULL + tv.tv_sec;
	uint32_t nanos = tv.tv_usec * 1000;
	U64TO8_BIG(output + 0, seconds);
	U32TO8_BIG(output + 8, nanos);
}

bool wireguard_is_under_load() {
	return false;
}
// vim: noexpandtab
