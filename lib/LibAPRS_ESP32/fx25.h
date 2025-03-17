/*
Copyright 2020-2023 Piotr Wilkon

This file is part of VP-Digi.

VP-Digi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

VP-Digi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with VP-Digi.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FX25_H_
#define FX25_H_

#ifdef ENABLE_FX25

#include <stdint.h>
#include <stdbool.h>

#define FX25_MAX_BLOCK_SIZE 255

struct Fx25Mode
{
	uint64_t tag; //correlation tag
	uint16_t K; //data size
	uint8_t T; //parity check size
};

extern const struct Fx25Mode Fx25ModeList[11];

/**
 * @brief Get FX.25 mode for given correlation tag
 * @param tag FX.25 correlation tag
 * @return FX.25 mode structure pointer or NULL if not a FX.25 tag
 */
const struct Fx25Mode* Fx25GetModeForTag(uint64_t tag);

/**
 * @brief Get FX.25 mode for given payload size
 * @param size Payload size including flags and CRC
 * @return FX.25 mode structure pointer or NULL if standard AX.25 must be used
 */
const struct Fx25Mode* Fx25GetModeForSize(uint16_t size);

/**
 * @brief Encode AX.25 message in FX.25
 * @param *buffer AX.25 message (bit-stuffed, with CRC and padding)
 * @param *mode FX.25 mode
 */
void Fx25Encode(uint8_t *buffer, const struct Fx25Mode *mode);

/**
 * @brief Decode/fix FX.25 packet
 * @param *buffer Input buffer
 * @param *mode FX.25 mode
 * @param *fixed Number of bytes fixed
 * @return True if message is valid, false if uncorrectable
 */
bool Fx25Decode(uint8_t *buffer, const struct Fx25Mode *mode, uint8_t *fixed);

/**
 * @brief Initialize FX.25 module
 */
void Fx25Init(void);

#endif

#endif /* FX25_H_ */