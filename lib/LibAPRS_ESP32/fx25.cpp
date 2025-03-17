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

#ifdef ENABLE_FX25
#include <Arduino.h>
#include "fx25.h"
#include <stddef.h>
#include "rs.h"

#define FX25_RS_FCR 1

#define FX25_PREGENERATE_POLYS
#define FX25_MAX_DISTANCE 10 //maximum Hamming distance when comparing tags

const struct Fx25Mode Fx25ModeList[11] =
{
	{.tag =  0xB74DB7DF8A532F3E, .K = 239, .T = 16},
	{.tag =  0x26FF60A600CC8FDE, .K = 128, .T = 16},
	{.tag =  0xC7DC0508F3D9B09E, .K = 64, .T = 16},
	{.tag =  0x8F056EB4369660EE, .K = 32, .T = 16},
	{.tag =  0x6E260B1AC5835FAE, .K = 223, .T = 32},
	{.tag =  0xFF94DC634F1CFF4E, .K = 128, .T = 32},
	{.tag =  0x1EB7B9CDBC09C00E, .K = 64, .T = 32},
	{.tag =  0xDBF869BD2DBB1776, .K = 32, .T = 32},
	{.tag =  0x3ADB0C13DEAE2836, .K = 191, .T = 64},
	{.tag =  0xAB69DB6A543188D6, .K = 128, .T = 64},
	{.tag =  0x4A4ABEC4A724B796, .K = 64, .T = 64}
};



const struct Fx25Mode* Fx25GetModeForTag(uint64_t tag)
{
	for(uint8_t i = 0; i < sizeof(Fx25ModeList) / sizeof(*Fx25ModeList); i++)
	{
		if(__builtin_popcountll(tag ^ Fx25ModeList[i].tag) <= FX25_MAX_DISTANCE)
			return &Fx25ModeList[i];
	}
	return NULL;
}

const struct Fx25Mode* Fx25GetModeForSize(uint16_t size)
{
	//use "UZ7HO Soundmodem standard" for choosing FX.25 mode
	if(size <= 32)
		return &Fx25ModeList[3];
	else if(size <= 64)
		return &Fx25ModeList[2];
	else if(size <= 128)
		return &Fx25ModeList[5];
	else if(size <= 191)
		return &Fx25ModeList[8];
	else if(size <= 223)
		return &Fx25ModeList[4];
	else if(size <= 239)
		return &Fx25ModeList[0];
	else
		return NULL; //frame too big, do not use FX.25
}

#ifdef FX25_PREGENERATE_POLYS
static struct LwFecRS rs16, rs32, rs64;
#else
static struct LwFecRS rs;
#endif

void Fx25Encode(uint8_t *buffer, const struct Fx25Mode *mode)
{
#ifdef FX25_PREGENERATE_POLYS
	struct LwFecRS *rs = NULL;
	switch(mode->T)
	{
		case 16:
			rs = &rs16;
			break;
		case 32:
			rs = &rs32;
			break;
		case 64:
			rs = &rs64;
			break;
		default:
			rs = &rs16;
			break;
	}
	RsEncode(rs, buffer, mode->K);
#else
	RsInit(&rs, mode->T, FX25_RS_FCR);
	RsEncode(&rs, buffer, mode->K);
#endif

}

bool Fx25Decode(uint8_t *buffer, const struct Fx25Mode *mode, uint8_t *fixed)
{
#ifdef FX25_PREGENERATE_POLYS
	struct LwFecRS *rs = NULL;
	switch(mode->T)
	{
		case 16:
			rs = &rs16;
			break;
		case 32:
			rs = &rs32;
			break;
		case 64:
			rs = &rs64;
			break;
		default:
			rs = &rs16;
			break;
	}
	return RsDecode(rs, buffer, mode->K, fixed);
#else
	RsInit(&rs, mode->T, FX25_RS_FCR);
	return RsDecode(&rs, buffer, mode->K, fixed);
#endif

}

void Fx25Init(void)
{
#ifdef FX25_PREGENERATE_POLYS
	RsInit(&rs16, 16, FX25_RS_FCR);
	RsInit(&rs32, 32, FX25_RS_FCR);
	RsInit(&rs64, 64, FX25_RS_FCR);
#else
#endif
}

#endif