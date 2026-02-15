// CRC-CCIT Implementation based on work by Francesco Sacchi

#ifndef CRC_CCIT_H
#define CRC_CCIT_H

#include <stdint.h>
#include <pgmspace.h>

#define CRC_CCIT_INIT_VAL ((uint16_t)0xFFFF)

extern const uint16_t crc_ccit_table[256];

inline uint16_t update_crc_ccit(uint8_t c, uint16_t prev_crc) {
    //return (prev_crc >> 8) ^ pgm_read_word(&crc_ccit_table[(prev_crc ^ c) & 0xff]);
    return (prev_crc >> 8) ^ crc_ccit_table[(prev_crc ^ c) & 0xff];
}

/*
 * Use this for an AX.25 frame.
 */

inline uint16_t fcs_calc(unsigned char *data, int len)
{
        uint16_t crc = 0xffff;
        int j;

        for (j=0; j<len; j++) {

          crc = ((crc) >> 8) ^ crc_ccit_table[((crc) ^ data[j]) & 0xff];
        }

        return ( crc ^ 0xffff );
}

inline unsigned short crc16 (unsigned char *data, int len, unsigned short seed)
{
	unsigned short crc = seed;
	int j;

	for (j=0; j<len; j++) {

  	  crc = ((crc) >> 8) ^ crc_ccit_table[((crc) ^ data[j]) & 0xff];
	}

	return ( crc ^ 0xffff );
}

#endif