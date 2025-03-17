#ifndef _PROTOCOL_KISS
#define _PROTOCOL_KISS 0x02

#include "AX25.h"
#include "AFSK.h"

#define FEND 0xC0
#define FESC 0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#define CMD_UNKNOWN 0xFE
#define CMD_DATA 0x00
#define CMD_TXDELAY 0x01
#define CMD_P 0x02
#define CMD_SLOTTIME 0x03
#define CMD_TXTAIL 0x04
#define CMD_FULLDUPLEX 0x05
#define CMD_SETHARDWARE 0x06
#define CMD_RETURN 0xFF

#define AX25_MAX_FRAME_LEN 329

//void kiss_csma(AX25Ctx *ctx, uint8_t *buf, size_t len);
int kiss_wrapper(uint8_t *pkg,uint8_t *buf,size_t len);
void kiss_serial(uint8_t sbyte);
size_t kiss_parse(uint8_t *buf,uint8_t *raw,size_t len);

#endif