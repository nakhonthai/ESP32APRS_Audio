#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "KISS.h"
#include "ax25.h"

size_t ctxbufflen;
size_t frame_len = 0;
uint8_t *ctxbuffer;

static uint8_t serialBuffer[AX25_MAX_FRAME_LEN]; // Buffer for holding incoming serial data
AX25Ctx testkiss;

bool IN_FRAME;
bool ESCAPE;

uint8_t command = CMD_UNKNOWN;

extern struct Ax25ProtoConfig Ax25Config;

uint8_t p = 63;

int kiss_wrapper(uint8_t *pkg, uint8_t *buf, size_t len)
{
    uint8_t *ptr = pkg;
    int size = 0;
    *ptr++ = FEND;
    *ptr++ = 0x00;
    for (unsigned i = 0; i < len; i++)
    {
        uint8_t b = buf[i];
        if (b == FEND)
        {
            *ptr++ = FESC;
            *ptr++ = TFEND;
        }
        else if (b == FESC)
        {
            *ptr++ = FESC;
            *ptr++ = TFESC;
        }
        else
        {
            *ptr++ = b;
        }
    }
    *ptr++ = FEND;
    size = ptr - pkg;
    return size;
}

void kiss_serial(uint8_t sbyte)
{

    if (IN_FRAME && sbyte == FEND && command == CMD_DATA)
    {
        IN_FRAME = false;
        Ax25WriteTxFrame(serialBuffer, frame_len);
    }
    else if (sbyte == FEND)
    {
        IN_FRAME = true;
        command = CMD_UNKNOWN;
        frame_len = 0;
    }
    else if (IN_FRAME && frame_len < AX25_MAX_FRAME_LEN)
    {
        // Have a look at the command byte first
        if (frame_len == 0 && command == CMD_UNKNOWN)
        {
            // MicroModem supports only one HDLC port, so we
            // strip off the port nibble of the command byte
            sbyte = sbyte & 0x0F;
            command = sbyte;
        }
        else if (command == CMD_DATA)
        {
            if (sbyte == FESC)
            {
                ESCAPE = true;
            }
            else
            {
                if (ESCAPE)
                {
                    if (sbyte == TFEND)
                        sbyte = FEND;
                    if (sbyte == TFESC)
                        sbyte = FESC;
                    ESCAPE = false;
                }
                serialBuffer[frame_len++] = sbyte;
            }
        }
        else if (command == CMD_TXDELAY)
        {
            Ax25Config.txDelayLength = sbyte * 10UL;
        }
        else if (command == CMD_TXTAIL)
        {
            Ax25Config.txTailLength = sbyte * 10;
        }
        else if (command == CMD_SLOTTIME)
        {
            Ax25Config.quietTime = sbyte * 10;
        }
        else if (command == CMD_P)
        {
            p = sbyte;
        }
    }
}

size_t kiss_parse(uint8_t *buf, uint8_t *raw, size_t len)
{
    uint8_t sbyte = 0;
    size_t frame_len = 0;
    IN_FRAME = false;
    for (int i = 0; i < len; i++)
    {
        sbyte = raw[i];
        if (IN_FRAME && sbyte == FEND && command == CMD_DATA)
        {
            IN_FRAME = false;
            return frame_len;
        }
        else if (sbyte == FEND)
        {
            IN_FRAME = true;
            command = CMD_UNKNOWN;
            frame_len = 0;
        }
        else if (IN_FRAME && frame_len < AX25_MAX_FRAME_LEN)
        {
            // Have a look at the command byte first
            if (frame_len == 0 && command == CMD_UNKNOWN)
            {
                // MicroModem supports only one HDLC port, so we
                // strip off the port nibble of the command byte
                sbyte = sbyte & 0x0F;
                command = sbyte;
            }
            else if (command == CMD_DATA)
            {
                if (sbyte == FESC)
                {
                    ESCAPE = true;
                }
                else
                {
                    if (ESCAPE)
                    {
                        if (sbyte == TFEND)
                            sbyte = FEND;
                        if (sbyte == TFESC)
                            sbyte = FESC;
                        ESCAPE = false;
                    }
                    serialBuffer[frame_len++] = sbyte;
                }
            }
            else if (command == CMD_TXDELAY)
            {
                Ax25Config.txDelayLength = sbyte * 10UL;
            }
            else if (command == CMD_TXTAIL)
            {
                Ax25Config.txTailLength = sbyte * 10;
            }
            else if (command == CMD_SLOTTIME)
            {
                Ax25Config.quietTime = sbyte * 10;
            }
            else if (command == CMD_P)
            {
                p = sbyte;
            }
        }
    }
    return frame_len;
}