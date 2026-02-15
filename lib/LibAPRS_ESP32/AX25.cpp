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

#include <Arduino.h>
#include "modem.h"
#include "AX25.h"
#include <stdlib.h>
#include "common.h"
#include <stdbool.h>
#include <string.h>

#include "AFSK.h"
#include "CRC-CCIT.h"

struct Ax25ProtoConfig Ax25Config;

#ifdef ENABLE_FX25
#include "fx25.h"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define FRAME_MAX_COUNT (5) //max count of frames in buffer
#else
#define FRAME_MAX_COUNT (3) //max count of frames in buffer
#endif
#define FRAME_BUFFER_SIZE (FRAME_MAX_COUNT * AX25_FRAME_MAX_SIZE) //circular frame buffer length

#define STATIC_HEADER_FLAG_COUNT 4 //number of flags sent before each frame
#define STATIC_FOOTER_FLAG_COUNT 1 //number of flags sent after each frame

#define MAX_TRANSMIT_RETRY_COUNT 8 //max number of retries if channel is busy

#define SYNC_BYTE 0x7E //preamble/postamble octet

struct FrameHandle
{
	uint16_t start;
	uint16_t size;
	int8_t peak;
	int8_t valley;
	uint8_t level;
	uint8_t corrected;
	uint16_t mVrms;
#ifdef ENABLE_FX25
	struct Fx25Mode *fx25Mode;
#endif
};

static uint8_t rxBuffer[FRAME_BUFFER_SIZE]; //circular buffer for received frames
static uint16_t rxBufferHead = 0; //circular RX buffer write index
static struct FrameHandle rxFrame[FRAME_MAX_COUNT];
static uint8_t rxFrameHead = 0;
static uint8_t rxFrameTail = 0;
static bool rxFrameBufferFull = false;

static uint8_t txBuffer[FRAME_BUFFER_SIZE];  //circular TX frame buffer
static uint16_t txBufferHead = 0; //circular TX buffer write index
static uint16_t txBufferTail = 0;
static struct FrameHandle txFrame[FRAME_MAX_COUNT];
static uint8_t txFrameHead = 0;
static uint8_t txFrameTail = 0;
static bool txFrameBufferFull = false;

#ifdef ENABLE_FX25
static uint8_t txFx25Buffer[FX25_MAX_BLOCK_SIZE];
static uint8_t txTagByteIdx = 0;
#endif

static uint8_t frameReceived; //a bitmap of receivers that received the frame

ax25_callback_t _hook;

enum TxStage
{
	TX_STAGE_IDLE = 0,
	TX_STAGE_PREAMBLE,
	TX_STAGE_HEADER_FLAGS,
	TX_STAGE_DATA,
	TX_STAGE_CRC,
	TX_STAGE_FOOTER_FLAGS,
	TX_STAGE_TAIL,

#ifdef ENABLE_FX25
	//stages used in FX.25 mode additionally
	TX_STAGE_CORRELATION_TAG,
#endif
};

enum TxInitStage
{
	TX_INIT_OFF,
	TX_INIT_WAITING,
	TX_INIT_TRANSMITTING
};

static uint8_t txByte = 0; //current TX byte
static uint16_t txByteIdx = 0; //current TX byte index
static int8_t txBitIdx = 0; //current bit index in txByte
static uint16_t txDelayElapsed = 0; //counter of TXDelay bytes already sent
static uint8_t txFlagsElapsed = 0; //counter of flag bytes already sent
static uint8_t txCrcByteIdx = 0; //currently transmitted byte of CRC
static uint8_t txBitstuff = 0; //bit-stuffing counter
static uint16_t txTailElapsed; //counter of TXTail bytes already sent
static uint16_t txCrc = 0xFFFF; //current CRC
static unsigned long txQuiet = 0; //quit time + current tick value
static uint8_t txRetries = 0; //number of TX retries
static enum TxInitStage txInitStage; //current TX initialization stage
static enum TxStage txStage; //current TX stage

struct RxState
{
	uint16_t crc; //current CRC
	uint8_t frame[AX25_FRAME_MAX_SIZE]; //raw frame buffer
	uint16_t frameIdx; //index for raw frame buffer
	uint8_t receivedByte; //byte being currently received
	uint8_t receivedBitIdx; //bit index for recByte
	uint8_t rawData; //raw data being currently received
	enum Ax25RxStage rx; //current RX stage
	uint8_t frameReceived; //frame received flag
#ifdef ENABLE_FX25
	struct Fx25Mode *fx25Mode;
	uint64_t tag; //received correlation tag
#endif
};

static struct RxState rxState[MODEM_MAX_DEMODULATOR_COUNT];

static uint16_t lastCrc = 0; //CRC of the last received frame. If not 0, a frame was successfully received
static uint16_t rxMultiplexDelay = 0; //simple delay for decoder multiplexer to avoid receiving the same frame twice

static uint16_t txDelay; //number of TXDelay bytes to send
static uint16_t txTail; //number of TXTail bytes to send

static uint8_t outputFrameBuffer[AX25_FRAME_MAX_SIZE];

#define GET_FREE_SIZE(max, head, tail) (((head) < (tail)) ? ((tail) - (head)) : ((max) - (head) + (tail)))
#define GET_USED_SIZE(max, head, tail) (max - GET_FREE_SIZE(max, head, tail))

/**
 * @brief Recalculate CRC for one bit
 * @param bit Input bit
 * @param *crc CRC pointer
 */
static void calculateCRC(uint8_t bit, uint16_t *crc)
{
    uint16_t xor_result;
    xor_result = *crc ^ bit;
    *crc >>= 1;
    if (xor_result & 0x0001)
    {
    	*crc ^= 0x8408;
    }
}

uint8_t Ax25GetReceivedFrameBitmap(void)
{
	return frameReceived;
}

void Ax25ClearReceivedFrameBitmap(void)
{
	frameReceived = 0;
}

#define countof(a) sizeof(a) / sizeof(a[0])
#define MIN(a, b) (                          \
    {                                        \
        typeof(a) _a = (a);                  \
        typeof(b) _b = (b);                  \
        ((typeof(_a))((_a < _b) ? _a : _b)); \
    })
#define DECODE_CALL(buf, addr)                                     \
    for (unsigned i = 0; i < sizeof((addr)) - CALL_OVERSPACE; i++) \
    {                                                              \
        char c = (*(buf)++ >> 1);                                  \
        (addr)[i] = (c == ' ') ? '\x0' : c;                        \
    }
#define AX25_SET_REPEATED(msg, idx, val)   \
    do                                     \
    {                                      \
        if (val)                           \
        {                                  \
            (msg)->rpt_flags |= _BV(idx);  \
        }                                  \
        else                               \
        {                                  \
            (msg)->rpt_flags &= ~_BV(idx); \
        }                                  \
    } while (0)

void ax25_decode(uint8_t *buf,size_t len,uint16_t mVrms)
{
    static AX25Msg msg;
    uint8_t *buf_start=buf;

    DECODE_CALL(buf, msg.dst.call);
    msg.dst.ssid = (*buf++ >> 1) & 0x0F;
    msg.dst.call[6] = 0;

    DECODE_CALL(buf, msg.src.call);
    msg.src.ssid = (*buf >> 1) & 0x0F;
    msg.src.call[6] = 0;

    for (msg.rpt_count = 0; !(*buf++ & 0x01) && (msg.rpt_count < countof(msg.rpt_list)); msg.rpt_count++)
    {
        DECODE_CALL(buf, msg.rpt_list[msg.rpt_count].call);
        msg.rpt_list[msg.rpt_count].ssid = (*buf >> 1) & 0x0F;
        AX25_SET_REPEATED(&msg, msg.rpt_count, (*buf & 0x80));
        msg.rpt_list[msg.rpt_count].call[6] = 0;
    }

    msg.ctrl = *buf++;
    if (msg.ctrl != AX25_CTRL_UI)
    {
        return;
    }

    msg.pid = *buf++;
    if (msg.pid != AX25_PID_NOLAYER3)
    {
        return;
    }

    memset(msg.info, 0, sizeof(msg.info));
    int rest = (buf_start+len)-buf;
    if(rest>0){
        msg.len = rest;
        strncpy((char *)msg.info, (const char*)buf, rest);
    }else{
        msg.len=0;
    }
    msg.mVrms=mVrms;

    if (_hook)
    {
        _hook(&msg);
    }
}

void ax25_decode(uint8_t *buf,size_t len,uint16_t mVrms, AX25Msg *msg)
{
    uint8_t *buf_start=buf;

    DECODE_CALL(buf, msg->dst.call);
    msg->dst.ssid = (*buf++ >> 1) & 0x0F;
    msg->dst.call[6] = 0;

    DECODE_CALL(buf, msg->src.call);
    msg->src.ssid = (*buf >> 1) & 0x0F;
    msg->src.call[6] = 0;

    for (msg->rpt_count = 0; !(*buf++ & 0x01) && (msg->rpt_count < countof(msg->rpt_list)); msg->rpt_count++)
    {
        DECODE_CALL(buf, msg->rpt_list[msg->rpt_count].call);
        msg->rpt_list[msg->rpt_count].ssid = (*buf >> 1) & 0x0F;
        AX25_SET_REPEATED(msg, msg->rpt_count, (*buf & 0x80));
        msg->rpt_list[msg->rpt_count].call[6] = 0;
    }

    msg->ctrl = *buf++;
    if (msg->ctrl != AX25_CTRL_UI)
    {
        return;
    }

    msg->pid = *buf++;
    if (msg->pid != AX25_PID_NOLAYER3)
    {
        return;
    }

    memset(msg->info, 0, sizeof(msg->info));
    int rest = (buf_start+len)-buf;
    if(rest>0){
        msg->len = rest;
        strncpy((char *)msg->info, (const char*)buf, rest);
    }else{
        msg->len=0;
    }
    msg->mVrms=mVrms;
}

#ifdef ENABLE_FX25
static void removeLastFrameFromRxBuffer(void)
{
	rxBufferHead = rxFrame[rxFrameHead].start;
	if(rxFrameHead == 0)
		rxFrameHead = FRAME_MAX_COUNT - 1;
	else
		rxFrameHead--;
	rxFrameBufferFull = false;
}



static void *writeFx25Frame(uint8_t *data, uint16_t size)
{
	//first calculate how big the frame can be
	//this includes 2 flags, 2 CRC bytes and all bits added by bitstuffing
	//bitstuffing occurs after 5 consecutive ones, so in worst scenario
	//bits inserted by bitstuffing can occupy up to frame size / 5 additional bytes
	//also add 1 in case there is a remainder when dividing
	const struct Fx25Mode *fx25Mode = fx25Mode = Fx25GetModeForSize(size + 4 + (size / 5) + 1);
	uint16_t requiredSize = size;
	if(NULL != fx25Mode)
		requiredSize = fx25Mode->K + fx25Mode->T;
	else
		return NULL; //frame will not fit in FX.25

	uint16_t freeSize = GET_FREE_SIZE(FRAME_BUFFER_SIZE, txBufferHead, txBufferTail);
	if(freeSize < requiredSize) //check if there is enough size to store full FX.25 (or AX.25) frame
	{
		return NULL; //if not, it may fit in standard AX.25
	}

	txFrame[txFrameHead].size = requiredSize;
	txFrame[txFrameHead].start = txBufferHead;
	txFrame[txFrameHead].fx25Mode = (struct Fx25Mode*)fx25Mode;

	memset(txFx25Buffer, 0, sizeof(txFx25Buffer));

	uint16_t index = 0;
	//header flag
	txFx25Buffer[index++] = 0x7E;

	uint16_t crc = 0xFFFF;

	uint8_t bits = 0; //bit counter within a byte
	uint8_t bitstuff = 0;
	for(uint16_t i = 0; i < size + 2; i++)
	{
		for(uint8_t k = 0; k < 8; k++)
		{
			txFx25Buffer[index] >>= 1;
			bits++;
			if(i < size) //frame data
			{
				if((data[i] >> k) & 1)
				{
					calculateCRC(1, &crc);
					bitstuff++;
					txFx25Buffer[index] |= 0x80;
				}
				else
				{
					calculateCRC(0, &crc);
					bitstuff = 0;
				}
			}
			else //crc
			{
				uint8_t c = 0;
				if(i == size)
					c = (crc & 0xFF) ^ 0xFF;
				else
					c = (crc >> 8) ^ 0xFF;

				if((c >> k) & 1)
				{
					bitstuff++;
					txFx25Buffer[index] |= 0x80;
				}
				else
				{
					bitstuff = 0;
				}
			}

			if(bits == 8)
			{
				bits = 0;
				index++;
			}
			if(bitstuff == 5)
			{
				bits++;
				bitstuff = 0;
				txFx25Buffer[index] >>= 1;
				if(bits == 8)
				{
					bits = 0;
					index++;
				}
			}
		}
	}

	//pad with flags
	while(index < fx25Mode->K)
	{
		for(uint8_t k = 0; k < 8; k++)
		{
			txFx25Buffer[index] >>= 1;
			bits++;

			if((0x7E >> k) & 1)
			{
				txFx25Buffer[index] |= 0x80;
			}

			if(bits == 8)
			{
				bits = 0;
				index++;
			}
		}
	}

	Fx25Encode(txFx25Buffer, fx25Mode);

	for(uint16_t i = 0; i < (fx25Mode->K + fx25Mode->T); i++)
	{
		txBuffer[txBufferHead++] = txFx25Buffer[i];
		txBufferHead %= FRAME_BUFFER_SIZE;
	}

	void *ret = &txFrame[txFrameHead];
	txFrameHead++;
	txFrameHead %= FRAME_MAX_COUNT;
	if(txFrameHead == txFrameTail)
		txFrameBufferFull = true;
	return ret;
}

static struct FrameHandle* parseFx25Frame(uint8_t *frame, uint16_t size, uint16_t *crc)
{
	struct FrameHandle *h = &rxFrame[rxFrameHead];
	uint16_t initialRxBufferHead = rxBufferHead;
	if(!rxFrameBufferFull)
	{
		rxFrame[rxFrameHead++].start = rxBufferHead;
		rxFrameHead %= FRAME_MAX_COUNT;
		if(rxFrameHead == txFrameHead)
			rxFrameBufferFull = true;
	}
	else
		return NULL;

	uint16_t i = 0; //input data index
	uint16_t k = 0; //output data size
	while(frame[i] == 0x7E)
		i++;

	uint8_t bitstuff = 0;
	uint8_t outBit = 0;
	for(; i < size; i++)
	{
		for(uint8_t b = 0; b < 8; b++)
		{
			if(frame[i] & (1 << b))
			{
				rxBuffer[rxBufferHead] >>= 1;
				rxBuffer[rxBufferHead] |= 0x80;
				bitstuff++;
			}
			else
			{
				if(bitstuff == 5) //zero after 5 ones, normal bitstuffing
				{
					bitstuff = 0;
					continue;
				}
				else if(bitstuff == 6) //zero after 6 ones, this is a flag
				{
					goto endParseFx25Frame;
				}
				else if(bitstuff >= 7) //zero after 7 ones, illegal byte
				{
					removeLastFrameFromRxBuffer();
					return NULL;
				}
				bitstuff = 0;
				rxBuffer[rxBufferHead] >>= 1;
			}
			outBit++;
			if(outBit == 8)
			{
				k++;
				rxBufferHead++;
				rxBufferHead %= FRAME_BUFFER_SIZE;
				outBit = 0;
			}
		}
	}

endParseFx25Frame:
	*crc = 0xFFFF;
	i = initialRxBufferHead;

	for(uint16_t j = 0; j < (k - 2); j++)
	{
		for(uint8_t b = 0; b < 8; b++)
			calculateCRC((rxBuffer[i] >> b) & 1, crc);

		i++;
		i %= FRAME_BUFFER_SIZE;
	}


	*crc ^= 0xFFFF;
	if((rxBuffer[i] == (*crc & 0xFF) )
		&& (rxBuffer[(i + 1) % FRAME_BUFFER_SIZE] == ((*crc >> 8) & 0xFF))) //check CRC
	{
		uint16_t pathEnd = initialRxBufferHead;
		for(uint16_t j = 0; j < (k - 2); j++)
		{
			if(rxBuffer[pathEnd] & 1)
				break;
			pathEnd++;
			pathEnd %= FRAME_BUFFER_SIZE;
		}

		if(Ax25Config.allowNonAprs || (((rxBuffer[(pathEnd + 1) % FRAME_BUFFER_SIZE] == 0x03) && (rxBuffer[(pathEnd + 2) % FRAME_BUFFER_SIZE] == 0xF0))))
		{
			h->size = k - 2;
			return h;
		}
	}

	removeLastFrameFromRxBuffer();
	return NULL;
}
#endif

void *Ax25WriteTxFrame(uint8_t *data, uint16_t size)
{
	if(txFrameBufferFull)
		return NULL;

#ifdef ENABLE_FX25
	if(Ax25Config.fx25 && Ax25Config.fx25Tx)
	{
		void *ret = writeFx25Frame(data, size);
		if(ret)
			return ret;
	}
#endif

	if(GET_FREE_SIZE(FRAME_BUFFER_SIZE, txBufferHead, txBufferTail) < size)
	{
		return NULL;
	}

	txFrame[txFrameHead].size = size;
	txFrame[txFrameHead].start = txBufferHead;

#ifdef ENABLE_FX25
	txFrame[txFrameHead].fx25Mode = NULL;
#endif


	for(uint16_t i = 0; i < size; i++)
	{
		txBuffer[txBufferHead++] = data[i];
		txBufferHead %= FRAME_BUFFER_SIZE;
	}


	void *ret = &txFrame[txFrameHead];
	//__disable_irq();
	txFrameHead++;
	txFrameHead %= FRAME_MAX_COUNT;
	if(txFrameHead == txFrameTail)
		txFrameBufferFull = true;
	//__enable_irq();
	return ret;
}


bool Ax25ReadNextRxFrame(uint8_t **dst, uint16_t *size, int8_t *peak, int8_t *valley, uint8_t *level, uint8_t *corrected, uint16_t *mV)
{
	if((rxFrameHead == rxFrameTail) && !rxFrameBufferFull)
		return false;

	*dst = outputFrameBuffer;

	for(uint16_t i = 0; i < rxFrame[rxFrameTail].size; i++)
	{
		(*dst)[i] = rxBuffer[(rxFrame[rxFrameTail].start + i) % FRAME_BUFFER_SIZE];
	}

	*peak = rxFrame[rxFrameTail].peak;
	*valley = rxFrame[rxFrameTail].valley;
	*level = rxFrame[rxFrameTail].level;
	*size = rxFrame[rxFrameTail].size;
	*corrected = rxFrame[rxFrameTail].corrected;
	*mV = rxFrame[rxFrameTail].mVrms;

	//__disable_irq();
	rxFrameBufferFull = false;
	rxFrameTail++;
	rxFrameTail %= FRAME_MAX_COUNT;
	//__enable_irq();
	return true;
}

enum Ax25RxStage Ax25GetRxStage(uint8_t modem)
{
	return rxState[modem].rx;
}

extern AX25Ctx AX25;
void Ax25BitParse(uint8_t bit, uint8_t modem,uint16_t mV)
{
	if(lastCrc != 0) //there was a frame received
	{
		rxMultiplexDelay++;
		if(rxMultiplexDelay > (4 * MODEM_MAX_DEMODULATOR_COUNT)) //hold it for a while and wait for other decoders to receive the frame
		{
			lastCrc = 0;
			rxMultiplexDelay = 0;
			for(uint8_t i = 0; i < MODEM_MAX_DEMODULATOR_COUNT; i++)
			{
				frameReceived |= ((rxState[i].frameReceived > 0) << i);
				rxState[i].frameReceived = 0;
			}
		}

	}

	struct RxState *rx = (struct RxState*)&(rxState[modem]);

	rx->rawData <<= 1; //store incoming bit
	rx->rawData |= (bit > 0);

#ifdef ENABLE_FX25
	rx->tag >>= 1;
	if(bit)
		rx->tag |= 0x8000000000000000;

	if(Ax25Config.fx25
			&& (rx->rx != RX_STAGE_FX25_FRAME)
			&& (NULL != (rx->fx25Mode = (struct Fx25Mode*)Fx25GetModeForTag(rx->tag))))
	{
		rx->rx = RX_STAGE_FX25_FRAME;
		rx->receivedByte = 0;
		rx->receivedBitIdx = 0;
		rx->frameIdx = 0;
		return;
	}

	if(rx->rx != RX_STAGE_FX25_FRAME)
	{
#endif

		if(rx->rawData == 0x7E) //HDLC flag received
		{
			if(rx->rx == RX_STAGE_FRAME) //if we are in frame, this is the end of the frame
			{
				if(rx->frameIdx >= 17) //correct frame must be at least 17 bytes long (source+destination+control+CRC)
				{
					rx->crc ^= 0xFFFF;
					if((rx->frame[rx->frameIdx - 2] == (rx->crc & 0xFF)) && (rx->frame[rx->frameIdx - 1] == ((rx->crc >> 8) & 0xFF))) //check CRC
					{
						uint16_t i = 13;
                        //log_d("[%i]%s",rx->frameIdx,rx->frame);
						//start from 13, which is the SSID of source
						for(; i < (rx->frameIdx - 2); i++) //look for path end bit
						{
							if(rx->frame[i] & 1)
								break;
						}

						//if non-APRS frames are not allowed, check if this frame has control=0x03 and PID=0xF0
						if(Ax25Config.allowNonAprs || (((rx->frame[i + 1] == 0x03) && (rx->frame[i + 2] == 0xF0))))
						{                                                       

							rx->frameReceived = 1;
							rx->frameIdx -= 2; //remove CRC
							if(rx->crc != lastCrc) //the other decoder has not received this frame yet, so store it in main frame buffer
							{
								lastCrc = rx->crc; //store CRC of this frame
                                //  ax25_decode(rx->frame,rx->frameIdx,mV);
                                //  int8_t peak,valley;
                                //  uint8_t level;
                                //  ModemGetSignalLevel(modem, &peak,&valley,&level);
                                //log_d("Pkt=%d SND: peak=%d valley=%d level=%d",rx->frameIdx,peak,valley,level);

								if(!rxFrameBufferFull) //if enough space, store the frame
								{
									rxFrame[rxFrameHead].start = rxBufferHead;
									rxFrame[rxFrameHead].mVrms=mV;
									ModemGetSignalLevel(modem, &rxFrame[rxFrameHead].peak, &rxFrame[rxFrameHead].valley, &rxFrame[rxFrameHead].level);
									log_d("Pkt=%d SND: peak=%d valley=%d level=%d",rx->frameIdx,rxFrame[rxFrameHead].peak,rxFrame[rxFrameHead].valley,rxFrame[rxFrameHead].level);
#ifdef ENABLE_FX25
									rxFrame[rxFrameHead].fx25Mode = NULL;
#endif
									rxFrame[rxFrameHead].corrected = AX25_NOT_FX25;
									//__disable_irq();
									rxFrame[rxFrameHead++].size = rx->frameIdx;
									rxFrameHead %= FRAME_MAX_COUNT;
									if(rxFrameHead == rxFrameTail)
										rxFrameBufferFull = true;
									//__enable_irq();

									for(uint16_t i = 0; i < rx->frameIdx; i++)
									{
										rxBuffer[rxBufferHead++] = rx->frame[i];
										rxBufferHead %= FRAME_BUFFER_SIZE;
									}
								}else{
									log_w("RX frame buffer full");
								}
							}
						}
					}
				}
			}
			rx->rx = RX_STAGE_FLAG;
			rx->receivedByte = 0;
			rx->receivedBitIdx = 0;
			rx->frameIdx = 0;
			rx->crc = 0xFFFF;
			return;
		}
		else
			rx->rx = RX_STAGE_FRAME;

#ifndef ENABLE_FX25
	{
		//this condition must not be checked when FX.25 is enabled
		//because FX.25 parity bytes and tags contain >= 7 consecutive ones
		if((rx->rawData & 0x7F) == 0x7F) //received 7 consecutive ones, this is an error
		{
			rx->rx = RX_STAGE_IDLE;
			rx->receivedByte = 0;
			rx->receivedBitIdx = 0;
			rx->frameIdx = 0;
			rx->crc = 0xFFFF;
			return;
		}
#endif
		if((rx->rawData & 0x3F) == 0x3E) //dismiss bit 0 added by bit stuffing
			return;
	}


	if(rx->rawData & 0x01) //received bit 1
		rx->receivedByte |= 0x80; //store it


	if(++rx->receivedBitIdx >= 8) //received full byte
	{
		if(rx->frameIdx >= 2)
		{
			for(uint8_t k = 0; k < 8; k++)
			{
				calculateCRC((rx->frame[rx->frameIdx - 2] >> k) & 1, &(rx->crc));
			}
		}

#ifdef ENABLE_FX25
		//end of FX.25 reception, that is received full block
		if((rx->fx25Mode != NULL) && (rx->frameIdx == (rx->fx25Mode->K + rx->fx25Mode->T)))
		{
			uint8_t fixed = 0;
			bool fecSuccess = Fx25Decode(rx->frame, rx->fx25Mode, &fixed);
			uint16_t crc;
			struct FrameHandle *h = parseFx25Frame(rx->frame, rx->frameIdx, &crc);
			if(h != NULL)
			{
				rx->frameReceived = 1;
				ModemGetSignalLevel(modem, &h->peak, &h->valley, &h->level);
				if(fecSuccess)
				{
					h->corrected = fixed;
					h->fx25Mode = rx->fx25Mode;
				}
				else
					h->corrected = AX25_NOT_FX25;
				lastCrc = crc;
			}
			rx->rx = RX_STAGE_FLAG;
			rx->receivedByte = 0;
			rx->receivedBitIdx = 0;
			rx->frameIdx = 0;
			return;
		}
#else
		rx->rx = RX_STAGE_FRAME;
#endif
		if(rx->frameIdx >= AX25_FRAME_MAX_SIZE) //frame is too long
		{
			rx->rx = RX_STAGE_IDLE;
			rx->receivedByte = 0;
			rx->receivedBitIdx = 0;
			rx->frameIdx = 0;
			rx->crc = 0xFFFF;
			return;
		}
		rx->frame[rx->frameIdx++] = rx->receivedByte; //store received byte
		rx->receivedByte = 0;
		rx->receivedBitIdx = 0;
	}
	else
		rx->receivedByte >>= 1;
}


uint8_t Ax25GetTxBit(void)
{
	if(txBitIdx == 8)
	{
		txBitIdx = 0;
		if(txStage == TX_STAGE_PREAMBLE) //transmitting preamble (TXDelay)
		{
			if(txDelayElapsed < txDelay)
			{
				txByte = SYNC_BYTE;
				txDelayElapsed++;
			}
			else
			{
				txDelayElapsed = 0;
#ifdef ENABLE_FX25
				if(NULL != txFrame[txFrameTail].fx25Mode)
				{
					txStage = TX_STAGE_CORRELATION_TAG;
					txTagByteIdx = 0;
				}
				else
#endif
					txStage = TX_STAGE_HEADER_FLAGS;
			}
		}
#ifdef ENABLE_FX25
transmitTag:
		if(txStage == TX_STAGE_CORRELATION_TAG) //FX.25 correlation tag
		{
			if(txTagByteIdx < 8)
				txByte = (txFrame[txFrameTail].fx25Mode->tag >> (8 * txTagByteIdx)) & 0xFF;
			else
				txStage = TX_STAGE_DATA;

			txTagByteIdx++;
		}
#endif
		if(txStage == TX_STAGE_HEADER_FLAGS) //transmitting initial flags
		{
			if(txFlagsElapsed < STATIC_HEADER_FLAG_COUNT)
			{
				txByte = 0x7E;
				txFlagsElapsed++;
			}
			else
			{
				txFlagsElapsed = 0;
				txStage = TX_STAGE_DATA; //transmit data
			}
		}
		if(txStage == TX_STAGE_DATA) //transmitting normal data
		{
transmitNormalData:
			//__disable_irq();
			if((txFrameHead != txFrameTail) || txFrameBufferFull)
			{
				//__enable_irq();
				if(txByteIdx < txFrame[txFrameTail].size) //send buffer
				{
					txByte = txBuffer[(txFrame[txFrameTail].start + txByteIdx) % FRAME_BUFFER_SIZE];
					txByteIdx++;
				}
#ifdef ENABLE_FX25
				else if(txFrame[txFrameTail].fx25Mode != NULL)
				{
					//__disable_irq();
					txFrameBufferFull = false;
					txFrameTail++;
					txFrameTail %= FRAME_MAX_COUNT;
					txByteIdx = 0;
					//__enable_irq();
					if((txFrameHead != txFrameTail) || txFrameBufferFull)
					{
						if(txFrame[txFrameTail].fx25Mode != NULL)
						{
							txStage = TX_STAGE_CORRELATION_TAG;
							txTagByteIdx = 0;
							goto transmitTag;
						}
						else
							goto transmitNormalData;
					}
					else
						goto transmitTail;
				}
#endif
				else
				{
					txStage = TX_STAGE_CRC; //transmit CRC
					txCrcByteIdx = 0;
				}
			}
			else //no more frames
			{
transmitTail:
				//__enable_irq();
				txByteIdx = 0;
				txBitIdx = 0;
				txStage = TX_STAGE_TAIL;
			}
		}
		if(txStage == TX_STAGE_CRC) //transmitting CRC
		{
			if(txCrcByteIdx <= 1)
			{
				txByte = (txCrc & 0xFF) ^ 0xFF;
				txCrc >>= 8;
				txCrcByteIdx++;
			}
			else
			{
				txCrc = 0xFFFF;
				txStage = TX_STAGE_FOOTER_FLAGS; //now transmit flags
				txFlagsElapsed = 0;
			}

		}
		if(txStage == TX_STAGE_FOOTER_FLAGS)
		{
			if(txFlagsElapsed < STATIC_FOOTER_FLAG_COUNT)
			{
				txByte = 0x7E;
				txFlagsElapsed++;
			}
			else
			{
				txFlagsElapsed = 0;
				//__disable_irq();
				txFrameBufferFull = false;
				txFrameTail++;
				txFrameTail %= FRAME_MAX_COUNT;
				txByteIdx = 0;
#ifdef ENABLE_FX25
				if(((txFrameHead != txFrameTail) || txFrameBufferFull) && (txFrame[txFrameTail].fx25Mode != NULL))
				{
					//__enable_irq();
					txStage = TX_STAGE_CORRELATION_TAG;
					txTagByteIdx = 0;
					goto transmitTag;
				}
#endif
				//__enable_irq();
				txStage = TX_STAGE_DATA; //return to normal data transmission stage. There might be a next frame to transmit
				goto transmitNormalData;
			}
		}
		if(txStage == TX_STAGE_TAIL) //transmitting tail
		{
			if(txTailElapsed < txTail)
			{
				txByte = SYNC_BYTE;
				txTailElapsed++;
			}
			else //tail transmitted, stop transmission
			{
				txTailElapsed = 0;
				txStage = TX_STAGE_IDLE;
				txCrc = 0xFFFF;
				txBitstuff = 0;
				txByte = 0;
				txInitStage = TX_INIT_OFF;
				txBufferTail = txBufferHead;
				ModemTransmitStop();
				return 0;
			}
		}

	}

	uint8_t txBit = 0;
	//transmitting normal data or CRC in AX.25 mode
	if(
#ifdef ENABLE_FX25
		(NULL == txFrame[txFrameTail].fx25Mode) &&
#endif
		((txStage == TX_STAGE_DATA) || (txStage == TX_STAGE_CRC)))
	{
		if(txBitstuff == 5) //5 consecutive ones transmitted
		{
			txBit = 0; //transmit bit-stuffed 0
			txBitstuff = 0;
		}
		else
		{
			if(txByte & 1) //1 being transmitted
			{
				txBitstuff++; //increment bit stuffing counter
				txBit = 1;
			}
			else
			{
				txBit = 0;
				txBitstuff = 0; //0 being transmitted, reset bit stuffing counter
			}
			if(txStage == TX_STAGE_DATA) //calculate CRC only for normal data
				calculateCRC(txByte & 1, &txCrc);

			txByte >>= 1;
			txBitIdx++;
		}
	}
	//transmitting in FX.25 mode or in AX.25 mode, but these are preamble or flags, don't calculate CRC, don't use bit stuffing
	else
	{
		txBit = txByte & 1;
		txByte >>= 1;
		txBitIdx++;
	}
	return txBit;
}

/**
 * @brief Initialize transmission and start when possible
 */
void Ax25TransmitBuffer()
{	
	 if(txInitStage == TX_INIT_WAITING)
	 	return;
	 if(txInitStage == TX_INIT_TRANSMITTING)
	 	return;

	 if((txFrameHead != txFrameTail) || txFrameBufferFull)
	 {
	 	txQuiet = (millis() + (Ax25Config.quietTime)); //calculate required delay
	 	txInitStage = TX_INIT_WAITING;
	 }
}



/**
 * @brief Start transmission immediately
 * @warning Transmission should be initialized using Ax25_transmitBuffer
 */
static void transmitStart(void)
{
	txCrc = 0xFFFF; //initial CRC value
	txStage = TX_STAGE_PREAMBLE;
	txByte = 0;
	txBitIdx = 0;
	txFlagsElapsed = 0;
	txDelayElapsed = 0;
	ModemTransmitStart();
	log_d("transmitStart");
}


/**
 * @brief Start transmitting when possible
 * @attention Must be continuously polled in main loop
 */
void Ax25TransmitCheck(void)
{
	 if(txInitStage == TX_INIT_OFF) //TX not initialized at all, nothing to transmit
	 	return;
	 if(txInitStage == TX_INIT_TRANSMITTING) //already transmitting
	 	return;

	 //if(ModemIsTxTestOngoing()) //TX test is enabled, wait for now
	 //	return;

	 if(txQuiet < millis()) //quit time has elapsed
	 {
	 	if(!ModemDcdState()) //channel is free
	 	{
	 		txInitStage = TX_INIT_TRANSMITTING; //transmit right now
	 		txRetries = 0;
	 		transmitStart();
	 	}
	 	else //channel is busy
	 	{
	 		if(txRetries >= MAX_TRANSMIT_RETRY_COUNT) //timeout
	 		{
	 			txInitStage = TX_INIT_TRANSMITTING; //transmit right now
	 			txRetries = 0;
	 			transmitStart();
	 		}
	 		else //still trying
	 		{
	 			txQuiet = millis() + random(100, 1000); //try again after some random time
	 			txRetries++;
	 		}
	 	}
	 }
}

void Ax25Init(uint8_t fx25Mode)
{
	txCrc = 0xFFFF;
    memset(&Ax25Config, 0, sizeof(Ax25Config));
    Ax25Config.quietTime = 2000;
	Ax25Config.txDelayLength = 300;
	Ax25Config.txTailLength = 1;
	if(fx25Mode==0){
		Ax25Config.fx25 = 0;
		Ax25Config.fx25Tx = 0;
	}else if(fx25Mode==1){
		Ax25Config.fx25 = 1;
		Ax25Config.fx25Tx = 0;
	}else{
		Ax25Config.fx25 = 1;
		Ax25Config.fx25Tx = 1;
	}

	memset((void*)rxState, 0, sizeof(rxState));
	for(uint8_t i = 0; i < (sizeof(rxState) / sizeof(rxState[0])); i++)
		rxState[i].crc = 0xFFFF;

	txDelay = ((float)Ax25Config.txDelayLength / (8.f * 1000.f / ModemGetBaudrate())); //change milliseconds to byte count
	txTail = ((float)Ax25Config.txTailLength / (8.f * 1000.f / ModemGetBaudrate()));
	txInitStage == TX_INIT_OFF;
	txQuiet = (millis() + (Ax25Config.quietTime) + random(10, 200)); //calculate required delay
}

void Ax25TxDelay(uint16_t delay_ms)
{
	Ax25Config.txDelayLength = delay_ms;
	txDelay = ((float)Ax25Config.txDelayLength / (8.f * 1000.f / ModemGetBaudrate())); //change milliseconds to byte count
}

void Ax25TimeSlot(uint16_t ts)
{
	if(ts>0){
		Ax25Config.quietTime = ts;	
		txQuiet = (millis() + (Ax25Config.quietTime) + random(100, 1000)); //calculate required delay
	}else{
		txQuiet = 0; //no delay
	}
}

//////////////////////////////////
unsigned int strpos(char *txt, char chk)
{
    char *pch;
    unsigned int idx = 0;
    pch = strchr(txt, chk);
    idx = pch - txt;
    return idx;
}

void convPath(ax25header *hdr, char *txt, unsigned int size)
{
    unsigned int i, p, j;
    char num[5];
    hdr->ssid = 0;
    memset(hdr->addr, 0, 7);
    memset(&num[0], 0, sizeof(num));

    p = strpos(txt, '-');
    if (p > 0 && p < size)
    {
        for (i = 0; i < p; i++)
        { // Get CallSign/Path
            hdr->addr[i] = txt[i];
        }
        j = 0;
        for (i = p + 1; i < size; i++)
        { // get SSID
            //            if(txt[i]=='*') break;
            //            if(txt[i]==',') break;
            //            if(txt[i]==':') break;
            if (txt[i] < 0x30)
                break;
            if (txt[i] > 0x39)
                break;
            num[j++] = txt[i];
        }
        if (j > 0)
        {
            hdr->ssid = atoi(num);
        }
        hdr->ssid <<= 1;
    }
    else
    {
        for (i = 0; i < size; i++)
        { // Get CallSign/Path
            if (txt[i] == '*')
                break;
            if (txt[i] == ',')
                break;
            if (txt[i] == ':')
                break;
            hdr->addr[i] = txt[i];
        }

        hdr->ssid = 0;
    }
    p = strpos(txt, '*');
    if (p > 0 && p < size)
        hdr->ssid |= 0x80;
    hdr->ssid |= 0x60;
}

char ax25_encode(ax25frame &frame, char *txt, int size)
{
    char *token, *ptr;
    int i;
    unsigned int p, p2, p3;
    char j;
    ptr = (char *)&frame;
    memset(ptr, 0, sizeof(ax25frame)); // Clear frame
    p = strpos(txt, ':');
    if (p > 0 && p < size)
    {
        // printf("p{:}=%d\r\n",p);
        // Get String APRS
        memset(&frame.data, 0, sizeof(frame.data));
        for (i = 0; i < (size - p); i++)
            frame.data[i] = txt[p + i + 1];
        p2 = strpos(txt, '>');
        if (p2 > 0 && p2 < size)
        {
            // printf("p2{>}=%d\r\n",p2);
            convPath(&frame.header[1], &txt[0], p2); // Get callsign src
            j = strpos(txt, ',');
            if ((j < 1) || (j > p))
                j = p;
            convPath(&frame.header[0], &txt[p2 + 1], j - p2 - 1); // Get callsign dest

            p3 = 0;
			if ((j > p2) && (j < p)) { //Check PATH
				for (i = j; i < size; i++)
				{ // copy path to origin
					if (txt[i] == ':')
					{
						for (; i < size; i++)
							txt[p3++] = 0x00;
						break;
					}
					txt[p3++] = txt[i];
				}
				// printf("Path:%s\r\n",txt);
				token = strtok(txt, ",");
				j = 0;
				while (token != NULL)
				{
					ptr = token;
					convPath(&frame.header[j + 2], ptr, strlen(ptr));
					token = strtok(NULL, ",");
					j++;
					if (j > 7)
						break;
				}
			}

            for (i = 0; i < 10; i++)
                frame.header[i].ssid &= 0xFE; // Clear All END Path
            // Fix END path
            for (i = 2; i < 10; i++)
            {
                if (frame.header[i].addr[0] == 0x00)
                {
                    frame.header[i - 1].ssid |= 0x01;
                    break;
                }
            }
            // }
            return 1;
        }
    }
    return 0;
}

#define CRC_CCIT_INIT_VAL ((uint16_t)0xFFFF)
#define HDLC_FLAG  0x7E
#define HDLC_RESET 0x7F
#define AX25_ESC   0x1B

uint8_t *ax25_putRaw(uint8_t *raw, AX25Ctx *ctx, uint8_t c)
{
    if (c == HDLC_FLAG || c == HDLC_RESET || c == AX25_ESC)
    {
        *raw++ = AX25_ESC;
    }
    ctx->crc_out = update_crc_ccit(c, ctx->crc_out);
    *raw++ = c;
    return raw;
}

int hdlcFrame(uint8_t *outbuf, size_t outbuf_len, AX25Ctx *ctx, ax25frame *pkg)
{
    int i, j, c = 0;
    int idx = 0;
    uint8_t data = 0;
    ctx->crc_out = CRC_CCIT_INIT_VAL;
    int raw_count = 0;
    uint8_t info[300];
    //info[idx++] = HDLC_FLAG;

    for (i = 0; i < 10; i++)
        pkg->header[i].ssid &= 0xFE; // Clear All END Path
    // Fix END path
    for (i = 1; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0x00)
        {
            pkg->header[i - 1].ssid |= 0x01;
            break;
        }
    }

    for (i = 0; i < 10; i++)
    {
        if (pkg->header[i].addr[0] == 0)
            break;
        for (j = 0; j < 6; j++)
        {
            data = (uint8_t)pkg->header[i].addr[j];
            if (data == 0)
                data = 0x20;
            // putchar(data);
            data <<= 1;
            ax25_putRaw(&info[idx++], ctx, data);
            c++;
        }
        ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->header[i].ssid);
        if (pkg->header[i].ssid & 0x01)
            break;
    }

    ax25_putRaw(&info[idx++], ctx, AX25_CTRL_UI);      // Control field - 0x03 is APRS UI-frame
    ax25_putRaw(&info[idx++], ctx, AX25_PID_NOLAYER3); // Protocol ID - 0xF0 is no layer 3

    for (i = 0; i < strlen(pkg->data); i++)
    {
        ax25_putRaw(&info[idx++], ctx, (uint8_t)pkg->data[i]);
    }

    //uint8_t crcl = (ctx->crc_out & 0xff) ^ 0xff;
    //uint8_t crch = (ctx->crc_out >> 8) ^ 0xff;
    //ax25_putRaw(&info[idx++], ctx, crcl);
    //ax25_putRaw(&info[idx++], ctx, crch);
    //memcpy(ctx->buf, &info[1], idx - 1);
    //ctx->frame_len = idx - 1;

    //info[idx++] = HDLC_FLAG;
    //int len = bit_stuffing(outbuf, outbuf_len, &info[0], idx);
	memcpy(outbuf, &info[0], idx);
    return idx;
}

bool Ax25NewRxFrames(void)
{
	return (rxFrameHead != rxFrameTail) || rxFrameBufferFull;
}