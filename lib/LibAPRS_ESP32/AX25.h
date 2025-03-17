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

#ifndef AX25_H_
#define AX25_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#define AX25_NOT_FX25 255

//for AX.25 329 bytes is the theoretical max size assuming 2-byte Control, 1-byte PID, 256-byte info field and 8 digi address fields
#define AX25_FRAME_MAX_SIZE (329) //single frame max length

enum Ax25RxStage
{
	RX_STAGE_IDLE = 0,
	RX_STAGE_FLAG,
	RX_STAGE_FRAME,
#ifdef ENABLE_FX25
	RX_STAGE_FX25_FRAME,
#endif
};

struct Ax25ProtoConfig
{
	uint16_t txDelayLength; //TXDelay length in ms
	uint16_t txTailLength; //TXTail length in ms
	uint16_t quietTime; //Quiet time in ms
	uint8_t allowNonAprs : 1; //allow non-APRS packets
	bool fx25 : 1; //enable FX.25 (AX.25 + FEC)
	bool fx25Tx : 1; //enable TX in FX.25
};

#define AX25_CTRL_UI      0x03
#define AX25_PID_NOLAYER3 0xF0

struct AX25Ctx;     // Forward declarations
struct AX25Msg;

extern bool ax25_stateTx;
extern int transmissionState;

typedef void (*ax25_callback_t)(struct AX25Msg *msg);
typedef struct Hdlc
{
    uint8_t demodulatedBits;
    uint8_t bitIndex;
    uint8_t currentByte;
    bool receiving;
} Hdlc;

typedef struct AX25Ctx {
    uint8_t buf[AX25_FRAME_MAX_SIZE];
    //FILE *ch;
    size_t frame_len;
    uint16_t crc_in;
    uint16_t crc_out;
    ax25_callback_t hook;
    bool sync;
    bool escape;
} AX25Ctx;
typedef struct ax25header_struct{   
    char addr[7];
    char ssid;   
}ax25header;

typedef struct ax25frame_struct{   
    ax25header  header[10];
    char data[AX25_FRAME_MAX_SIZE];   
}ax25frame;

#define AX25_CALL(str, id) {.call = (str), .ssid = (id) }
#define AX25_MAX_RPT 8
#define AX25_REPEATED(msg, n) ((msg)->rpt_flags & BV(n))

#define CALL_OVERSPACE 1

typedef struct AX25Call {
    char call[6+CALL_OVERSPACE];
    //char STRING_TERMINATION = 0;
    uint8_t ssid;
} AX25Call;

typedef struct AX25Msg {
    AX25Call src;
    AX25Call dst;
    AX25Call rpt_list[AX25_MAX_RPT];
    uint8_t  rpt_count;
    uint8_t  rpt_flags;
    uint16_t ctrl;
    uint8_t  pid;
    uint8_t info[AX25_FRAME_MAX_SIZE];
    size_t len;
    uint16_t mVrms;
} AX25Msg;

extern struct Ax25ProtoConfig Ax25Config;


/**
 * @brief Write frame to transmit buffer
 * @param *data Data to transmit
 * @param size Data size
 * @return Pointer to internal frame handle or NULL on failure
 * @attention This function will block if transmission is already in progress
 */
void *Ax25WriteTxFrame(uint8_t *data, uint16_t size);

/**
 * @brief Get bitmap of "frame received" flags for each decoder. A non-zero value means that a frame was received
 * @return Bitmap of decoder that received the frame
 */
uint8_t Ax25GetReceivedFrameBitmap(void);

/**
 * @brief Clear bitmap of "frame received" flags
 */
void Ax25ClearReceivedFrameBitmap(void);

/**
 * @brief Get next received frame (if available)
 * @param **dst Pointer to internal buffer
 * @param *size Actual frame size
<<<<<<< HEAD
 * @param *peak Signak positive peak value in %
 * @param *valley Signal negative peak value in %
 * @param *level Signal level in %
 * @param *corrected Number of bytes corrected in FX.25 mode. 255 is returned if not a FX.25 packet.
 * @return True if frame was read, false if no more frames to read
 */
bool Ax25ReadNextRxFrame(uint8_t **dst, uint16_t *size, int8_t *peak, int8_t *valley, uint8_t *level, uint8_t *corrected, uint16_t *mV);

/**
 * @brief Get current RX stage
 * @param[in] modemNo Modem/decoder number (0 or 1)
 * @return RX_STATE_IDLE, RX_STATE_FLAG or RX_STATE_FRAME
 * @warning Only for internal use
 */
enum Ax25RxStage Ax25GetRxStage(uint8_t modemNo);

/**
 * @brief Parse incoming bit (not symbol!)
 * @details Handles bit-stuffing, header and CRC checking, stores received frame and sets "frame received flag", multiplexes both decoders
 * @param[in] bit Incoming bit
 * @param[in] *dem Modem state pointer
 * @warning Only for internal use
 */
void Ax25BitParse(uint8_t bit, uint8_t modem,uint16_t mV);

/**
 * @brief Get next bit to be transmitted
 * @return Bit to be transmitted
 * @warning Only for internal use
 */
uint8_t Ax25GetTxBit(void);

/**
 * @brief Initialize transmission and start when possible
 */
void Ax25TransmitBuffer(void);

/**
 * @brief Start transmitting when possible
 * @attention Must be continuously polled in main loop
 */
void Ax25TransmitCheck(void);

/**
 * @brief Initialize AX25 module
 */
void Ax25Init(uint8_t fx25Mode);

void ax25_decode(uint8_t *buf,size_t len,uint16_t mVrms, AX25Msg *msg);

char ax25_encode(ax25frame &frame, char *txt,int size);
//void ax25sendFrame(AX25Ctx *ctx,ax25frame *pkg);
int hdlcFrame(uint8_t *outbuf, size_t outbuf_len, AX25Ctx *ctx, ax25frame *pkg);
void Ax25TxDelay(uint16_t delay_ms);
void Ax25TimeSlot(uint16_t ts);
bool Ax25NewRxFrames(void);

#endif /* AX25_H_ */