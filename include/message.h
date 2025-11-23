#include "main.h"
#ifndef MESSAGE_H
#define MESSAGE_H

#include <Arduino.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>
#include <esp_system.h>
#include "config.h"
#include "message.h"

typedef struct
{
	time_t time;
	int8_t ack;
    bool rxtx; // true=rx,false=tx
	uint8_t type;
	uint16_t msgID;
	char callsign[10];
	size_t length;
	char *text;	
} msgType;

int pkgMsg_Find(const char *call, uint16_t msgID, bool rxtx);
int pkgMsg_Find(uint16_t msg_id);
msgType getMsgList(int idx);
int pkgMsgUpdate(const char *call,const char *raw, uint16_t msg_id, int8_t ack, bool rxtx=false);
String aesEncryptBase64WithIV(const String &plain, const uint8_t key[16], uint16_t msgID);
String aesDecryptBase64WithIV(const String &b64, const uint8_t key[16], const char* callsign, uint16_t msgID);
void sendAPRSMessage(const String &toCall, const String &message, bool encrypt);
void handleIncomingAPRS(const String& line);
void sendAPRSMessageRetry();
void pkgMsgSort(msgType a[]);
// void processMessage(const char* message);
// void sendMessage(const char* message);

#endif // MESSAGE_H