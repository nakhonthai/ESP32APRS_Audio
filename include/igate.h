#ifndef IGATE_H
#define IGATE_H

#include <LibAPRSesp.h>
#include <WiFiClient.h>
#include <AX25.h>
#include "main.h"

// Duplicate packet detection
#define DUP_PACKET_CACHE_SIZE 10
#define DUP_PACKET_TIMEOUT_MS 30000  // 30 seconds

struct DupPacketCache {
    char hash[16];
    unsigned long timestamp;
};

int igateProcess(AX25Msg &Packet);
bool isDuplicatePacket(AX25Msg &Packet);
void clearExpiredDuplicates(void);

#endif