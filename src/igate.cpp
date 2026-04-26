/*
 Name:		ESP32APRS
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#include "igate.h"

extern WiFiClient aprsClient;
extern Configuration config;
extern statusType status;

// Duplicate packet cache
static struct DupPacketCache dupCache[DUP_PACKET_CACHE_SIZE];
static uint8_t dupCacheIndex = 0;

// Simple hash function for packet deduplication
static void packetHash(AX25Msg &Packet, char *hash)
{
    sprintf(hash, "%s%d%d", Packet.src.call, Packet.src.ssid, Packet.len);
    for(int i = 0; i < std::min(16, (int)Packet.len); i++) {
        hash[i % 15] ^= Packet.info[i];
    }
}

bool isDuplicatePacket(AX25Msg &Packet)
{
    char hash[16];
    packetHash(Packet, hash);

    unsigned long now = millis();
    clearExpiredDuplicates();

    for(uint8_t i = 0; i < DUP_PACKET_CACHE_SIZE; i++) {
        if(dupCache[i].timestamp > 0 && strncmp(dupCache[i].hash, hash, 16) == 0) {
            log_d("Duplicate packet detected: %s", hash);
            return true;
        }
    }

    // Add to cache
    strncpy(dupCache[dupCacheIndex].hash, hash, 16);
    dupCache[dupCacheIndex].timestamp = now;
    dupCacheIndex = (dupCacheIndex + 1) % DUP_PACKET_CACHE_SIZE;

    return false;
}

void clearExpiredDuplicates(void)
{
    unsigned long now = millis();
    for(uint8_t i = 0; i < DUP_PACKET_CACHE_SIZE; i++) {
        if(dupCache[i].timestamp > 0 && (now - dupCache[i].timestamp) > DUP_PACKET_TIMEOUT_MS) {
            dupCache[i].timestamp = 0;
        }
    }
}

int igateProcess(AX25Msg &Packet)
{
    int idx;

    // Check for duplicate packets
    if(isDuplicatePacket(Packet)) {
        status.dupCount++;  // Increment duplicate counter
        return 0;
    }

    if (Packet.len < 2)
    {
        status.dropCount++;
        return 0; // NO INFO DATA
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "RFONLY", 6))
        {
            status.dropCount++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "TCPIP", 5))
        {
            status.dropCount++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "qA", 2))
        {
            status.dropCount++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "NOGATE", 6))
        {
            status.dropCount++;
            return 0;
        }
    }

    // NONE Repeat from sattelite repeater
    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "RS0ISS", 6)) // Repeat from ISS
        {
            if (strchr(&Packet.rpt_list[idx].call[5], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "YBOX", 4)) // Repeat from LAPAN-A2
        {
            if (strchr(&Packet.rpt_list[idx].call[3], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "YBSAT", 5)) // Repeat from LAPAN-A2
        {
            if (strchr(&Packet.rpt_list[idx].call[4], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "PSAT", 4)) // Repeat from PSAT2-1
        {
            if (strchr(&Packet.rpt_list[idx].call[3], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "W3ADO", 5)) // Repeat from PCSAT-1
        {
            if (strchr(&Packet.rpt_list[idx].call[4], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "BJ1SI", 5)) // Repeat from LilacSat-2
        {
            if (strchr(&Packet.rpt_list[idx].call[4], '*') == NULL)
            {
                status.dropCount++;
                return 0;
            }
        }
    }

    // Memory optimization: Use char array instead of String concatenation
    char header[300]; // Fixed buffer to prevent heap fragmentation
    int headerLen = 0;

    // Build source part
    if (Packet.src.ssid > 0)
    {
        headerLen = snprintf(header, sizeof(header), "%s-%d>%s",
                           Packet.src.call, Packet.src.ssid, Packet.dst.call);
    }
    else
    {
        headerLen = snprintf(header, sizeof(header), "%s>%s",
                           Packet.src.call, Packet.dst.call);
    }

    // Add destination SSID if present
    if (Packet.dst.ssid > 0)
    {
        headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                               "-%d", Packet.dst.ssid);
    }

    // Add Path
    for (int i = 0; i < Packet.rpt_count; i++)
    {
        headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                               ",%s", Packet.rpt_list[i].call);
        if (Packet.rpt_list[i].ssid > 0)
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   "-%d", Packet.rpt_list[i].ssid);
        }
        if (Packet.rpt_flags & (1 << i))
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   "*");
        }
    }

    // Add IGATE object
    if (strlen((const char *)config.igate_object) >= 3)
    {
        if (config.aprs_ssid > 0)
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   ",%s-%d*,qAO,%s",
                                   config.aprs_mycall, config.aprs_ssid, config.igate_object);
        }
        else
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   ",%s*,qAO,%s",
                                   config.aprs_mycall, config.igate_object);
        }
    }
    else
    {
        // Add qAR path for standard IGATE
        if (config.aprs_ssid > 0)
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   ",qAR,%s-%d",
                                   config.aprs_mycall, config.aprs_ssid);
        }
        else
        {
            headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen,
                                   ",qAR,%s",
                                   config.aprs_mycall);
        }
    }

    // Add Information field separator
    headerLen += snprintf(&header[headerLen], sizeof(header) - headerLen, ":");

    uint8_t *Raw = (uint8_t *)calloc(500,sizeof(uint8_t));
    if (Raw)
    {
        memset(Raw, 0, 500); // Clear frame packet
        size_t hSize = headerLen; // Use actual header length instead of strlen()
        memcpy(&Raw[0], header, hSize);           // Copy header to frame packet
        memcpy(&Raw[hSize], &Packet.info[0], Packet.len); // Copy info to frame packet
        uint8_t *ptr = &Raw[0];
        int i, rmv = 0;
        size_t fsize=hSize + Packet.len;        
        // Remove CR,LF in frame packet
        for (i = 0; i < fsize; i++)
        {
            if ((Raw[i] == '\r') || (Raw[i] == '\n'))
            {
                ptr++;
                rmv++;
            }
            else
            {
                Raw[i] = *ptr++;
            }
            if(i>(fsize-rmv)){
                i=fsize-rmv;
                break;
            }
        }
        if(i>500 || i>fsize) i=strlen((char*)Raw);
        log_d("RF2INET: %s", Raw);
        if(aprsClient.connected()){
            aprsClient.write(&Raw[0], i); // Send binary frame packet to APRS-IS (aprsc)
            aprsClient.write("\r\n");     // Send CR LF the end frame packet
        }
        status.txCount++;
        free(Raw);
        log_d("Send TCP Finish!");
    }
    return 1;
}