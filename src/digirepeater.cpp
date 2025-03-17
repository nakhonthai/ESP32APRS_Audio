#include "digirepeater.h"
#include "main.h"

RTC_DATA_ATTR digiTLMType digiLog;
RTC_DATA_ATTR uint8_t digiCount = 0;

extern Configuration config;

int digiProcess(AX25Msg &Packet)
{
    int idx, j;
    uint8_t ctmp;
    // if(!DIGI) return;
    // if(rx_data) return;
    // if(digi_timeout<aprs_delay) return;
    // digi_timeout = 65530;
    // aprs_delay = 65535;

    j = 0;
    if (Packet.len < 5)
    {
        digiLog.ErPkts++;
        return 0; // NO DST
    }

    if (!strncmp(&Packet.src.call[0], "NOCALL", 6))
    {
        digiLog.DropRx++;
        return 0;
    }
    if (!strncmp(&Packet.src.call[0], "MYCALL", 6))
    {
        digiLog.DropRx++;
        return 0;
    }

    // Destination SSID Trace
    if (Packet.dst.ssid > 0)
    {
        uint8_t ctmp = Packet.dst.ssid & 0x1E; // Check DSSID

        if (ctmp > 15)
            ctmp = 0;
        if (ctmp < 8)
        { // Edit PATH Change to TRACEn-N
            if (ctmp > 0)
                ctmp--;
            Packet.dst.ssid = ctmp;
            if (Packet.rpt_count > 0)
            {
                for (idx = 0; idx < Packet.rpt_count; idx++)
                {
                    if (!strcmp(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0])) // Is path same callsign
                    {
                        if (Packet.rpt_list[idx].ssid == config.digi_ssid) // IS path same SSID
                        {
                            if (Packet.rpt_flags & (1 << idx))
                            {
                                digiLog.DropRx++;
                                return 0; // bypass flag *
                            }
                            Packet.rpt_flags |= (1 << idx);
                            return 1;
                        }
                    }
                    if (Packet.rpt_flags & (1 << idx))
                        continue;
                    for (j = idx; j < Packet.rpt_count; j++)
                    {
                        if (Packet.rpt_flags & (1 << j))
                            break;
                    }
                    // Move current part to next part
                    for (; j >= idx; j--)
                    {
                        int n = j + 1;
                        strcpy(&Packet.rpt_list[n].call[0], &Packet.rpt_list[j].call[0]);
                        Packet.rpt_list[n].ssid = Packet.rpt_list[j].ssid;
                        if (Packet.rpt_flags & (1 << j))
                            Packet.rpt_flags |= (1 << n);
                        else
                            Packet.rpt_flags &= ~(1 << n);
                    }

                    // Add new part
                    Packet.rpt_count += 1;
                    strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                    Packet.rpt_list[idx].ssid = config.digi_ssid;
                    Packet.rpt_flags |= (1 << idx);
                    return 2;
                    // j = 1;
                    // break;
                }
            }
            else
            {
                idx = 0;
                strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                Packet.rpt_list[idx].ssid = config.digi_ssid;
                Packet.rpt_flags |= (1 << idx);
                Packet.rpt_count += 1;
                return 2;
            }
        }
        else
        {
            digiLog.DropRx++;
            return 0; // NO PATH
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "qA", 2))
        {
            digiLog.DropRx++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (!strncmp(&Packet.rpt_list[idx].call[0], "TCP", 3))
        {
            digiLog.DropRx++;
            return 0;
        }
    }

    for (idx = 0; idx < Packet.rpt_count; idx++)
    {
        if (Packet.rpt_flags & (1 << idx))
        {
            if (idx == (Packet.rpt_count - 1))
                digiCount++;
            continue; // bypass flag *
        }
        if (!strncmp(&Packet.rpt_list[idx].call[0], "WIDE", 4))
        {
            // Check WIDEn-N
            if (Packet.rpt_list[idx].ssid > 0)
            {
                if (Packet.rpt_flags & (1 << idx))
                    continue; // bypass flag *
                ctmp = Packet.rpt_list[idx].ssid & 0x1F;
                if (ctmp > 0)
                    ctmp--;
                if (ctmp > 15)
                    ctmp = 0;
                if (ctmp == 0)
                {
                    strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                    Packet.rpt_list[idx].ssid = config.digi_ssid;
                    Packet.rpt_flags |= (1 << idx);
                    j = 2;
                    break;
                }
                else
                {
                    Packet.rpt_list[idx].ssid = ctmp;
                    Packet.rpt_flags &= ~(1 << idx);
                    j = 2;
                    break;
                }
            }
            else
            {
                j = 2;
                strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                Packet.rpt_list[idx].ssid = config.digi_ssid;
                Packet.rpt_flags |= (1 << idx);
                break;
            }
        }
        else if (!strncmp(&Packet.rpt_list[idx].call[0], "TRACE", 5))
        {
            if (Packet.rpt_flags & (1 << idx))
                continue; // bypass flag *
            ctmp = Packet.rpt_list[idx].ssid & 0x1F;
            if (ctmp > 0)
                ctmp--;
            if (ctmp > 15)
                ctmp = 0;
            if (ctmp == 0)
            {
                strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                Packet.rpt_list[idx].ssid = config.digi_ssid;
                Packet.rpt_flags |= (1 << idx);
                j = 2;
                break;
            }
            else
            {
                for (j = idx; j < Packet.rpt_count; j++)
                {
                    if (Packet.rpt_flags & (1 << j))
                        break;
                }
                // Move current part to next part
                for (; j >= idx; j--)
                {
                    int n = j + 1;
                    strcpy(&Packet.rpt_list[n].call[0], &Packet.rpt_list[j].call[0]);
                    Packet.rpt_list[n].ssid = Packet.rpt_list[j].ssid;
                    if (Packet.rpt_flags & (1 << j))
                        Packet.rpt_flags |= (1 << n);
                    else
                        Packet.rpt_flags &= ~(1 << n);
                }
                // Reduce N part of TRACEn-N
                Packet.rpt_list[idx + 1].ssid = ctmp;

                // Add new part
                Packet.rpt_count += 1;
                strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
                Packet.rpt_list[idx].ssid = config.digi_ssid;
                Packet.rpt_flags |= (1 << idx);
                j = 2;
                break;
            }
        }
        else if (!strncmp(&Packet.rpt_list[idx].call[0], "RFONLY", 6))
        {
            j = 2;
            // strcpy(&Packet.rpt_list[idx].call[0], &config.aprs_mycall[0]);
            // Packet.rpt_list[idx].ssid = config.aprs_ssid;
            Packet.rpt_flags |= (1 << idx);
            break;
        }
        else if (!strncmp(&Packet.rpt_list[idx].call[0], "RELAY", 5))
        {
            j = 2;
            strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
            Packet.rpt_list[idx].ssid = config.digi_ssid;
            Packet.rpt_flags |= (1 << idx);
            break;
        }
        else if (!strncmp(&Packet.rpt_list[idx].call[0], "GATE", 4))
        {
            j = 2;
            strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
            Packet.rpt_list[idx].ssid = config.digi_ssid;
            Packet.rpt_flags |= (1 << idx);
            break;
        }
        else if (!strncmp(&Packet.rpt_list[idx].call[0], "ECHO", 4))
        {
            j = 2;
            strcpy(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0]);
            Packet.rpt_list[idx].ssid = config.digi_ssid;
            Packet.rpt_flags |= (1 << idx);
            break;
        }
        else if (!strcmp(&Packet.rpt_list[idx].call[0], &config.digi_mycall[0])) // Is path same callsign
        {
            ctmp = Packet.rpt_list[idx].ssid & 0x1F;
            if (ctmp == config.digi_ssid) // IS path same SSID
            {
                if (Packet.rpt_flags & (1 << idx))
                {
                    digiLog.DropRx++;
                    break; // bypass flag *
                }
                Packet.rpt_flags |= (1 << idx);
                j = 1;
                break;
            }
            else
            {
                j = 0;
                break;
            }
        }
        else
        {
            j = 0;
            break;
        }
    }
    return j;
}