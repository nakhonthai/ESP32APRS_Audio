//==============================================================================
//  Multi-include guard
//==============================================================================
#ifndef INC_WIREGUARD_VPN_H
#define INC_WIREGUARD_VPN_H

//==============================================================================
// Wireguard VPN Client demo for LwIP/ESP32
//==============================================================================
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "lwip/tcpip.h"

#ifdef CONFIG_LWIP_TCPIP_CORE_LOCKING
  #define TCP_MUTEX_LOCK()                                \
    if (!sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER)) { \
      LOCK_TCPIP_CORE();                                  \
    }

  #define TCP_MUTEX_UNLOCK()                             \
    if (sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER)) { \
      UNLOCK_TCPIP_CORE();                               \
    }
#else // CONFIG_LWIP_TCPIP_CORE_LOCKING
  #define TCP_MUTEX_LOCK()
  #define TCP_MUTEX_UNLOCK()
#endif // CONFIG_LWIP_TCPIP_CORE_LOCKING

//==============================================================================
//  Includes
//==============================================================================

//==============================================================================
//  Defines
//==============================================================================

// #define WG_LOCAL_ADDRESS        IPADDR4_INIT_BYTES(192, 168, 44, 202)
// #define WG_LOCAL_NETMASK        IPADDR4_INIT_BYTES(255, 255, 255, 255)
// #define WG_GATEWAY_ADDRESS      IPADDR4_INIT_BYTES(192, 168, 44, 195)
// #define WG_CLIENT_PRIVATE_KEY   "gH2YqDa+St6x5eFhomVQDwtV1F0YMQd3HtOElPkZgVY="
// #define WG_CLIENT_PORT          51821

// #define WG_PEER_PUBLIC_KEY      "ZEFr+/B/T5+k0DhVG/GOTvAOjeOiuFKmwtu/cy23xVs="
// #define WG_PEER_PORT            51820
// #define WG_PEER_ADDRESS         IPADDR4_INIT_BYTES(203, 150, 19, 23)

//==============================================================================
//  Exported types
//==============================================================================

//==============================================================================
//  Exported data
//==============================================================================

//==============================================================================
//  Exported functions
//==============================================================================
bool wireguard_active();
void wireguard_remove();
void wireguard_setup();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // INC_WIREGUARD_VPN_H



