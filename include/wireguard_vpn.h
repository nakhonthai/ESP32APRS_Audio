
//==============================================================================
// Wireguard VPN Client demo for LwIP/ESP32
//==============================================================================
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

//==============================================================================
//  Multi-include guard
//==============================================================================
#ifndef INC_WIREGUARD_VPN_H
#define INC_WIREGUARD_VPN_H

//==============================================================================
//  Includes
//==============================================================================

//==============================================================================
//  Defines
//==============================================================================

#define WG_LOCAL_ADDRESS        IPADDR4_INIT_BYTES(192, 168, 44, 202)
#define WG_LOCAL_NETMASK        IPADDR4_INIT_BYTES(255, 255, 255, 255)
#define WG_GATEWAY_ADDRESS      IPADDR4_INIT_BYTES(44, 63, 31, 193)
#define WG_CLIENT_PRIVATE_KEY   "gH2YqDa+St6x5eFhomVQDwtV1F0YMQd3HtOElPkZgVY="
//#define WG_CLIENT_PRIVATE_KEY   "sHueMvT+zVP7Pm/zoRptYcDkCERaBoJc/oUi9n0bmGE="
#define WG_CLIENT_PORT          51821

#define WG_PEER_PUBLIC_KEY      "ZEFr+/B/T5+k0DhVG/GOTvAOjeOiuFKmwtu/cy23xVs="
#define WG_PEER_PORT            51820
#define WG_PEER_ADDRESS         IPADDR4_INIT_BYTES(203, 150, 19, 25)

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

bool wireguard_up();

void wireguard_change_device();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // INC_WIREGUARD_VPN_H



