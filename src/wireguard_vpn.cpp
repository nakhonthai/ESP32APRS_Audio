//==============================================================================
// Wireguard VPN Client demo for LwIP/ESP32
//==============================================================================

//==============================================================================
//  Includes
//==============================================================================
#include <Arduino.h>
#include "main.h"
#include "wireguard_vpn.h"

// ใช้ตัวแปรโกลบอลในไฟล์ main.cpp
extern Configuration config;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "VPN";
static int s_retry_num = 0;
static wireguard_config_t wg_config = ESP_WIREGUARD_CONFIG_DEFAULT();

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

esp_err_t wireguard_setup(wireguard_ctx_t* ctx)
{
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "Initializing WireGuard.");
    wg_config.private_key = config.wg_private_key;
    wg_config.listen_port = config.wg_port + 1;
    wg_config.public_key = config.wg_public_key;
    // if (strcmp(CONFIG_WG_PRESHARED_KEY, "") != 0) {
    //     wg_config.preshared_key = CONFIG_WG_PRESHARED_KEY;
    // } else {
    //     wg_config.preshared_key = NULL;
    // }
    wg_config.address = config.wg_local_address;
    wg_config.netmask = config.wg_netmask_address;
    wg_config.endpoint = config.wg_peer_address;
    wg_config.port = config.wg_port;
    wg_config.persistent_keepalive = 120;

    err = esp_wireguard_init(&wg_config, ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wireguard_init: %s", esp_err_to_name(err));
        goto fail;
    }

    // ESP_LOGI(TAG, "Connecting to the peer.");
    // err = esp_wireguard_connect(ctx);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "esp_wireguard_connect: %s", esp_err_to_name(err));
    //     goto fail;
    // }

    //err = ESP_OK;
fail:
    return err;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include "wireguardif.h"
#include "wireguard.h"
//==============================================================================
//  Defines
//==============================================================================
#define CMP_NAME "WG_VPN"


    //==============================================================================
    //  Local types
    //==============================================================================

    //==============================================================================
    //  Local data
    //==============================================================================
    static struct netif wg_netif_struct = {0};
    static struct netif *wg_netif = NULL;
    static uint8_t wireguard_peer_index_local = WIREGUARDIF_INVALID_INDEX;

    //==============================================================================
    //  Exported data
    //==============================================================================

    //==============================================================================
    //  Local functions
    //==============================================================================

    //==============================================================================
    //  Exported functions
    //==============================================================================
    bool wireguard_active()
    {
        if (wg_netif != NULL)
            return true;
        return false;
    }

    void wireguard_remove()
    {

        if (wg_netif != NULL)
        {
            wireguardif_disconnect(wg_netif, wireguard_peer_index_local);
            wireguardif_remove_peer(wg_netif, wireguard_peer_index_local);
            // netif_set_down(wg_netif);
            // netif_remove(&wg_netif_struct);
        }
    }

    void wireguard_setup_old()
    {
        struct wireguardif_init_data wg;
        struct wireguardif_peer peer;
        ip_addr_t ipaddr = WG_LOCAL_ADDRESS;
        ip_addr_t netmask = WG_LOCAL_NETMASK;
        ip_addr_t gateway = WG_GATEWAY_ADDRESS;
        ip_addr_t peer_address = WG_PEER_ADDRESS;

        ipaddr_aton(config.wg_local_address, &ipaddr);
        ipaddr_aton(config.wg_netmask_address, &netmask);
        ipaddr_aton(config.wg_gw_address, &gateway);
        ipaddr_aton(config.wg_peer_address, &peer_address);

        // Setup the WireGuard device structure
        // wg.private_key = WG_CLIENT_PRIVATE_KEY;
        // wg.listen_port = WG_CLIENT_PORT;
        wg.private_key = config.wg_private_key;
        wg.listen_port = config.wg_port + 1;
        wg.bind_netif = NULL; // NB! not working on ESP32 even if set!
        
        TCP_MUTEX_LOCK();
        if (wg_netif == NULL)
        {
            
            // Register the new WireGuard network interface with lwIP
            wg_netif = netif_add(&wg_netif_struct, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gateway), &wg, &wireguardif_init, &ip_input);

            // Mark the interface as administratively up, link up flag is set automatically when peer connects
            netif_set_up(wg_netif);
            
        }

        // Initialise the first WireGuard peer structure
        wireguardif_peer_init(&peer);
        // peer.public_key = WG_PEER_PUBLIC_KEY;
        peer.public_key = config.wg_public_key;
        peer.preshared_key = NULL;
        // Allow all IPs through tunnel
        // peer.allowed_ip = IPADDR4_INIT_BYTES(0, 0, 0, 0);
        IP_ADDR4(&peer.allowed_ip, 0, 0, 0, 0);
        IP_ADDR4(&peer.allowed_mask, 0, 0, 0, 0);

        // If we know the endpoint's address can add here
        ip_addr_set(&peer.endpoint_ip, &peer_address);
        // peer.endport_port = WG_PEER_PORT;
        peer.endport_port = config.wg_port;

        // Register the new WireGuard peer with the netwok interface
        wireguardif_add_peer(wg_netif, &peer, &wireguard_peer_index_local);

        if ((wireguard_peer_index_local != WIREGUARDIF_INVALID_INDEX) && !ip_addr_isany(&peer.endpoint_ip))
        {
            // Start outbound connection to peer
            wireguardif_connect(wg_netif, wireguard_peer_index_local);
        }
        TCP_MUTEX_UNLOCK();
    }

#ifdef __cplusplus
}
#endif // __cplusplus