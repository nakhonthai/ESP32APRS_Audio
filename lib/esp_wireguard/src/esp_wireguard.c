/*
 * Copyright (c) 2022 Tomoyuki Sakurai <y@trombik.org>
 * Copyright (c) 2023-2024 Simone Rossetto <simros85@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 * 3. Neither the name of "Floorsense Ltd", "Agile Workspace Ltd" nor the names of
 *  its contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "esp_wireguard.h"

#include <assert.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "esp_wireguard_err.h"
#include "esp_wireguard_log.h"
#include "mbedtls/base64.h"

#include "wireguard-platform.h"
#include "wireguardif.h"

#define TAG "esp_wireguard"
#define WG_KEY_LEN  (32)
#define WG_B64_KEY_LEN (4 * ((WG_KEY_LEN + 2) / 3))

#if defined(CONFIG_LWIP_IPV6)
#define WG_ADDRSTRLEN  INET6_ADDRSTRLEN
#else
#define WG_ADDRSTRLEN  INET_ADDRSTRLEN
#endif

static struct netif wg_netif_struct = {0};
static struct netif *wg_netif = NULL;
static struct wireguardif_peer peer = {0};
static uint8_t wireguard_peer_index = WIREGUARDIF_INVALID_INDEX;
static uint8_t preshared_key_decoded[WG_KEY_LEN];

static void esp_wireguard_dns_query_callback(const char *hostname, const ip_addr_t *ipaddr, wireguard_config_t *config) {
    if(ipaddr) {
        ESP_LOGV(TAG, "dns_query_callback: hostname %s resolved to ip %s", hostname, ipaddr_ntoa(ipaddr));
        ip_addr_copy(config->endpoint_ip, *ipaddr);
    } else {
        ESP_LOGW(TAG, "dns_query_callback: cannot resolve hostname %s", hostname);
        ip_addr_set_zero(&(config->endpoint_ip));
    }
}

static esp_err_t esp_wireguard_peer_init(const wireguard_config_t *config, struct wireguardif_peer *peer)
{
    esp_err_t err;

    if (!config || !peer) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if(ip_addr_isany(&(config->endpoint_ip))) {
        ESP_LOGE(TAG, "peer_init: invalid endpoint ip: `%s`", ipaddr_ntoa(&(config->endpoint_ip)));
        err = ESP_ERR_INVALID_IP;
        goto fail;
    }
    ip_addr_copy(peer->endpoint_ip, config->endpoint_ip);

    peer->public_key = config->public_key;
    if (config->preshared_key != NULL) {
        size_t len;
        int res;

        ESP_LOGI(TAG, "using preshared_key");
        //ESP_LOGD(TAG, "preshared_key: %s", config->preshared_key);
        res = mbedtls_base64_decode(preshared_key_decoded, WG_KEY_LEN, &len, (unsigned char *)config->preshared_key, WG_B64_KEY_LEN);
        if (res != 0 || len != WG_KEY_LEN) {
            err = ESP_FAIL;
            ESP_LOGE(TAG, "base64_decode: %i", res);
            if (len != WG_KEY_LEN) {
                ESP_LOGE(TAG, "invalid decoded length, len: %u, should be %u", len, WG_KEY_LEN);
            }
            goto fail;
        }
        peer->preshared_key = preshared_key_decoded;
    } else {
        peer->preshared_key = NULL;
    }
    peer->keep_alive = config->persistent_keepalive;

    /* Allow device's own address through tunnel */
    {
        if(ipaddr_aton(config->address, &(peer->allowed_ip)) != 1) {
            ESP_LOGE(TAG, "peer_init: invalid address: `%s`", config->address);
            err = ESP_ERR_INVALID_ARG;
            goto fail;
        }

        // Only the single IP is allowed, thus /32 netmask, leaving to the user
        // the responsibility to set the appropriate list of other allowed IPs.
        ip_addr_t allowed_mask = IPADDR4_INIT_BYTES(255, 255, 255, 255);
        peer->allowed_mask = allowed_mask;
    }
    ESP_LOGI(TAG, "default allowed_ip: %s/%s", config->address, ipaddr_ntoa(&(peer->allowed_mask)));

    peer->endport_port = config->port;
    peer->keep_alive = config->persistent_keepalive;
    err = ESP_OK;
fail:
    return err;
}

static esp_err_t esp_wireguard_netif_create(const wireguard_config_t *config)
{
    esp_err_t err;
    ip_addr_t ip_addr;
    ip_addr_t netmask;
    ip_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);
    struct wireguardif_init_data wg = {0};

    if (!config) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    /* Setup the WireGuard device structure */
    wg.private_key = config->private_key;
    wg.listen_port = config->listen_port;
    wg.bind_netif = NULL;

    if (ipaddr_aton(config->address, &ip_addr) != 1) {
        ESP_LOGE(TAG, "netif_create: invalid address: `%s`", config->address);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if (ipaddr_aton(config->netmask, &netmask) != 1) {
        ESP_LOGE(TAG, "netif_create: invalid netmask: `%s`", config->netmask);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    /* Register the new WireGuard network interface with lwIP */
    wg_netif = netif_add(
            &wg_netif_struct,
            ip_2_ip4(&ip_addr),
            ip_2_ip4(&netmask),
            ip_2_ip4(&gateway),
            &wg, &wireguardif_init,
            &ip_input);
    if (wg_netif == NULL) {
        ESP_LOGE(TAG, "netif_add: failed");
        err = ESP_FAIL;
        goto fail;
    }

    /* Mark the interface as administratively up, link up flag is set
     * automatically when peer connects */
    netif_set_up(wg_netif);
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_init(wireguard_config_t *config, wireguard_ctx_t *ctx)
{
    esp_err_t err = ESP_FAIL;

    if (!config || !ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    /* start async hostname resolution */
    if(dns_gethostbyname(
            config->endpoint,
            &(config->endpoint_ip),
            (dns_found_callback)&esp_wireguard_dns_query_callback,
            config) == ERR_ARG) {
        ESP_LOGE(TAG, "init: dns client not initialized or invalid hostname");
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }

    err = wireguard_platform_init();
    if (err != ESP_OK) {
#if !defined(LIBRETINY)
        ESP_LOGE(TAG, "wireguard_platform_init: %s", esp_err_to_name(err));
#else // !defined(LIBRETINY)
        ESP_LOGE(TAG, "wireguard_platform_init: %d", err);
#endif // !defined(LIBRETINY)
        goto fail;
    }
    ctx->config = config;
    ctx->netif = NULL;
    ctx->netif_default = netif_default;

    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_connect(wireguard_ctx_t *ctx)
{
    esp_err_t err = ESP_FAIL;
    err_t lwip_err = -1;

    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if (ctx->netif == NULL) {
        err = esp_wireguard_netif_create(ctx->config);
        if (err != ESP_OK) {
#if !defined(LIBRETINY)
            ESP_LOGE(TAG, "netif_create: %s", esp_err_to_name(err));
#else // !defined(LIBRETINY)
            ESP_LOGE(TAG, "netif_create: %d", err);
#endif // !defined(LIBRETINY)
            goto fail;
        }
        ctx->netif = wg_netif;
        ctx->netif_default = netif_default;
    }

    /* start another async hostname resolution in case the first was executed too early */
    lwip_err = dns_gethostbyname(
            ctx->config->endpoint,
            &(ctx->config->endpoint_ip),
            (dns_found_callback)&esp_wireguard_dns_query_callback,
            ctx->config);

    switch(lwip_err) {
        case ERR_OK:
            ESP_LOGV(TAG, "connect: endpoint ip ready (%s)", ipaddr_ntoa(&(ctx->config->endpoint_ip)));
            break;

        case ERR_INPROGRESS:
            ESP_LOGI(TAG, "connect: dns resolution of endpoint hostname is still in progress");
            err = ESP_ERR_RETRY;
            goto fail;

        case ERR_ARG:
            ESP_LOGE(TAG, "connect: dns client not initialized or invalid hostname");
            err = ESP_ERR_INVALID_STATE;
            goto fail;

        default:
            ESP_LOGE(TAG, "connect: unknown error `%i` in hostname resolution", lwip_err);
            err = ESP_FAIL;
            goto fail;
    }

        /* Initialize the first WireGuard peer structure */
        err = esp_wireguard_peer_init(ctx->config, &peer);
        if (err != ESP_OK) {
#if !defined(LIBRETINY)
            ESP_LOGE(TAG, "peer_init: %s", esp_err_to_name(err));
#else // !defined(LIBRETINY)
            ESP_LOGE(TAG, "peer_init: %d", err);
#endif // !defined(LIBRETINY)
            goto fail;
        }

        /* Register the new WireGuard peer with the network interface */
        lwip_err = wireguardif_add_peer(ctx->netif, &peer, &wireguard_peer_index);
        if (lwip_err != ERR_OK || wireguard_peer_index == WIREGUARDIF_INVALID_INDEX) {
            ESP_LOGE(TAG, "wireguardif_add_peer: %i", lwip_err);
            err = ESP_FAIL;
            goto fail;
        }
        if (ip_addr_isany(&peer.endpoint_ip)) {
            err = ESP_FAIL;
            goto fail;
        }

    ESP_LOGI(TAG, "connecting to %s (%s), port %i", ctx->config->endpoint, ipaddr_ntoa(&(peer.endpoint_ip)), peer.endport_port);
    lwip_err = wireguardif_connect(ctx->netif, wireguard_peer_index);
    if (lwip_err != ERR_OK) {
        ESP_LOGE(TAG, "wireguardif_connect: %i", lwip_err);
        err = ESP_FAIL;
        goto fail;
    }
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_set_default(const wireguard_ctx_t *ctx)
{
    esp_err_t err;
    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if(!ctx->netif) {
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }
    netif_set_default(ctx->netif);
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_restore_default(const wireguard_ctx_t *ctx)
{
    esp_err_t err;
    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }
    if(!ctx->netif_default) {
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }
    netif_set_default(ctx->netif_default);
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_disconnect(wireguard_ctx_t *ctx)
{
    esp_err_t err;
    err_t lwip_err;

    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    // Clear the IP address to gracefully disconnect any clients while the
    // peers are still valid
    netif_set_ipaddr(ctx->netif, IP4_ADDR_ANY4);

    lwip_err = wireguardif_disconnect(ctx->netif, wireguard_peer_index);
    if (lwip_err != ERR_OK) {
        ESP_LOGW(TAG, "wireguardif_disconnect: peer_index: %" PRIu8 " err: %i", wireguard_peer_index, lwip_err);
    }

    lwip_err = wireguardif_remove_peer(ctx->netif, wireguard_peer_index);
    if (lwip_err != ERR_OK) {
        ESP_LOGW(TAG, "wireguardif_remove_peer: peer_index: %" PRIu8 " err: %i", wireguard_peer_index, lwip_err);
    }

    wireguard_peer_index = WIREGUARDIF_INVALID_INDEX;
    wireguardif_shutdown(ctx->netif);
    netif_remove(ctx->netif);
    wireguardif_fini(ctx->netif);
    netif_set_default(ctx->netif_default);
    ctx->netif = NULL;

    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_peer_is_up(const wireguard_ctx_t *ctx)
{
    esp_err_t err;
    err_t lwip_err;

    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    lwip_err = wireguardif_peer_is_up(
            ctx->netif,
            wireguard_peer_index,
            &peer.endpoint_ip,
            &peer.endport_port);

    if (lwip_err != ERR_OK) {
        err = ESP_FAIL;
        goto fail;
    }
    err = ESP_OK;
fail:
    return err;
}

esp_err_t esp_wireguard_latest_handshake(const wireguard_ctx_t *ctx, time_t *result)
{
    esp_err_t err;

    if (!ctx) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if (!ctx->netif) {
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }

    *result = wireguardif_latest_handshake(ctx->netif, wireguard_peer_index);
    err = (*result > 0) ? ESP_OK : ESP_FAIL;

fail:
    return err;
}

esp_err_t esp_wireguard_add_allowed_ip(const wireguard_ctx_t *ctx, const char *allowed_ip, const char *allowed_ip_mask)
{
    esp_err_t err;
    err_t lwip_err;

    ip_addr_t ip_addr;
    ip_addr_t netmask;

    if (!ctx || !allowed_ip || !allowed_ip_mask) {
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if (!ctx->netif) {
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }

    if (ipaddr_aton(allowed_ip, &ip_addr) != 1) {
        ESP_LOGE(TAG, "add_allowed_ip: invalid allowed_ip: `%s`", allowed_ip);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    if (ipaddr_aton(allowed_ip_mask, &netmask) != 1) {
        ESP_LOGE(TAG, "add_allowed_ip: invalid allowed_ip_mask: `%s`", allowed_ip_mask);
        err = ESP_ERR_INVALID_ARG;
        goto fail;
    }

    ESP_LOGI(TAG, "add allowed_ip: %s/%s", allowed_ip, allowed_ip_mask);
    lwip_err = wireguardif_add_allowed_ip(ctx->netif, wireguard_peer_index, ip_addr, netmask);
    err = (lwip_err == ERR_OK ? ESP_OK : ESP_FAIL);

fail:
    return err;
}
// vim: expandtab tabstop=4
