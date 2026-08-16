/* Initiator mode USB Mass Storage Class connection.
 * This file binds platform-specific MSC routines to the initiator mode
 * SCSI bus interface. The call structure is modeled after TinyUSB, but
 * should be usable with other USB libraries.
 *
 * ZuluSCSI™ - Copyright (c) 2023-2025 Rabbit Hole Computing™
 * Copyright (c) 2026 Eric Helgeson <eric@bluescsi.com>
 *
 * This file is licensed under the GPL version 3 or any later version. 
 * It is derived from cdrom.c in SCSI2SD V6
 *
 * https://www.gnu.org/licenses/gpl-3.0.html
 * ----
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. 
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include "BlueSCSI_config.h"
#include "BlueSCSI_log.h"
#include "BlueSCSI_log_trace.h"
#include "BlueSCSI_initiator.h"
#include "BlueSCSI_settings.h"
#include "BlueSCSI_platform_msc.h"
#ifdef BLUESCSI_NETWORK
#include "tusb_config.h"
#include "network.h"
extern bool scsiNetworkEnabled;
extern struct scsiNetworkPacketQueue scsiNetworkInboundQueue;
#endif
#include <scsi.h>
#include <BlueSCSI_platform.h>
#include <minIni.h>
#include "SdFat.h"

bool g_msc_initiator;

#ifndef PLATFORM_HAS_INITIATOR_MODE

bool setup_msc_initiator() { return false; }
void poll_msc_initiator() {}

void init_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {}
uint8_t init_msc_inquiry_device_type_cb(uint8_t lun) { (void)lun; return SCSI_DEVICE_TYPE_DIRECT_ACCESS; }
bool init_msc_inquiry_is_removable_cb(uint8_t lun) { (void)lun; return false; }
uint8_t init_msc_get_maxlun_cb(void) { return 0; }
bool init_msc_is_writable_cb (uint8_t lun) { return false; }
bool init_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) { return false; }
bool init_msc_test_unit_ready_cb(uint8_t lun) { return false; }
void init_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {}
int32_t init_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize) {return -1;}
int32_t init_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {return -1;}
int32_t init_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) { return -1;}
void init_msc_write10_complete_cb(uint8_t lun) {}

#else

// If there are multiple SCSI devices connected, they are mapped into LUNs for host.
static struct {
    int target_id;
    int config_device_index;
    uint32_t sectorsize;
    uint32_t sectorcount;
    bool use_read10; // Always use read10/write10 commands for this target
    bool block_io_supported; // True when the target should behave like a block device
    bool writable; // Cached write-protect state for the host-facing LUN
    bool media_ready; // Cached TEST UNIT READY state for refresh gating
    uint8_t device_type; // Peripheral device type from INQUIRY byte 0
    bool is_removable;   // RMB bit from INQUIRY byte 1
    bool bridge_network; // Synthetic DaynaPORT/AmigaWIFI target from config
} g_msc_initiator_targets[NUM_SCSIID];
static int g_msc_initiator_target_count;

// Sectors to read ahead of the host. The USB endpoint buffer is only one sector,
// so without read-ahead every host read costs a full SCSI command round trip.
#define MSC_PREFETCH_SECTORS 32

// Extra reads to serve without prefetching after a prefetch fails, on top of the
// failed window itself. The bad sector can be anywhere in the window, so the host
// has to walk the whole window before read-ahead is worth re-arming.
#define MSC_PREFETCH_BACKOFF_READS 64

// Prefetch next sectors in main loop while USB is transferring previous one.
static struct {
    uint8_t *prefetch_buffer; // Buffer to use for storing the data
    uint32_t prefetch_bufsize;
    uint32_t prefetch_lba; // First sector to fetch
    int prefetch_target_id; // Target to read from
    size_t prefetch_sectorcount; // Number of sectors to fetch
    size_t prefetch_sectorsize;
    uint32_t prefetch_depth; // Sectors to read ahead of the host
    uint32_t prefetch_backoff; // Reads left to skip after a failed prefetch
    bool prefetch_use_read10;
    bool prefetch_done; // True after prefetch is complete

    // Write staging for targets whose sector is larger than the USB chunk:
    // chunks accumulate in the prefetch buffer until a full sector is ready.
    uint32_t stage_lba;
    uint32_t stage_bytes;
    int stage_target_id;
    bool stage_active;

    bool readonly; // Disable writing to any drives

    // Periodic status reporting to log output
    uint32_t status_prev_time;
    uint32_t status_interval;
    uint32_t status_reqcount;
    uint32_t status_bytecount;

    // Scan new targets if none found
    uint32_t last_scan_time;
} g_msc_initiator_state;

static int get_target(uint8_t lun);
static int do_read6_or_10(int target_id, uint32_t start_sector, uint32_t sectorcount, uint32_t sectorsize, void *buffer, bool use_read10);
static int do_write6_or_10(int target_id, uint32_t start_sector, uint32_t sectorcount, uint32_t sectorsize, const uint8_t *buffer, bool use_write10);
static void scan_targets();
static bool bridge_network_is_lun(uint8_t lun);
static int32_t bridge_network_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize);
static void fill_network_inquiry(uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4], int config_device_index);
#ifdef BLUESCSI_NETWORK
static int find_bridge_network_device();
static bool bridge_network_device_present();
#else
static bool bridge_network_is_lun(uint8_t) { return false; }
static int32_t bridge_network_scsi_cb(uint8_t, const uint8_t[16], void *, uint16_t) { return -1; }
static void fill_network_inquiry(uint8_t[8], uint8_t[16], uint8_t[4], int) {}
#endif

static const char *msc_device_type_name(uint8_t device_type)
{
    switch (device_type)
    {
        case SCSI_DEVICE_TYPE_DIRECT_ACCESS:  return "DISK";
        case SCSI_DEVICE_TYPE_SEQUENTIAL:     return "TAPE";
        case SCSI_DEVICE_TYPE_PRINTER:        return "PRINTER";
        case SCSI_DEVICE_TYPE_PROCESSOR:      return "PROCESSOR";
        case SCSI_DEVICE_TYPE_WRITE_ONCE:     return "WRITE-ONCE";
        case SCSI_DEVICE_TYPE_CD:             return "CD-ROM";
        case SCSI_DEVICE_TYPE_SCANNER:        return "SCANNER";
        case SCSI_DEVICE_TYPE_MO:             return "MO";
        case SCSI_DEVICE_TYPE_MEDIA_CHANGER:  return "MEDIA-CHANGER";
        case SCSI_DEVICE_TYPE_COMMUNICATION:  return "COMMUNICATION";
        case SCSI_DEVICE_TYPE_ASC_IT8_A:      return "ASC-IT8-A";
        case SCSI_DEVICE_TYPE_ASC_IT8_B:      return "ASC-IT8-B";
        case SCSI_DEVICE_TYPE_DISK_ARRAY:     return "DISK-ARRAY";
        default:                              return "UNKNOWN";
    }
}

static bool msc_device_supports_block_io(uint8_t device_type)
{
    switch (device_type)
    {
        case SCSI_DEVICE_TYPE_DIRECT_ACCESS:
        case SCSI_DEVICE_TYPE_WRITE_ONCE:
        case SCSI_DEVICE_TYPE_CD:
        case SCSI_DEVICE_TYPE_MO:
        case SCSI_DEVICE_TYPE_DISK_ARRAY:
            return true;
        default:
            return false;
    }
}

#ifdef BLUESCSI_NETWORK
static int find_bridge_network_device()
{
    for (int i = 0; i < NUM_SCSIID; i++)
    {
        auto *cfg = g_scsi_settings.getDevice(i);
        if (cfg->deviceType == S2S_CFG_NETWORK || cfg->deviceType == S2S_CFG_AMIGAWIFI)
        {
            return i;
        }
    }
    return -1;
}

static bool bridge_network_device_present()
{
    if (!platform_network_supported())
    {
        return false;
    }
    return find_bridge_network_device() >= 0;
}

static void fill_fixed_field(char *dst, size_t dst_len, const char *src)
{
    memset(dst, ' ', dst_len);
    if (src == nullptr)
    {
        return;
    }
    size_t copy_len = strlen(src);
    if (copy_len > dst_len)
    {
        copy_len = dst_len;
    }
    memcpy(dst, src, copy_len);
}

static void fill_network_inquiry(uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4], int config_device_index)
{
    const auto *cfg = g_scsi_settings.getDevice(config_device_index);
    fill_fixed_field((char *)vendor_id, 8, cfg->vendor[0] ? cfg->vendor : "Dayna");
    fill_fixed_field((char *)product_id, 16, cfg->prodId[0] ? cfg->prodId : "SCSI/Link");
    fill_fixed_field((char *)product_rev, 4, cfg->revision[0] ? cfg->revision : "2.0f");
}

static bool bridge_network_is_lun(uint8_t lun)
{
    return lun < g_msc_initiator_target_count &&
           g_msc_initiator_targets[lun].bridge_network;
}

static const char *bridge_network_command_name(uint8_t opcode)
{
    switch (opcode)
    {
        case 0x08: return "READ(6)";
        case 0x09: return "MAC+STATS";
        case 0x0A: return "WRITE(6)";
        case 0x0C: return "SET INTERFACE MODE";
        case 0x0D: return "ADD MULTICAST";
        case 0x0E: return "TOGGLE INTERFACE";
        case 0x1A: return "MODE SENSE";
        case 0x40: return "SET MAC";
        case 0x80: return "SET MODE";
        case SCSI_NETWORK_WIFI_CMD: return "WIFI CMD";
        default: return "UNKNOWN";
    }
}

/* A USB MSC endpoint buffer is only CFG_TUD_MSC_EP_BUFSIZE bytes, yet the SCSI
 * layer asks us to fill a much larger transfer (the host's READ CDB requests up
 * to 16 KB; TinyUSB passes that as `bufsize`, not the real buffer size).  Frames
 * larger than what fits in one MSC transfer are therefore split across multiple
 * READ(6)s: each response carries one 6-byte header followed by up to
 * (CFG_TUD_MSC_EP_BUFSIZE - 6) payload bytes (4090 with the 4096-byte endpoint
 * buffer, so a full 1518-byte Ethernet frame fits in one chunk), with record
 * flag bit BRIDGE_NETWORK_CHUNK_FLAG set on every chunk except the last.  The
 * host driver reassembles the chunks before handing the frame to the network
 * stack.
 *
 * The length field always reports the payload actually returned in THIS response
 * (the whole packet for a single-chunk packet, one chunk's bytes otherwise) and
 * the payload always includes the trailing 4-byte Ethernet FCS -- matching what
 * the DaynaPORT reference and the host parser expect. */
#define BRIDGE_NETWORK_RX_HDR 6
#define BRIDGE_NETWORK_CHUNK_FLAG 0x20

/* CDB flag for WRITE(6) TX reassembly: the host splits a frame into multiple
 * WRITE(6)s (each <= CFG_TUD_MSC_EP_BUFSIZE bytes) and marks every chunk except
 * the last with this flag; the bridge buffers them and transmits once the chunk
 * with the flag clear arrives. */
#define BRIDGE_NETWORK_TX_CHUNK_FLAG 0x20

static_assert(CFG_TUD_MSC_EP_BUFSIZE > BRIDGE_NETWORK_RX_HDR,
              "CFG_TUD_MSC_EP_BUFSIZE must fit the 6-byte DaynaPORT record header");

/* Read-side streaming state: one packet larger than the endpoint buffer is
 * served across several READ(6)s.  A single global queue backs every bridge LUN
 * (its readIndex/writeIndex are global), so one shared chunk state is
 * consistent.  readIndex advances only when the final chunk is consumed; the
 * state is reset on ENABLE so a wedged or reloaded host re-syncs. */
struct
{
    bool active;      // a packet is being streamed out across READ(6)s
    uint8_t idx;      // queue index being streamed
    uint16_t offset;  // byte offset into packets[idx]
    uint16_t total;   // sizes[idx]: full packet including the 4-byte FCS
} static g_bridge_network_rx_chunk;

/* Write-side reassembly buffer: chunks of one TX frame accumulate here until the
 * final chunk arrives. */
static uint8_t g_bridge_network_tx_buf[NETWORK_PACKET_MAX_SIZE];
static uint16_t g_bridge_network_tx_len = 0;

static int32_t bridge_network_read(uint8_t *buffer, uint16_t bufsize, uint32_t size, uint8_t cdb5)
{
    const size_t cap = bufsize < CFG_TUD_MSC_EP_BUFSIZE ? (size_t)bufsize : (size_t)CFG_TUD_MSC_EP_BUFSIZE;

    if (cap < BRIDGE_NETWORK_RX_HDR)
    {
        return -1;
    }

    const size_t payload_cap = cap - BRIDGE_NETWORK_RX_HDR;
    if (payload_cap == 0)
    {
        return -1;
    }

    if (!g_bridge_network_rx_chunk.active)
    {
        if (scsiNetworkInboundQueue.readIndex == scsiNetworkInboundQueue.writeIndex)
        {
            memset(buffer, 0, BRIDGE_NETWORK_RX_HDR);
            return BRIDGE_NETWORK_RX_HDR;
        }

        g_bridge_network_rx_chunk.active = true;
        g_bridge_network_rx_chunk.idx = scsiNetworkInboundQueue.readIndex;
        g_bridge_network_rx_chunk.offset = 0;
        g_bridge_network_rx_chunk.total = scsiNetworkInboundQueue.sizes[g_bridge_network_rx_chunk.idx];
    }

    const uint16_t remaining = g_bridge_network_rx_chunk.total - g_bridge_network_rx_chunk.offset;
    const size_t chunk_len = remaining > payload_cap ? payload_cap : (size_t)remaining;
    const bool more = (size_t)g_bridge_network_rx_chunk.offset + chunk_len < g_bridge_network_rx_chunk.total;

    buffer[0] = (uint8_t)(chunk_len >> 8);
    buffer[1] = (uint8_t)(chunk_len & 0xff);
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = more ? BRIDGE_NETWORK_CHUNK_FLAG : 0;

    memcpy(buffer + BRIDGE_NETWORK_RX_HDR,
           scsiNetworkInboundQueue.packets[g_bridge_network_rx_chunk.idx] + g_bridge_network_rx_chunk.offset,
           chunk_len);
    g_bridge_network_rx_chunk.offset += (uint16_t)chunk_len;

    if (!more)
    {
        if (scsiNetworkInboundQueue.readIndex == NETWORK_PACKET_QUEUE_SIZE - 1)
        {
            scsiNetworkInboundQueue.readIndex = 0;
        }
        else
        {
            scsiNetworkInboundQueue.readIndex++;
        }
        g_bridge_network_rx_chunk.active = false;
    }

    (void)size;
    (void)cdb5;
    return (int32_t)(BRIDGE_NETWORK_RX_HDR + chunk_len);
}

static int32_t bridge_network_write(const uint8_t *buffer, uint16_t bufsize, uint32_t size, uint8_t cdb5)
{
    if (size > bufsize)
    {
        size = bufsize;
    }

    if (cdb5 & BRIDGE_NETWORK_TX_CHUNK_FLAG)
    {
        // A WRITE(6) carrying the chunk flag continues a frame started by an
        // earlier WRITE(6).  Reassemble and transmit once the final chunk (flag
        // clear) arrives.
        if (g_bridge_network_tx_len + size > sizeof(g_bridge_network_tx_buf))
        {
            g_bridge_network_tx_len = 0; // overlong chain: drop the partial frame
        }
        else
        {
            memcpy(g_bridge_network_tx_buf + g_bridge_network_tx_len, buffer, size);
            g_bridge_network_tx_len += (uint16_t)size;
        }
        return (int32_t)size;
    }

    if (cdb5 == 0x00)
    {
        if (size == 0)
        {
            // A zero-length WRITE(6) aborts any partial TX chain (the host driver
            // bailed out mid-frame) and never transmits anything.
            g_bridge_network_tx_len = 0;
            return 0;
        }
        if (g_bridge_network_tx_len > 0)
        {
            // Final chunk of a reassembled frame.
            if (g_bridge_network_tx_len + size > sizeof(g_bridge_network_tx_buf))
            {
                g_bridge_network_tx_len = 0;
                return (int32_t)size;
            }
            memcpy(g_bridge_network_tx_buf + g_bridge_network_tx_len, buffer, size);
            g_bridge_network_tx_len += (uint16_t)size;
            int32_t ret = platform_network_send(g_bridge_network_tx_buf, g_bridge_network_tx_len) == 0
                              ? (int32_t)size : -1;
            g_bridge_network_tx_len = 0;
            return ret;
        }
        return platform_network_send((uint8_t *)buffer, size) == 0 ? (int32_t)size : -1;
    }

    uint32_t pos = 0;
    while (pos + 4 <= bufsize)
    {
        uint32_t packet_size = ((uint32_t)buffer[pos] << 8) | buffer[pos + 1];
        pos += 4;
        if (packet_size == 0)
        {
            break;
        }
        if (pos + packet_size > bufsize)
        {
            break;
        }
        platform_network_send((uint8_t *)(buffer + pos), packet_size);
        pos += packet_size;
    }

    (void)size;
    return (int32_t)pos;
}

static int32_t bridge_network_wifi_command(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    uint8_t *out = (uint8_t *)buffer;
    const uint32_t size = ((uint32_t)scsi_cmd[3] << 8) | scsi_cmd[4];

    switch (scsi_cmd[1])
    {
        case SCSI_NETWORK_WIFI_CMD_SCAN:
        {
            out[0] = platform_network_wifi_start_scan() < 0 ? 0 : 1;
            return 1;
        }
        case SCSI_NETWORK_WIFI_CMD_COMPLETE:
        {
            out[0] = platform_network_wifi_scan_finished() ? 1 : 0;
            return 1;
        }
        case SCSI_NETWORK_WIFI_CMD_SCAN_RESULTS:
        {
            if (!platform_network_wifi_scan_finished() || size < 2)
            {
                platform_msc_set_sense(lun, ILLEGAL_REQUEST, 0x24, 0x00);
                return -1;
            }
            int nets = 0;
            for (int i = 0; i < WIFI_NETWORK_LIST_ENTRY_COUNT; i++)
            {
                if (wifi_network_list[i].ssid[0] == '\0')
                {
                    break;
                }
                nets++;
            }
            uint32_t netsize = sizeof(struct wifi_network_entry) * nets;
            if (netsize + 2 > bufsize)
            {
                netsize = bufsize > 2 ? (bufsize - 2) : 0;
                netsize -= netsize % sizeof(struct wifi_network_entry);
            }
            if (netsize + 2 > size)
            {
                netsize = size > 2 ? (size - 2) : 0;
                netsize -= netsize % sizeof(struct wifi_network_entry);
            }
            out[0] = (netsize >> 8) & 0xff;
            out[1] = netsize & 0xff;
            if (netsize)
            {
                memcpy(out + 2, wifi_network_list, netsize);
            }
            return (int32_t)(netsize + 2);
        }
        case SCSI_NETWORK_WIFI_CMD_INFO:
        {
            struct wifi_network_entry wifi_cur = { 0 };
            char *ssid = platform_network_wifi_ssid();
            if (ssid != nullptr)
            {
                strlcpy(wifi_cur.ssid, ssid, sizeof(wifi_cur.ssid));
            }
            char *bssid = platform_network_wifi_bssid();
            if (bssid != nullptr)
            {
                memcpy(wifi_cur.bssid, bssid, sizeof(wifi_cur.bssid));
            }
            wifi_cur.rssi = platform_network_wifi_rssi();
            wifi_cur.channel = platform_network_wifi_channel();
            if (bufsize < sizeof(wifi_cur) + 2)
            {
                return -1;
            }
            out[0] = (sizeof(wifi_cur) >> 8) & 0xff;
            out[1] = sizeof(wifi_cur) & 0xff;
            memcpy(out + 2, &wifi_cur, sizeof(wifi_cur));
            return (int32_t)(sizeof(wifi_cur) + 2);
        }
        case SCSI_NETWORK_WIFI_CMD_JOIN:
        {
            if (size != sizeof(struct wifi_join_request) || bufsize < size)
            {
                platform_msc_set_sense(lun, ILLEGAL_REQUEST, 0x24, 0x00);
                return -1;
            }
            const struct wifi_join_request *req = (const struct wifi_join_request *)buffer;
            platform_network_wifi_join((char *)req->ssid, (char *)req->key, false);
            return 0;
        }
        default:
            platform_msc_set_sense(lun, ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}

static int32_t bridge_network_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    const uint32_t size = ((uint32_t)scsi_cmd[3] << 8) | scsi_cmd[4];
    uint8_t *out = (uint8_t *)buffer;

    dbgmsg("DaynaPORT bridge LUN ", (int)lun,
           " handling opcode 0x", bytearray(&scsi_cmd[0], 1),
           " (", bridge_network_command_name(scsi_cmd[0]), ") size ", (int)size);

    switch (scsi_cmd[0])
    {
        case 0x08: // READ(6)
            dbgmsg("DaynaPORT bridge READ(6): buffer ", (int)bufsize, " bytes, cdb5=0x", bytearray(&scsi_cmd[5], 1));
            return bridge_network_read(out, bufsize, size, scsi_cmd[5]);

        case 0x09: // MAC + counters
            dbgmsg("DaynaPORT bridge MAC+STATS request");
            if (bufsize < 18)
            {
                return -1;
            }
            memcpy(out, scsiDev.boardCfg.wifiMACAddress, sizeof(scsiDev.boardCfg.wifiMACAddress));
            memset(out + sizeof(scsiDev.boardCfg.wifiMACAddress), 0, 18 - sizeof(scsiDev.boardCfg.wifiMACAddress));
            return 18;

        case 0x0A: // WRITE(6)
            dbgmsg("DaynaPORT bridge WRITE(6): buffer ", (int)bufsize, " bytes, cdb5=0x", bytearray(&scsi_cmd[5], 1));
            return bridge_network_write((const uint8_t *)buffer, bufsize, size, scsi_cmd[5]);

        case 0x0D:
            dbgmsg("DaynaPORT bridge ADD MULTICAST");
            if (size > bufsize)
            {
                return -1;
            }
            platform_network_add_multicast_address((uint8_t *)buffer);
            return 0;

        case 0x0E:
            dbgmsg("DaynaPORT bridge TOGGLE INTERFACE: ", (scsi_cmd[5] & 0x80) ? "enable" : "disable");
            if (scsi_cmd[5] & 0x80)
            {
                scsiNetworkEnabled = true;
                memset(&scsiNetworkInboundQueue, 0, sizeof(scsiNetworkInboundQueue));
                memset(&g_bridge_network_rx_chunk, 0, sizeof(g_bridge_network_rx_chunk));
                g_bridge_network_tx_len = 0;
            }
            else
            {
                scsiNetworkEnabled = false;
                memset(&g_bridge_network_rx_chunk, 0, sizeof(g_bridge_network_rx_chunk));
                g_bridge_network_tx_len = 0;
            }
            return 0;

        case 0x1A:
            dbgmsg("DaynaPORT bridge MODE SENSE (ignored)");
        case 0x40:
            if (scsi_cmd[0] == 0x40) dbgmsg("DaynaPORT bridge SET MAC (ignored)");
        case 0x80:
            if (scsi_cmd[0] == 0x80) dbgmsg("DaynaPORT bridge SET MODE (ignored)");
            return 0;

        case SCSI_NETWORK_WIFI_CMD:
            dbgmsg("DaynaPORT bridge WIFI CMD subcommand 0x", bytearray(&scsi_cmd[1], 1));
            return bridge_network_wifi_command(lun, scsi_cmd, buffer, bufsize);

        default:
            platform_msc_set_sense(lun, ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}
#endif

static void refresh_target_writable_cache(uint8_t lun)
{
    if (lun >= g_msc_initiator_target_count)
    {
        return;
    }

    g_msc_initiator_targets[lun].writable = false;

    if (g_msc_initiator_state.readonly || !g_msc_initiator_targets[lun].block_io_supported)
    {
        return;
    }

    int target = get_target(lun);
    uint8_t command[6] = {0x1A, 0x08, 0, 0, 4, 0}; // MODE SENSE(6)
    uint8_t response[4] = {0};

    LED_ON();
    g_msc_initiator_state.status_reqcount++;
    int status = scsiInitiatorRunCommand(target, command, 6, response, 4, NULL, 0);
    LED_OFF();

    if (status == 0)
    {
        g_msc_initiator_targets[lun].writable = (response[2] & 0x80) == 0;
    }
}

static bool ensure_targets_scanned()
{
    static bool logged_scan_delay = false;

    if (g_msc_initiator_target_count > 0)
    {
        return true;
    }

    // The first host query can arrive before the attached target has settled,
    // especially on slower bridges. Retry a few times before giving up so the
    // bridge does not advertise an empty model string.
    for (int attempt = 0; attempt < 3 && g_msc_initiator_target_count == 0; attempt++)
    {
        if (!logged_scan_delay)
        {
            logmsg("USB MSC target scan delayed, rescanning for SCSI devices");
            logged_scan_delay = true;
        }
        platform_reset_watchdog();
        scan_targets();
        if (g_msc_initiator_target_count > 0)
        {
            break;
        }
        platform_delay_ms(250);
    }

    return g_msc_initiator_target_count > 0;
}

static void publish_last_sense_to_host(uint8_t lun, int target_id, const char *command_text)
{
    uint8_t sense_key;
    uint8_t sense_asc;
    uint8_t sense_ascq;
    if (scsiGetLastRequestSense(&sense_key, &sense_asc, &sense_ascq))
    {
        logmsg("USB MSC sense publish: LUN ", (int)lun,
               " target ", target_id,
               " command ", command_text,
               " sense ", (int)sense_key, "/", (int)sense_asc, "/", (int)sense_ascq);
        platform_msc_set_sense(lun, sense_key, sense_asc, sense_ascq);
    }
    else
    {
        logmsg("USB MSC sense publish: LUN ", (int)lun,
               " target ", target_id,
               " command ", command_text,
               " no cached sense");
    }
}

static void scan_targets()
{
    int found_count = 0;
    int initiator_id = scsiInitiatorGetOwnID();
    uint8_t inquiry_data[36] = {0};
    g_msc_initiator_target_count = 0;
    for (int target_id = 0; target_id < NUM_SCSIID; target_id++)
    {
        if (target_id == initiator_id) continue;

        // INQUIRY answers with or without media loaded, so it detects the
        // device itself. Empty removable media reports "medium not present"
        // later, through the normal sense path.
        bool inquiryok = scsiInquiry(target_id, inquiry_data);
        if (!inquiryok) continue;

        char vendor_id[9] = {0};
        char product_id[17] = {0};
        memcpy(vendor_id, &inquiry_data[8], 8);
        memcpy(product_id, &inquiry_data[16], 16);
        uint8_t device_type = inquiry_data[0] & 0x1F;
        bool is_removable = (inquiry_data[1] & 0x80) != 0;
        g_msc_initiator_targets[found_count].target_id = target_id;
        g_msc_initiator_targets[found_count].config_device_index = -1;
        g_msc_initiator_targets[found_count].device_type = device_type;
        g_msc_initiator_targets[found_count].is_removable = is_removable;
        g_msc_initiator_targets[found_count].block_io_supported = msc_device_supports_block_io(device_type);
        g_msc_initiator_targets[found_count].writable = false;
        g_msc_initiator_targets[found_count].media_ready = false;
        g_msc_initiator_targets[found_count].bridge_network = false;
        const char *type_name = msc_device_type_name(device_type);

        bool ready = scsiTestUnitReady(target_id);
        uint32_t sectorcount = 0, sectorsize = 0;
        bool readcapok = ready &&
            scsiInitiatorReadCapacity(target_id, &sectorcount, &sectorsize);

        if (readcapok)
        {
            if (g_msc_initiator_targets[found_count].block_io_supported)
            {
                logmsg("Found SCSI device with ID ", target_id, ": ", vendor_id, " ", product_id,
                    " (", type_name, ")",
                    " capacity ", (int)(((uint64_t)sectorcount * sectorsize) / 1024 / 1024), " MB");
                g_msc_initiator_targets[found_count].sectorcount = sectorcount;
                g_msc_initiator_targets[found_count].sectorsize = sectorsize;
                g_msc_initiator_targets[found_count].use_read10 = scsiInitiatorTestSupportsRead10(target_id, sectorsize);
                g_msc_initiator_targets[found_count].media_ready = true;
                refresh_target_writable_cache(found_count);
            }
            else
            {
                logmsg("Found SCSI device with ID ", target_id, ": ", vendor_id, " ", product_id,
                    " (", type_name, ")",
                    " reported capacity but is exposed as raw-only");
                g_msc_initiator_targets[found_count].sectorcount = 0;
                g_msc_initiator_targets[found_count].sectorsize = 0;
                g_msc_initiator_targets[found_count].use_read10 = true;
                g_msc_initiator_targets[found_count].writable = false;
                g_msc_initiator_targets[found_count].media_ready = false;
            }
        }
        else if (ready && g_msc_initiator_targets[found_count].block_io_supported &&
                 device_type == SCSI_DEVICE_TYPE_DIRECT_ACCESS)
        {
            logmsg("Found SCSI device with ID ", target_id, ": ", vendor_id, " ", product_id,
                   " (", type_name, ")",
                   " but failed to read capacity. Assuming SCSI-1 drive up to 1 GB.");
            g_msc_initiator_targets[found_count].sectorcount = 2097152;
            g_msc_initiator_targets[found_count].sectorsize = 512;
            g_msc_initiator_targets[found_count].use_read10 = false;
            g_msc_initiator_targets[found_count].media_ready = true;
            refresh_target_writable_cache(found_count);
        }
        else
        {
            if (g_msc_initiator_targets[found_count].block_io_supported)
            {
                // Capacity gets probed again by the host once media is loaded
                logmsg("Found SCSI device with ID ", target_id, ": ", vendor_id, " ", product_id,
                       " (", type_name, ") - no media present");
            }
            else
            {
                logmsg("Found SCSI device with ID ", target_id, ": ", vendor_id, " ", product_id,
                       " (", type_name, ") - raw-only bridge target");
            }
            g_msc_initiator_targets[found_count].sectorcount = 0;
            g_msc_initiator_targets[found_count].sectorsize = 0;
            g_msc_initiator_targets[found_count].use_read10 = true;
            g_msc_initiator_targets[found_count].writable = false;
            g_msc_initiator_targets[found_count].media_ready = false;
        }
        found_count++;
    }

#ifdef BLUESCSI_NETWORK
    if (found_count < NUM_SCSIID && bridge_network_device_present())
    {
        int config_device_index = find_bridge_network_device();
        if (config_device_index >= 0)
        {
            g_msc_initiator_targets[found_count].target_id = -1;
            g_msc_initiator_targets[found_count].config_device_index = config_device_index;
            g_msc_initiator_targets[found_count].device_type = SCSI_DEVICE_TYPE_PROCESSOR;
            g_msc_initiator_targets[found_count].is_removable = false;
            g_msc_initiator_targets[found_count].block_io_supported = false;
            g_msc_initiator_targets[found_count].writable = false;
            g_msc_initiator_targets[found_count].media_ready = true;
            g_msc_initiator_targets[found_count].sectorcount = 0;
            g_msc_initiator_targets[found_count].sectorsize = 0;
            g_msc_initiator_targets[found_count].use_read10 = true;
            g_msc_initiator_targets[found_count].bridge_network = true;
            logmsg("Found configured network device in bridge mode: ",
                   g_scsi_settings.getDevice(config_device_index)->vendor, " ",
                   g_scsi_settings.getDevice(config_device_index)->prodId,
                   " (", msc_device_type_name(SCSI_DEVICE_TYPE_PROCESSOR), ")");
            found_count++;
        }
    }
#endif

    // USB MSC requests can start processing after we set this
    g_msc_initiator_target_count = found_count;
}

bool setup_msc_initiator()
{
    if (platform_is_pico_w()) {
        platform_disable_led();
    }
    logmsg("SCSI Initiator: activating USB raw bridge mode");
    g_msc_initiator = true;

    // We can use the device mode buffer for prefetching data in initiator mode.
    // The buffer is needed even with read-ahead disabled: sectors larger than
    // the USB endpoint buffer are bounced through it one chunk at a time.
    g_msc_initiator_state.prefetch_buffer = scsiDev.data;
    g_msc_initiator_state.prefetch_bufsize = sizeof(scsiDev.data);

    if (!ini_getbool("SCSI", "InitiatorMSCDisablePrefetch", false, CONFIGFILE))
    {
        g_msc_initiator_state.prefetch_depth = ini_getl("SCSI", "InitiatorMSCPrefetchSectors",
                                                        MSC_PREFETCH_SECTORS, CONFIGFILE);
        logmsg("--- Initiator prefetch: ", (int)g_msc_initiator_state.prefetch_depth, " sectors read-ahead");
    }
    else
    {
        g_msc_initiator_state.prefetch_depth = 0;
    }

    g_msc_initiator_state.status_interval = ini_getl("SCSI", "InitiatorMSCStatusInterval", 5000, CONFIGFILE);
    g_msc_initiator_state.readonly = ini_getbool("SCSI", "InitiatorMSCReadOnly", false, CONFIGFILE);

    if (g_msc_initiator_state.readonly)
    {
        logmsg("--- Initiator is configured in read-only mode: writes to device are prevented");
    }

    scsiInitiatorInit();

    // Scan for targets
    scan_targets();

    logmsg("SCSI Initiator: found " , g_msc_initiator_target_count, " SCSI devices");
    return g_msc_initiator_target_count > 0;
}

void poll_msc_initiator()
{
    uint32_t time_now = platform_millis();
    uint32_t time_since_scan = time_now - g_msc_initiator_state.last_scan_time;
    if (g_msc_initiator_target_count == 0 && time_since_scan > 5000)
    {
        // Scan for targets until we find one - drive might be slow to start up.
        // MSC lock is not required here because commands will early exit when target_count is 0.
        platform_reset_watchdog();
        scan_targets();
        g_msc_initiator_state.last_scan_time = time_now;
    }

    uint32_t delta = time_now - g_msc_initiator_state.status_prev_time;
    if (g_msc_initiator_state.status_interval > 0 &&
        delta > g_msc_initiator_state.status_interval)
    {
        if (g_msc_initiator_state.status_reqcount > 0)
        {
            logmsg("USB MSC: ", (int)g_msc_initiator_state.status_reqcount, " commands, ",
                   (int)(g_msc_initiator_state.status_bytecount / delta), " kB/s");
        }

        g_msc_initiator_state.status_reqcount = 0;
        g_msc_initiator_state.status_bytecount = 0;
        g_msc_initiator_state.status_prev_time = time_now;
    }


    platform_poll();
    platform_msc_lock_set(true); // Cannot handle new MSC commands while running prefetch
    if (g_msc_initiator_state.prefetch_sectorcount > 0
        && !g_msc_initiator_state.prefetch_done
        && !g_msc_initiator_state.stage_active)
    {
        LED_ON();

        dbgmsg("Prefetch ", (int)g_msc_initiator_state.prefetch_lba, " + ",
                (int)g_msc_initiator_state.prefetch_sectorcount, "x",
                (int)g_msc_initiator_state.prefetch_sectorsize);
        // Read next block while USB is transferring
        int status = do_read6_or_10(g_msc_initiator_state.prefetch_target_id,
                                    g_msc_initiator_state.prefetch_lba,
                                    g_msc_initiator_state.prefetch_sectorcount,
                                    g_msc_initiator_state.prefetch_sectorsize,
                                    g_msc_initiator_state.prefetch_buffer,
                                    g_msc_initiator_state.prefetch_use_read10);
        if (status == 0)
        {
            g_msc_initiator_state.prefetch_done = true;
        }
        else
        {
            // The failure may be a bad sector anywhere in the window, so serve reads
            // directly until the host has walked past the whole window. Backing off
            // less than the window re-arms into the same bad sector and fails again.
            logmsg("Prefetch of sector ", g_msc_initiator_state.prefetch_lba, " + ",
                   (int)g_msc_initiator_state.prefetch_sectorcount, " failed: status ", status);
            g_msc_initiator_state.prefetch_backoff = g_msc_initiator_state.prefetch_sectorcount
                                                   + MSC_PREFETCH_BACKOFF_READS;
            g_msc_initiator_state.prefetch_sectorcount = 0;
        }

        LED_OFF();
    }
    platform_msc_lock_set(false);
}

static int get_target(uint8_t lun)
{
    if (lun >= g_msc_initiator_target_count)
    {
        logmsg("Host requested access to non-existing lun ", (int)lun);
        return 0;
    }
    else
    {
        return g_msc_initiator_targets[lun].target_id;
    }
}

// Whether the host should be allowed to eject media in this device
static bool is_removable_device(uint8_t lun)
{
    if (lun >= g_msc_initiator_target_count)
        return false;

    uint8_t dtype = g_msc_initiator_targets[lun].device_type;

    // CD-ROM and MO are always removable; direct access devices are
    // removable when the RMB bit is set (Zip, removable disks)
    return (dtype == SCSI_DEVICE_TYPE_CD) ||
           (dtype == SCSI_DEVICE_TYPE_MO) ||
           (dtype == SCSI_DEVICE_TYPE_DIRECT_ACCESS && g_msc_initiator_targets[lun].is_removable);
}

void init_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    dbgmsg("-- MSC Inquiry");

    if (!ensure_targets_scanned())
    {
        memset(vendor_id, 0, 8);
        memset(product_id, 0, 8);
        memset(product_rev, 0, 8);
        return;
    }

    if (bridge_network_is_lun(lun))
    {
        fill_network_inquiry(vendor_id, product_id, product_rev,
                             g_msc_initiator_targets[lun].config_device_index);
        return;
    }

    LED_ON();
    g_msc_initiator_state.status_reqcount++;

    int target = get_target(lun);
    uint8_t response[36] = {0};
    bool status = scsiInquiry(target, response);
    if (!status)
    {
        logmsg("SCSI Inquiry to target ", target, " failed");
    }

    memcpy(vendor_id, &response[8], 8);
    memcpy(product_id, &response[16], 16);
    memcpy(product_rev, &response[32], 4);

    LED_OFF();
}

uint8_t init_msc_inquiry_device_type_cb(uint8_t lun)
{
    if (!ensure_targets_scanned())
    {
        return SCSI_DEVICE_TYPE_DIRECT_ACCESS;
    }

    if (lun >= g_msc_initiator_target_count)
    {
        return SCSI_DEVICE_TYPE_DIRECT_ACCESS;
    }

    if (bridge_network_is_lun(lun))
    {
        return SCSI_DEVICE_TYPE_PROCESSOR;
    }

    return g_msc_initiator_targets[lun].device_type;
}

bool init_msc_inquiry_is_removable_cb(uint8_t lun)
{
    if (!ensure_targets_scanned())
    {
        return false;
    }

    if (lun >= g_msc_initiator_target_count)
    {
        return false;
    }

    if (bridge_network_is_lun(lun))
    {
        return false;
    }

    return is_removable_device(lun);
}

uint8_t init_msc_get_maxlun_cb(void)
{
    ensure_targets_scanned();
    return g_msc_initiator_target_count;
}

bool init_msc_is_writable_cb (uint8_t lun)
{
    if (!ensure_targets_scanned())
    {
        return false;
    }

    if (g_msc_initiator_target_count == 0)
    {
        return false;
    }

    if (g_msc_initiator_state.readonly)
    {
        return false;
    }

    if (bridge_network_is_lun(lun))
    {
        return false;
    }

    if (!g_msc_initiator_targets[lun].block_io_supported)
    {
        return false;
    }

    return g_msc_initiator_targets[lun].writable;
}

bool init_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    dbgmsg("-- MSC Start Stop, start: ", (int)start, ", load_eject: ", (int)load_eject);

    if (!ensure_targets_scanned())
    {
        return false;
    }

    if (g_msc_initiator_target_count == 0)
    {
        return false;
    }

    if (bridge_network_is_lun(lun))
    {
        return true;
    }

    LED_ON();
    g_msc_initiator_state.status_reqcount++;

    int target = get_target(lun);
    uint8_t command[6] = {0x1B, 0x00, 0, 0, 0, 0};
    uint8_t response[4] = {0};

    if (start)
    {
        command[4] |= 1; // Start
    }

    // LoEj moves media, so only removable devices get it
    if (load_eject && is_removable_device(lun))
    {
        logmsg(start ? "Loading removable media" : "Ejecting removable media");
        command[4] |= 2;
    }

    command[4] |= static_cast<uint8_t>((power_condition & 0x0F) << 4);

    int status = scsiInitiatorRunCommand(target,
                                         command, sizeof(command),
                                         response, sizeof(response),
                                         NULL, 0);

    if (status == 2)
    {
        scsiClearLastRequestSense();
        uint8_t sense_key;
        scsiRequestSense(target, &sense_key);
        publish_last_sense_to_host(lun, target, "START STOP UNIT");
        scsiLogInitiatorCommandFailure("START STOP UNIT", target, status, sense_key);
    }

    LED_OFF();

    return status == 0;
}

bool init_msc_test_unit_ready_cb(uint8_t lun)
{
    dbgmsg("-- MSC Test Unit Ready");

    if (!ensure_targets_scanned())
    {
        return false;
    }

    if (g_msc_initiator_target_count == 0)
    {
        return false;
    }

    if (bridge_network_is_lun(lun))
    {
        return true;
    }

    g_msc_initiator_state.status_reqcount++;
    scsiClearLastRequestSense();
    bool ready = scsiTestUnitReady(get_target(lun));
    if (g_msc_initiator_targets[lun].block_io_supported)
    {
        if (!ready)
        {
            g_msc_initiator_targets[lun].media_ready = false;
            g_msc_initiator_targets[lun].writable = false;
        }
        else
        {
            g_msc_initiator_targets[lun].media_ready = true;
            g_msc_initiator_targets[lun].writable = true;
        }
    }
    if (!ready)
    {
        publish_last_sense_to_host(lun, get_target(lun), "TEST UNIT READY");
    }
    return ready;
}

void init_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    dbgmsg("-- MSC Get Capacity");
    g_msc_initiator_state.status_reqcount++;

    if (!ensure_targets_scanned())
    {
        *block_count = 0;
        *block_size = 0;
        return;
    }

    if (g_msc_initiator_target_count == 0 || lun >= g_msc_initiator_target_count)
    {
        *block_count = 0;
        *block_size = 0;
        return;
    }

    if (bridge_network_is_lun(lun))
    {
        *block_count = 0;
        *block_size = 0;
        return;
    }

    uint32_t sectorcount = 0;
    uint32_t sectorsize = 0;
    bool success = scsiInitiatorReadCapacity(get_target(lun), &sectorcount, &sectorsize);

    if (success && sectorcount > 0)
    {
        // Remember the current capacity so it survives a not-ready phase
        g_msc_initiator_targets[lun].sectorcount = sectorcount;
        g_msc_initiator_targets[lun].sectorsize = sectorsize;
        *block_count = sectorcount;
        *block_size = sectorsize;
    }
    else
    {
        // READ CAPACITY fails while removable media is loading; returning the
        // stored capacity keeps the host from flapping the device to 0 blocks
        *block_count = g_msc_initiator_targets[lun].sectorcount;
        *block_size = g_msc_initiator_targets[lun].sectorsize;
        dbgmsg("---- Using stored capacity: ", (int)*block_count, " x ", (int)*block_size);
    }
}

int32_t init_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    if (!ensure_targets_scanned())
    {
        return -1;
    }

    if (g_msc_initiator_target_count == 0)
    {
        return -1;
    }

    if (bridge_network_is_lun(lun))
    {
#ifdef BLUESCSI_NETWORK
        return bridge_network_scsi_cb(lun, scsi_cmd, buffer, bufsize);
#else
        return -1;
#endif
    }

    dbgmsg("-- MSC Raw SCSI command ", bytearray(scsi_cmd, 16));
    LED_ON();
    g_msc_initiator_state.status_reqcount++;

    // NOTE: the TinyUSB API around free-form commands is not very good,
    // this function could need improvement.
    
    // Figure out command length
    static const uint8_t CmdGroupBytes[8] = {6, 10, 10, 6, 16, 12, 6, 6}; // From SCSI2SD
    int cmdlen = CmdGroupBytes[scsi_cmd[0] >> 5];

    int target = get_target(lun);
    int status = scsiInitiatorRunCommand(target,
                                         scsi_cmd, cmdlen,
                                         NULL, 0,
                                         (const uint8_t*)buffer, bufsize);

    if (status != 0)
    {
        scsiClearLastRequestSense();
        uint8_t sense_key;
        scsiRequestSense(target, &sense_key);
                publish_last_sense_to_host(lun, target, "READ CAPACITY");
    }

    LED_OFF();

    return status;
}

static int do_read6_or_10(int target_id, uint32_t start_sector, uint32_t sectorcount, uint32_t sectorsize, void *buffer, bool use_read10)
{
    int status;

    // Read6 command supports 21 bit LBA - max of 0x1FFFFF
    // ref: https://www.seagate.com/files/staticfiles/support/docs/manual/Interface%20manuals/100293068j.pdf pg 134
    bool fits_read6 = (start_sector < 0x1FFFFF && sectorcount <= 256);
    if (!use_read10 && fits_read6)
    {
        // Use READ6 command for compatibility with old SCSI1 drives
        // Note that even with SCSI1 drives we have no choice but to use READ10 if the drive
        // size is larger than 1 GB, as the sector number wouldn't fit in the command.
        uint8_t command[6] = {0x08,
            (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8),
            (uint8_t)start_sector,
            (uint8_t)sectorcount,
            0x00
        };

        // Note: we must not call platform poll in the commands,
        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), (uint8_t*)buffer, sectorcount * sectorsize, NULL, 0);
    }
    else
    {
        // Use READ10 command for larger number of blocks
        uint8_t command[10] = {0x28, 0x00,
            (uint8_t)(start_sector >> 24), (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8), (uint8_t)start_sector,
            0x00,
            (uint8_t)(sectorcount >> 8), (uint8_t)(sectorcount),
            0x00
        };

        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), (uint8_t*)buffer, sectorcount * sectorsize, NULL, 0);
    }

    return status;
}

// Serve a host read chunk smaller than one device sector, e.g. 2048-byte
// MO media behind a 512-byte USB endpoint buffer. TinyUSB chunks such
// transfers and passes the byte offset inside the current sector; the
// sector is read into the prefetch buffer once and handed out chunk by
// chunk. Must not return 0: TinyUSB treats that as "retry" and re-invokes
// the callback in a loop that starves the watchdog.
static int32_t init_msc_read_partial(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    int target_id = get_target(lun);
    uint32_t sectorsize = g_msc_initiator_targets[lun].sectorsize;
    uint32_t disk_sectorcount = g_msc_initiator_targets[lun].sectorcount;
    bool use_read10 = g_msc_initiator_targets[lun].use_read10;

    if (g_msc_initiator_state.prefetch_bufsize < sectorsize || lba >= disk_sectorcount)
    {
        logmsg("USB read of ", (int)bufsize, " bytes at LBA ", (int)lba, " offset ", (int)offset,
               " failed: sector size ", (int)sectorsize, " unsupported");
        return -1;
    }

    bool covered = g_msc_initiator_state.prefetch_done
                && g_msc_initiator_state.prefetch_target_id == target_id
                && g_msc_initiator_state.prefetch_sectorsize == sectorsize
                && lba >= g_msc_initiator_state.prefetch_lba
                && lba < g_msc_initiator_state.prefetch_lba + g_msc_initiator_state.prefetch_sectorcount;

    if (!covered)
    {
        LED_ON();
        uint32_t depth = g_msc_initiator_state.prefetch_depth;
        uint32_t max_by_buffer = g_msc_initiator_state.prefetch_bufsize / sectorsize;
        uint32_t max_by_disk = disk_sectorcount - lba;
        if (depth < 1) depth = 1;
        if (depth > max_by_buffer) depth = max_by_buffer;
        if (depth > max_by_disk) depth = max_by_disk;

        dbgmsg("USB Read command ", (int)lba, " offset ", (int)offset, ", reading ",
               (int)depth, "x", (int)sectorsize, " into bounce buffer");
        int status = do_read6_or_10(target_id, lba, depth, sectorsize,
                                    g_msc_initiator_state.prefetch_buffer, use_read10);
        if (status != 0 && depth > 1)
        {
            // A bad sector later in the window should not fail this chunk
            scsiClearLastRequestSense();
            uint8_t sense_key;
            scsiRequestSense(target_id, &sense_key);
            depth = 1;
            status = do_read6_or_10(target_id, lba, depth, sectorsize,
                                    g_msc_initiator_state.prefetch_buffer, use_read10);
        }
        LED_OFF();

        if (status != 0)
        {
            scsiClearLastRequestSense();
            uint8_t sense_key;
            scsiRequestSense(target_id, &sense_key);
            if (sense_key == RECOVERED_ERROR)
            {
                dbgmsg("SCSI Initiator read: RECOVERED_ERROR at ", (int)lba);
            }
            else
            {
                publish_last_sense_to_host(lun, target_id, "READ");
                scsiLogInitiatorCommandFailure("SCSI Initiator read", target_id, status, sense_key);
                g_msc_initiator_state.prefetch_sectorcount = 0;
                g_msc_initiator_state.prefetch_done = false;
                return -1;
            }
        }

        g_msc_initiator_state.prefetch_lba = lba;
        g_msc_initiator_state.prefetch_target_id = target_id;
        g_msc_initiator_state.prefetch_sectorcount = depth;
        g_msc_initiator_state.prefetch_sectorsize = sectorsize;
        g_msc_initiator_state.prefetch_use_read10 = use_read10;
        g_msc_initiator_state.prefetch_done = true;
    }

    uint32_t len = sectorsize - offset;
    if (len > bufsize) len = bufsize;
    memcpy(buffer, g_msc_initiator_state.prefetch_buffer
                   + (lba - g_msc_initiator_state.prefetch_lba) * sectorsize + offset, len);

    if (offset == 0)
    {
        g_msc_initiator_state.status_reqcount++;
    }
    g_msc_initiator_state.status_bytecount += len;
    return len;
}

int32_t init_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    if (g_msc_initiator_target_count == 0)
    {
        return -1;
    }

    if (g_msc_initiator_targets[lun].sectorsize == 0)
    {
        return -1;
    }

    if (offset != 0 || bufsize < g_msc_initiator_targets[lun].sectorsize)
    {
        return init_msc_read_partial(lun, lba, offset, (uint8_t*)buffer, bufsize);
    }

    LED_ON();
    int status = 0;

    int target_id = get_target(lun);
    int sectorsize = g_msc_initiator_targets[lun].sectorsize;
    bool use_read10 = g_msc_initiator_targets[lun].use_read10;
    uint32_t sectorcount = bufsize / sectorsize;
    uint32_t total_sectorcount = sectorcount;
    uint32_t orig_lba = lba;

    // Prefetch buffer is shared by all targets, so it is only valid for the
    // target it was filled from - and only at the sector size it was read
    // with, which can change when removable media is swapped.
    if (g_msc_initiator_state.prefetch_done &&
        g_msc_initiator_state.prefetch_target_id == target_id &&
        g_msc_initiator_state.prefetch_sectorsize == (size_t)sectorsize)
    {
        int32_t offset = (int32_t)lba - (int32_t)g_msc_initiator_state.prefetch_lba;
        uint8_t *dest = (uint8_t*)buffer;
        while (offset >= 0 && offset < g_msc_initiator_state.prefetch_sectorcount && sectorcount > 0)
        {
            // Copy sectors from prefetch
            memcpy(dest, g_msc_initiator_state.prefetch_buffer + sectorsize * offset, sectorsize);
            dest += sectorsize;
            offset += 1;
            lba += 1;
            sectorcount -= 1;
        }
    }

    if (sectorcount > 0)
    {
        dbgmsg("USB Read command ", (int)orig_lba, " + ", (int)total_sectorcount, "x", (int)sectorsize,
               " got ", (int)(total_sectorcount - sectorcount), " sectors from prefetch");
        status = do_read6_or_10(target_id, lba, sectorcount, sectorsize, buffer, use_read10);
        lba += sectorcount;
    }
    else
    {
        dbgmsg("USB Read command ", (int)orig_lba, " + ", (int)total_sectorcount, "x", (int)sectorsize, " fully satisfied from prefetch");
    }

    g_msc_initiator_state.status_reqcount++;
    g_msc_initiator_state.status_bytecount += total_sectorcount * sectorsize;
    LED_OFF();

    if (status != 0)
    {
        scsiClearLastRequestSense();
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);

        if (sense_key == RECOVERED_ERROR)
        {
            dbgmsg("SCSI Initiator read: RECOVERED_ERROR at ", (int)orig_lba);
        }
        else if (sense_key == UNIT_ATTENTION)
        {
            dbgmsg("SCSI Initiator read: UNIT_ATTENTION");
        }
        else
        {
            publish_last_sense_to_host(lun, target_id, "READ");
            scsiLogInitiatorCommandFailure("SCSI Initiator read", target_id, status, sense_key);
            return -1;
        }
    }

    // Keep the current prefetch while it still covers the sectors the host is
    // walking towards, so a read-ahead window is fetched once and then served
    // from RAM instead of being discarded and re-read on every request.
    uint32_t disk_sectorcount = g_msc_initiator_targets[lun].sectorcount;
    bool covered = g_msc_initiator_state.prefetch_done
                && g_msc_initiator_state.prefetch_target_id == target_id
                && g_msc_initiator_state.prefetch_sectorsize == (size_t)sectorsize
                && lba >= g_msc_initiator_state.prefetch_lba
                && lba < g_msc_initiator_state.prefetch_lba + g_msc_initiator_state.prefetch_sectorcount;

    if (g_msc_initiator_state.prefetch_backoff > 0)
    {
        g_msc_initiator_state.prefetch_backoff--;
    }
    else if (!covered && lba < disk_sectorcount)
    {
        uint32_t depth = g_msc_initiator_state.prefetch_depth;
        uint32_t max_by_buffer = g_msc_initiator_state.prefetch_bufsize / sectorsize;
        uint32_t max_by_disk = disk_sectorcount - lba;
        if (depth > max_by_buffer) depth = max_by_buffer;
        if (depth > max_by_disk) depth = max_by_disk;

        if (depth > 0)
        {
            // Request prefetch of the next block while USB transfers the previous one
            g_msc_initiator_state.prefetch_lba = lba;
            g_msc_initiator_state.prefetch_target_id = target_id;
            g_msc_initiator_state.prefetch_sectorcount = depth;
            g_msc_initiator_state.prefetch_sectorsize = sectorsize;
            g_msc_initiator_state.prefetch_use_read10 = use_read10;
            g_msc_initiator_state.prefetch_done = false;
        }
    }

    return total_sectorcount * sectorsize;
}

static int do_write6_or_10(int target_id, uint32_t start_sector, uint32_t sectorcount, uint32_t sectorsize, const uint8_t *buffer, bool use_write10)
{
    int status;

    // Write6 command supports 21 bit LBA - max of 0x1FFFFF
    bool fits_write6 = (start_sector < 0x1FFFFF && sectorcount <= 256);
    if (!use_write10 && fits_write6)
    {
        // Use WRITE6 command for compatibility with old SCSI1 drives
        uint8_t command[6] = {0x0A,
            (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8),
            (uint8_t)start_sector,
            (uint8_t)sectorcount,
            0x00
        };

        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), NULL, 0, buffer, sectorcount * sectorsize);
    }
    else
    {
        // Use WRITE10 command for larger number of blocks
        uint8_t command[10] = {0x2A, 0x00,
            (uint8_t)(start_sector >> 24), (uint8_t)(start_sector >> 16),
            (uint8_t)(start_sector >> 8), (uint8_t)start_sector,
            0x00,
            (uint8_t)(sectorcount >> 8), (uint8_t)(sectorcount),
            0x00
        };

        status = scsiInitiatorRunCommand(target_id, command, sizeof(command), NULL, 0, buffer, sectorcount * sectorsize);
    }

    return status;
}

// Check write status and decide whether the operation counts as failed.
static int32_t check_write_status(uint8_t lun, int target_id, int status, uint32_t start_sector);

static bool flush_write_cache(uint8_t lun, bool force)
{
    if (!g_msc_initiator_state.stage_active || g_msc_initiator_state.stage_bytes == 0)
    {
        return true;
    }

    int target_id = g_msc_initiator_state.stage_target_id;
    uint32_t sectorsize = g_msc_initiator_targets[lun].sectorsize;
    uint32_t full_bytes = g_msc_initiator_state.stage_bytes - (g_msc_initiator_state.stage_bytes % sectorsize);
    if (full_bytes == 0)
    {
        return !force;
    }

    bool use_read10 = g_msc_initiator_targets[lun].use_read10;
    uint32_t sectorcount = full_bytes / sectorsize;
    uint32_t start_sector = g_msc_initiator_state.stage_lba;

    dbgmsg("USB Write cache flush ", (int)start_sector, " + ", (int)sectorcount, "x", (int)sectorsize);

    LED_ON();
    int status = do_write6_or_10(target_id, start_sector, sectorcount, sectorsize,
                                 g_msc_initiator_state.prefetch_buffer, use_read10);
    LED_OFF();

    if (check_write_status(lun, target_id, status, start_sector) != 0)
    {
        return false;
    }

    g_msc_initiator_state.status_reqcount++;
    g_msc_initiator_state.status_bytecount += full_bytes;

    uint32_t leftover = g_msc_initiator_state.stage_bytes - full_bytes;
    g_msc_initiator_state.stage_lba += sectorcount;
    if (leftover > 0)
    {
        memmove(g_msc_initiator_state.prefetch_buffer,
                g_msc_initiator_state.prefetch_buffer + full_bytes,
                leftover);
        g_msc_initiator_state.stage_bytes = leftover;
    }
    else
    {
        g_msc_initiator_state.stage_active = false;
        g_msc_initiator_state.stage_bytes = 0;
    }

    return true;
}

// Check write status and decide whether the operation counts as failed.
static int32_t check_write_status(uint8_t lun, int target_id, int status, uint32_t start_sector)
{
    if (status != 0)
    {
        scsiClearLastRequestSense();
        uint8_t sense_key;
        scsiRequestSense(target_id, &sense_key);

        if (sense_key == RECOVERED_ERROR)
        {
            dbgmsg("SCSI Initiator write: RECOVERED_ERROR at ", (int)start_sector);
        }
        else if (sense_key == UNIT_ATTENTION)
        {
            dbgmsg("SCSI Initiator write: UNIT_ATTENTION");
        }
        else
        {
            publish_last_sense_to_host(lun, target_id, "WRITE");
            scsiLogInitiatorCommandFailure("SCSI Initiator write", target_id, status, sense_key);
            return -1;
        }
    }

    return 0;
}

// Accept a host write chunk, stage it in the bounce buffer, and flush
// contiguous runs to the target in larger batches.
static int32_t append_write_data(uint8_t lun, uint32_t lba, uint32_t offset, const uint8_t *buffer, uint32_t bufsize)
{
    int target_id = get_target(lun);
    uint32_t sectorsize = g_msc_initiator_targets[lun].sectorsize;

    if (g_msc_initiator_state.prefetch_bufsize < sectorsize)
    {
        logmsg("USB write of ", (int)bufsize, " bytes at LBA ", (int)lba,
               " failed: sector size ", (int)sectorsize, " unsupported");
        return -1;
    }

    if (!g_msc_initiator_state.stage_active)
    {
        if (offset != 0)
        {
            logmsg("USB write chunk out of sequence at LBA ", (int)lba, " offset ", (int)offset);
            return -1;
        }

        g_msc_initiator_state.stage_active = true;
        g_msc_initiator_state.stage_lba = lba;
        g_msc_initiator_state.stage_target_id = target_id;
        g_msc_initiator_state.stage_bytes = 0;
    }
    else if (g_msc_initiator_state.stage_target_id != target_id ||
             g_msc_initiator_state.stage_lba + (g_msc_initiator_state.stage_bytes / sectorsize) != lba ||
             (g_msc_initiator_state.stage_bytes % sectorsize) != offset)
    {
        logmsg("USB write chunk out of sequence at LBA ", (int)lba, " offset ", (int)offset);
        g_msc_initiator_state.stage_active = false;
        return -1;
    }

    // Staging reuses the prefetch buffer, so drop any cached read data.
    g_msc_initiator_state.prefetch_sectorcount = 0;
    g_msc_initiator_state.prefetch_done = false;

    uint32_t stage_capacity = g_msc_initiator_state.prefetch_bufsize;
    uint32_t consumed = 0;
    while (consumed < bufsize)
    {
        if (!g_msc_initiator_state.stage_active)
        {
            g_msc_initiator_state.stage_active = true;
            g_msc_initiator_state.stage_target_id = target_id;
            g_msc_initiator_state.stage_bytes = 0;
        }

        if (g_msc_initiator_state.stage_bytes == stage_capacity)
        {
            if (!flush_write_cache(lun, false))
            {
                return -1;
            }
            continue;
        }

        uint32_t room = stage_capacity - g_msc_initiator_state.stage_bytes;
        uint32_t len = bufsize - consumed;
        if (len > room)
        {
            len = room;
        }

        memcpy(g_msc_initiator_state.prefetch_buffer + g_msc_initiator_state.stage_bytes, buffer + consumed, len);
        g_msc_initiator_state.stage_bytes += len;
        consumed += len;

        if (g_msc_initiator_state.stage_bytes == stage_capacity)
        {
            if (!flush_write_cache(lun, false))
            {
                return -1;
            }
        }
    }

    return bufsize;
}

int32_t init_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (g_msc_initiator_target_count == 0)
    {
        return -1;
    }

    if (g_msc_initiator_state.readonly)
    {
        logmsg("--- Refusing host write request, InitiatorMSCReadOnly is set.");
        return -1;
    }

    if (g_msc_initiator_targets[lun].sectorsize == 0)
    {
        return -1;
    }

    if (!g_msc_initiator_targets[lun].block_io_supported)
    {
        logmsg("--- Refusing host write request, target is raw-only.");
        return -1;
    }

    return append_write_data(lun, lba, offset, buffer, bufsize);
}

void init_msc_write10_complete_cb(uint8_t lun)
{
    if (lun >= g_msc_initiator_target_count)
    {
        return;
    }

    if (!flush_write_cache(lun, true))
    {
        logmsg("USB write cache flush failed for LUN ", (int)lun);
    }
    g_msc_initiator_state.stage_active = false;
}


#endif
