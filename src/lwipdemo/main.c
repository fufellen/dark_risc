#include <io.h>
#include <stdio.h>
#include <string.h>

#include "lwip/init.h"
#include "lwip/ip.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "netif/ethernet.h"

#define DARKETH_BASE 0x80000000u

#define ETH_STATUS_RX_AVAILABLE 0x00000001u
#define ETH_STATUS_RX_OVERFLOW  0x00000002u
#define ETH_STATUS_RX_DROPPED   0x00000004u
#define ETH_STATUS_RX_READY     0x00000100u

#define ETH_RX_CTRL_RELEASE     0x00000001u
#define ETH_RX_CTRL_CLEAR_FLAGS 0x00000002u

#define ETH_TX_STATUS_READY     0x00000001u
#define ETH_TX_STATUS_BUSY      0x00000002u
#define ETH_TX_STATUS_OVERFLOW  0x00000004u
#define ETH_TX_STATUS_DONE      0x00000008u
#define ETH_TX_STATUS_WRITTEN   0x00000010u

#define ETH_TX_CTRL_START       0x00000001u
#define ETH_TX_CTRL_ABORT       0x00000002u
#define ETH_TX_CTRL_CLEAR_FLAGS 0x00000004u

#define ETH_CFG_MAC_FILTER_ENABLE 0x00000001u
#define ETH_CFG_ACCEPT_BROADCAST  0x00000002u
#define ETH_CFG_ACCEPT_MULTICAST  0x00000004u

#define LIDARSIM_MAX_FRAME        1518u
#define LIDARSIM_UART_LINE_MAX    64u
#define LIDARSIM_DISCOVERY_PORT   50103u
#define LIDARSIM_DATA_PORT        50100u
#define LIDARSIM_CMD_PORT         50101u
#define LIDARSIM_FIRMWARE_PORT    50102u
#define LIDARSIM_MODEL            "R120_FAKE"
#define LIDARSIM_FIRMWARE         "pegus_1"
#define LIDARSIM_MSOP_POINTS      180u
#define LIDARSIM_MSOP_DISTANCE_BYTES 2u
#define LIDARSIM_MSOP_INTENSITY_BYTES 0u
#define LIDARSIM_MSOP_ECHO_COUNT  2u
#define LIDARSIM_MSOP_ECHO_MODE   3u
#define LIDARSIM_MSOP_POINT_BYTES \
    (LIDARSIM_MSOP_ECHO_COUNT * \
     (LIDARSIM_MSOP_DISTANCE_BYTES + LIDARSIM_MSOP_INTENSITY_BYTES))
#define LIDARSIM_MSOP_PACKET_MAX \
    (2u + 34u + LIDARSIM_MSOP_POINTS * LIDARSIM_MSOP_POINT_BYTES + 2u)
#define LIDARSIM_MSOP_INVALID_DISTANCE 0xffffu
#define LIDARSIM_MSOP_PERIOD_MS   20u
#define LIDARSIM_MSOP_TX_BUFFERS  2u
#define LIDARSIM_PIG_PERIOD_MS    1500u
#define LIDARSIM_CONTROL_BUF_MAX  320u
#define LIDARSIM_CONTROL_REPLY_MAX 320u
#define LIDARSIM_FIRMWARE_BUF_MAX 320u

#define LIDAR_DISCOVERY_RESPONSE_SIZE 80u
#define LIDAR_DISCOVERY_REQUEST_SIZE  17u

#define LIDAR_PROTO_VERSION 12u
#define LIDAR_PROTO_FIXED   0xa0u
#define LIDAR_PROTO_VARLEN  0xa1u
#define LIDAR_PROTO_READ    0u
#define LIDAR_PROTO_WRITE   1u

#define LIDAR_CMD_FULL_STATUS       0x00u
#define LIDAR_CMD_STATUS            0x01u
#define LIDAR_CMD_VOLTAGE           0x02u
#define LIDAR_CMD_MOTOR_TARGET      0x10u
#define LIDAR_CMD_MOTOR_CURRENT     0x11u
#define LIDAR_CMD_VOLTAGE_TARGET_LD 0x20u
#define LIDAR_CMD_VOLTAGE_TARGET_PD 0x21u
#define LIDAR_CMD_VOLTAGE_CURRENT_LD 0x22u
#define LIDAR_CMD_VOLTAGE_CURRENT_PD 0x23u
#define LIDAR_CMD_TEMPERATURE_LD    0x24u
#define LIDAR_CMD_TEMPERATURE_PD    0x25u
#define LIDAR_CMD_PRESET_PD         0x26u
#define LIDAR_CMD_TEMPERATURE_TDC   0x27u
#define LIDAR_CMD_LIDAR_CMD         0x30u
#define LIDAR_CMD_LIDAR_ACTION      0x31u
#define LIDAR_CMD_NET_IP            0x40u
#define LIDAR_CMD_NET_DATA_PORT     0x41u
#define LIDAR_CMD_FLASH_SPI         0x45u
#define LIDAR_CMD_NET_MAC           0x46u
#define LIDAR_CMD_FPGA_SET_ANG_RES  0x47u
#define LIDAR_CMD_NET_CMD_PORT      0x4au
#define LIDAR_CMD_COMP_SHIFTS       0x4cu
#define LIDAR_CMD_SHOTS_AT_EDGES    0x4eu
#define LIDAR_CMD_ENCODER_SHIFT     0x50u
#define LIDAR_CMD_PROP_DELAY        0x51u
#define LIDAR_CMD_VIEW_SECTOR       0x53u
#define LIDAR_CMD_ECHO_COUNT        0x54u
#define LIDAR_CMD_NET_CONFIG        0x60u
#define LIDAR_CMD_LIDAR_FIRMWARE    0x7eu

#define FW_CMD_CPU_PRG_BEGIN        55u
#define FW_CMD_CPU_PRG_DATA         56u
#define FW_CMD_CPU_PRG_END          57u
#define FW_CMD_JMP_BOOT             61u

#define FW_STATUS_NONE              0u
#define FW_STATUS_WRONG_CONFIG      1u
#define FW_STATUS_DEVICE_BUSY       2u
#define FW_STATUS_CHECKSUM_ERROR    3u
#define FW_STATUS_REJECTED          4u
#define FW_STATUS_UNKNOWN_COMMAND   6u
#define FW_STATUS_ERASE_PERCENT     9u

#define FW_HEADER_SIZE              256u
#define FW_FPGA_MAGIC               0xdeadbeefu
#define FW_FLASH_SECTOR_SIZE        4096u
#define FW_FLASH_BLOCK_SIZE         65536u
#define FW_FLASH_PAGE_SIZE          256u
#define FW_FLASH_MAX_SIZE           (16u * 1024u * 1024u)

#define SPIBB_MOSI                  0x00000001u
#define SPIBB_SCK                   0x00000002u
#define SPIBB_CSN                   0x00000004u
#define SPIBB_EN                    0x00000008u
#define SPIBB_MISO                  0x00000040u

#define SIM_FLAG_DISCOVERY          0x01u
#define SIM_FLAG_CONTROL            0x02u
#define SIM_FLAG_FWLOADER           0x04u
#define SIM_FLAGS_DONE              (SIM_FLAG_DISCOVERY | SIM_FLAG_CONTROL | SIM_FLAG_FWLOADER)

struct DARKETH {
    unsigned status;
    unsigned rx_len;
    unsigned rx_data;
    unsigned rx_ctrl;
    unsigned tx_status;
    unsigned tx_len;
    unsigned tx_data;
    unsigned tx_ctrl;
    unsigned cfg_mac_lo;
    unsigned cfg_mac_hi;
    unsigned cfg_flags;
};

struct LIDARSIM_CONFIG {
    unsigned char mac[6];
    unsigned char ip[4];
    unsigned char netmask[4];
    unsigned char gateway[4];
    unsigned char remote_ip[4];
    unsigned data_port;
    unsigned data_remote_port;
    unsigned cmd_port;
    unsigned cmd_remote_port;
    unsigned discovery_port;
};

struct CONTROL_CONN {
    struct tcp_pcb *pcb;
    unsigned len;
    unsigned char buf[LIDARSIM_CONTROL_BUF_MAX];
};

struct FIRMWARE_CONN {
    struct tcp_pcb *pcb;
    unsigned len;
    unsigned char buf[LIDARSIM_FIRMWARE_BUF_MAX];
};

struct FIRMWARE_SESSION {
    unsigned active;
    unsigned mock_flash;
    unsigned stream_offset;
    unsigned mcu_len;
    unsigned fpga_len;
    unsigned fpga_crc_expected;
    unsigned fpga_start;
    unsigned fpga_end;
    unsigned bytes_written;
    unsigned crc_state;
    unsigned flash_id0;
    unsigned flash_id1;
    unsigned flash_id2;
};

static volatile struct DARKETH *eth = (volatile struct DARKETH *)DARKETH_BASE;
static struct netif fpga_netif;
static struct udp_pcb *udp_discovery_listener;
static struct udp_pcb *udp_command_listener;
static struct tcp_pcb *tcp_data_listener;
static struct tcp_pcb *tcp_command_listener;
static struct tcp_pcb *tcp_firmware_listener;
static struct tcp_pcb *tcp_data_client;
static struct CONTROL_CONN tcp_control;
static struct FIRMWARE_CONN tcp_firmware;
static struct FIRMWARE_SESSION firmware_session;
static unsigned netif_configured;
static unsigned network_reconfig_pending;
static char uart_line[LIDARSIM_UART_LINE_MAX];
static unsigned uart_line_len;
static unsigned msop_frame_num;
static unsigned last_msop_ms;
static unsigned msop_tx_head;
static unsigned msop_tx_tail;
static unsigned msop_tx_inflight;
static unsigned msop_nocopy_unacked_bytes[LIDARSIM_MSOP_TX_BUFFERS];
static unsigned target_speed_bits = 0x41a00000u;
static unsigned voltage_ld = 27u;
static unsigned voltage_pd = 150u;
static unsigned preset_pd = 3u;
static unsigned heater_mode;
static unsigned fpga_angle_res = 500u;
static unsigned view_sector_start = 25000u;
static unsigned view_sector_end = 167000u;
static unsigned sim_progress_flags;
static unsigned last_rx_peer_valid;
static unsigned char last_rx_peer_mac[6];
static unsigned char last_rx_peer_ip[4];

static struct LIDARSIM_CONFIG runtime_config = {
    .mac = {0x02, 0x20, 0x20, 0x20, 0x20, 0x01},
    .ip = {192, 168, 2, 240},
    .netmask = {255, 255, 255, 0},
    .gateway = {192, 168, 2, 1},
    .remote_ip = {192, 168, 2, 146},
    .data_port = LIDARSIM_DATA_PORT,
    .data_remote_port = LIDARSIM_DATA_PORT,
    .cmd_port = LIDARSIM_CMD_PORT,
    .cmd_remote_port = LIDARSIM_CMD_PORT,
    .discovery_port = LIDARSIM_DISCOVERY_PORT,
};

u32_t sys_now(void)
{
    return io->timeus / 1000u;
}

static void write_le16(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
}

static void write_be16(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)((value >> 8) & 0xffu);
    dst[1] = (unsigned char)(value & 0xffu);
}

static void write_le24(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
    dst[2] = (unsigned char)((value >> 16) & 0xffu);
}

static void write_msop_distance(unsigned char *dst, unsigned value)
{
    if (LIDARSIM_MSOP_DISTANCE_BYTES == 3u) {
        write_le24(dst, value);
    } else {
        write_le16(dst, value);
    }
}

static void write_le32(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
    dst[2] = (unsigned char)((value >> 16) & 0xffu);
    dst[3] = (unsigned char)((value >> 24) & 0xffu);
}

static unsigned read_le16(const unsigned char *src)
{
    return (unsigned)src[0] | ((unsigned)src[1] << 8);
}

static unsigned read_le24(const unsigned char *src)
{
    return (unsigned)src[0] | ((unsigned)src[1] << 8) |
           ((unsigned)src[2] << 16);
}

static unsigned read_le32(const unsigned char *src)
{
    return (unsigned)src[0] | ((unsigned)src[1] << 8) |
           ((unsigned)src[2] << 16) | ((unsigned)src[3] << 24);
}

static unsigned mac_matches(const unsigned char *mac)
{
    unsigned all_ff = 1;
    unsigned all_zero = 1;

    for (unsigned i = 0; i < 6; i++) {
        if (mac[i] != runtime_config.mac[i]) {
            all_zero = 0;
        }
        if (mac[i] != 0xffu) {
            all_ff = 0;
        }
    }

    return all_ff || all_zero;
}

static void ip4_from_config(ip4_addr_t *addr, const unsigned char octets[4])
{
    IP4_ADDR(addr, octets[0], octets[1], octets[2], octets[3]);
}

static void copy_bytes(unsigned char *dst, const unsigned char *src, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        dst[i] = src[i];
    }
}

static void print_mac(const unsigned char *mac)
{
    for (unsigned i = 0; i < 6; i++) {
        printf("%x", mac[i]);
    }
}

static void print_hex_nibble(unsigned value)
{
    value &= 0x0fu;
    putchar((char)(value < 10u ? ('0' + value) : ('a' + value - 10u)));
}

static void print_hex_byte(unsigned value)
{
    print_hex_nibble(value >> 4);
    print_hex_nibble(value);
}

static void print_config(void)
{
    printf("lidarsim cfg mac=");
    print_mac(runtime_config.mac);
    printf(" ip=%d.%d.%d.%d data=%d cmd=%d discovery=%d model="
           LIDARSIM_MODEL " fw=" LIDARSIM_FIRMWARE "\n",
           runtime_config.ip[0], runtime_config.ip[1],
           runtime_config.ip[2], runtime_config.ip[3],
           runtime_config.data_port, runtime_config.cmd_port,
           runtime_config.discovery_port);
}

static void darketh_apply_filter_config(void)
{
    unsigned mac_hi = ((unsigned)runtime_config.mac[0] << 8) |
                      ((unsigned)runtime_config.mac[1]);
    unsigned mac_lo = ((unsigned)runtime_config.mac[2] << 24) |
                      ((unsigned)runtime_config.mac[3] << 16) |
                      ((unsigned)runtime_config.mac[4] << 8) |
                      ((unsigned)runtime_config.mac[5]);

    eth->cfg_mac_lo = mac_lo;
    eth->cfg_mac_hi = mac_hi;
    eth->cfg_flags = ETH_CFG_MAC_FILTER_ENABLE |
                     ETH_CFG_ACCEPT_BROADCAST |
                     ETH_CFG_ACCEPT_MULTICAST;
}

static void capture_rx_ipv4_peer(const unsigned char *hdr, unsigned len)
{
    last_rx_peer_valid = 0;

    if (!hdr || len < 34u) {
        return;
    }
    if (hdr[12] != 0x08u || hdr[13] != 0x00u) {
        return;
    }
    if ((hdr[14] & 0xf0u) != 0x40u || (hdr[14] & 0x0fu) != 5u) {
        return;
    }
    if (hdr[23] != 17u) {
        return;
    }

    copy_bytes(last_rx_peer_mac, hdr + 6, 6);
    copy_bytes(last_rx_peer_ip, hdr + 26, 4);
    last_rx_peer_valid = 1;
}

static err_t darketh_linkoutput(struct netif *netif, struct pbuf *p)
{
    unsigned timeout = 1000000;

    (void)netif;

    if (p->tot_len > LIDARSIM_MAX_FRAME) {
        printf("lidarsim tx too big=%d\n", p->tot_len);
        return ERR_BUF;
    }

    eth->tx_ctrl = ETH_TX_CTRL_CLEAR_FLAGS | ETH_TX_CTRL_ABORT;
    eth->tx_len = p->tot_len;

    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *src = (unsigned char *)q->payload;
        for (u16_t i = 0; i < q->len; i++) {
            eth->tx_data = src[i];
        }
    }

    unsigned staged = eth->tx_status;
    if ((staged & (ETH_TX_STATUS_READY | ETH_TX_STATUS_WRITTEN)) !=
        (ETH_TX_STATUS_READY | ETH_TX_STATUS_WRITTEN)) {
        printf("lidarsim tx stage err=%x\n", staged);
        return ERR_IF;
    }

    eth->tx_ctrl = ETH_TX_CTRL_START;
    while (!(eth->tx_status & ETH_TX_STATUS_DONE) && timeout) {
        timeout--;
    }

    unsigned done = eth->tx_status;
    if (!timeout || !(done & ETH_TX_STATUS_DONE) ||
        (done & (ETH_TX_STATUS_BUSY | ETH_TX_STATUS_OVERFLOW))) {
        printf("lidarsim tx fail status=%x\n", done);
        return ERR_IF;
    }

    eth->tx_ctrl = ETH_TX_CTRL_CLEAR_FLAGS;
    return ERR_OK;
}

static err_t send_raw_frame_bytes(const unsigned char *frame, unsigned len)
{
    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_RAM);
    if (!p) {
        return ERR_MEM;
    }

    copy_bytes((unsigned char *)p->payload, frame, len);
    err_t err = darketh_linkoutput(&fpga_netif, p);
    pbuf_free(p);
    return err;
}

static err_t darketh_netif_init(struct netif *netif)
{
    netif->name[0] = 'd';
    netif->name[1] = 'e';
    netif->output = etharp_output;
    netif->linkoutput = darketh_linkoutput;
    netif->hwaddr_len = ETH_HWADDR_LEN;
    for (unsigned i = 0; i < 6; i++) {
        netif->hwaddr[i] = runtime_config.mac[i];
    }
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
#ifdef NETIF_FLAG_ETHERNET
    netif->flags |= NETIF_FLAG_ETHERNET;
#endif
    return ERR_OK;
}

static void apply_netif_config(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;

    for (unsigned i = 0; i < 6; i++) {
        fpga_netif.hwaddr[i] = runtime_config.mac[i];
    }

    ip4_from_config(&ipaddr, runtime_config.ip);
    ip4_from_config(&netmask, runtime_config.netmask);
    ip4_from_config(&gw, runtime_config.gateway);
    etharp_cleanup_netif(&fpga_netif);
    ip4_addr_set(ip_2_ip4(&fpga_netif.ip_addr), &ipaddr);
    IP_SET_TYPE_VAL(fpga_netif.ip_addr, IPADDR_TYPE_V4);
    ip4_addr_set(ip_2_ip4(&fpga_netif.netmask), &netmask);
    IP_SET_TYPE_VAL(fpga_netif.netmask, IPADDR_TYPE_V4);
    ip4_addr_set(ip_2_ip4(&fpga_netif.gw), &gw);
    IP_SET_TYPE_VAL(fpga_netif.gw, IPADDR_TYPE_V4);
    darketh_apply_filter_config();
}

static void apply_net_config_payload(const unsigned char *payload, unsigned len)
{
    unsigned data_port;
    unsigned data_remote_port;
    unsigned cmd_port;
    unsigned cmd_remote_port;
    unsigned discovery_port;

    if (!payload || len < 24u || !mac_matches(payload)) {
        return;
    }

    copy_bytes(runtime_config.ip, payload + 6, 4);
    copy_bytes(runtime_config.remote_ip, payload + 10, 4);

    if (len >= 26u) {
        data_port = read_le16(payload + 14);
        data_remote_port = read_le16(payload + 16);
        cmd_port = read_le16(payload + 18);
        cmd_remote_port = read_le16(payload + 20);
        discovery_port = read_le16(payload + 24);
    } else {
        data_port = read_le16(payload + 15);
        data_remote_port = read_le16(payload + 17);
        cmd_port = read_le16(payload + 20);
        cmd_remote_port = read_le16(payload + 22);
        discovery_port = runtime_config.discovery_port;
    }

    data_port = data_port ? data_port : LIDARSIM_DATA_PORT;
    data_remote_port = data_remote_port ? data_remote_port : LIDARSIM_DATA_PORT;
    cmd_port = cmd_port ? cmd_port : LIDARSIM_CMD_PORT;
    cmd_remote_port = cmd_remote_port ? cmd_remote_port : LIDARSIM_CMD_PORT;
    discovery_port = discovery_port ? discovery_port : LIDARSIM_DISCOVERY_PORT;

    runtime_config.data_port = data_port;
    runtime_config.data_remote_port = data_remote_port;
    runtime_config.cmd_port = cmd_port;
    runtime_config.cmd_remote_port = cmd_remote_port;
    runtime_config.discovery_port = discovery_port;
    network_reconfig_pending = 1;
    printf("lidarsim net_config queued\n");
    print_config();
}

static void fill_fixed_payload(unsigned cmd, unsigned rw,
                               const unsigned char *request,
                               unsigned char payload[16])
{
    for (unsigned i = 0; i < 16; i++) {
        payload[i] = 0;
    }

    if (rw == LIDAR_PROTO_WRITE && request) {
        copy_bytes(payload, request, 16);
        if (cmd == LIDAR_CMD_MOTOR_TARGET) {
            target_speed_bits = (unsigned)request[0] |
                                ((unsigned)request[1] << 8) |
                                ((unsigned)request[2] << 16) |
                                ((unsigned)request[3] << 24);
        } else if (cmd == LIDAR_CMD_VOLTAGE_TARGET_LD) {
            voltage_ld = read_le16(request);
        } else if (cmd == LIDAR_CMD_VOLTAGE_TARGET_PD) {
            voltage_pd = read_le16(request);
        } else if (cmd == LIDAR_CMD_PRESET_PD) {
            preset_pd = request[0];
        } else if (cmd == LIDAR_CMD_LIDAR_CMD) {
            heater_mode = request[0];
        } else if (cmd == LIDAR_CMD_FPGA_SET_ANG_RES) {
            fpga_angle_res = read_le16(request);
        } else if (cmd == LIDAR_CMD_VIEW_SECTOR) {
            view_sector_start = read_le24(request + 1);
            view_sector_end = read_le24(request + 4);
        } else if (cmd == LIDAR_CMD_NET_MAC) {
            copy_bytes(runtime_config.mac, request, 6);
            if (netif_configured) {
                apply_netif_config();
            }
        }
        return;
    }

    switch (cmd) {
    case LIDAR_CMD_FULL_STATUS:
    case LIDAR_CMD_STATUS:
        break;
    case LIDAR_CMD_VOLTAGE:
        write_le32(payload + 0, 0x40533333u);
        write_le32(payload + 4, 0x40a00000u);
        write_le32(payload + 8, 0x41400000u);
        break;
    case LIDAR_CMD_MOTOR_TARGET:
    case LIDAR_CMD_MOTOR_CURRENT:
        write_le32(payload, target_speed_bits);
        break;
    case LIDAR_CMD_VOLTAGE_TARGET_LD:
    case LIDAR_CMD_VOLTAGE_CURRENT_LD:
        write_le16(payload, voltage_ld);
        break;
    case LIDAR_CMD_VOLTAGE_TARGET_PD:
    case LIDAR_CMD_VOLTAGE_CURRENT_PD:
        write_le16(payload, voltage_pd);
        break;
    case LIDAR_CMD_TEMPERATURE_LD:
        payload[0] = (unsigned char)(29u + ((sys_now() / 1000u) & 3u));
        break;
    case LIDAR_CMD_TEMPERATURE_PD:
        payload[0] = (unsigned char)(26u + ((sys_now() / 1400u) & 3u));
        break;
    case LIDAR_CMD_PRESET_PD:
        payload[0] = (unsigned char)preset_pd;
        break;
    case LIDAR_CMD_TEMPERATURE_TDC:
        payload[0] = (unsigned char)(31u + ((sys_now() / 1700u) & 3u));
        break;
    case LIDAR_CMD_LIDAR_CMD:
        payload[0] = (unsigned char)heater_mode;
        break;
    case LIDAR_CMD_LIDAR_ACTION:
        payload[0] = 0;
        break;
    case LIDAR_CMD_NET_IP:
        copy_bytes(payload, runtime_config.ip, 4);
        copy_bytes(payload + 4, runtime_config.remote_ip, 4);
        break;
    case LIDAR_CMD_NET_DATA_PORT:
        payload[0] = 0;
        write_le16(payload + 1, runtime_config.data_port);
        write_le16(payload + 3, runtime_config.data_remote_port);
        break;
    case LIDAR_CMD_NET_CMD_PORT:
        payload[0] = 0;
        write_le16(payload + 1, runtime_config.cmd_port);
        write_le16(payload + 3, runtime_config.cmd_remote_port);
        break;
    case LIDAR_CMD_NET_MAC:
        copy_bytes(payload, runtime_config.mac, 6);
        copy_bytes(payload + 6, runtime_config.mac, 6);
        break;
    case LIDAR_CMD_FPGA_SET_ANG_RES:
        write_le16(payload, fpga_angle_res);
        break;
    case LIDAR_CMD_COMP_SHIFTS:
    case LIDAR_CMD_SHOTS_AT_EDGES:
    case LIDAR_CMD_ENCODER_SHIFT:
    case LIDAR_CMD_PROP_DELAY:
        break;
    case LIDAR_CMD_VIEW_SECTOR:
        payload[0] = 1;
        write_le24(payload + 1, view_sector_start);
        write_le24(payload + 4, view_sector_end);
        break;
    case LIDAR_CMD_ECHO_COUNT:
        payload[0] = LIDARSIM_MSOP_ECHO_COUNT;
        payload[1] = LIDARSIM_MSOP_DISTANCE_BYTES;
        payload[2] = LIDARSIM_MSOP_ECHO_MODE;
        break;
    default:
        break;
    }
}

static void fill_flash_spi_fixed_payload(const unsigned char *request,
                                         unsigned char payload[16]);
static unsigned build_flash_spi_varlen_payload(const unsigned char *payload,
                                               unsigned payload_len,
                                               unsigned char *out,
                                               unsigned out_capacity);

static unsigned build_control_reply(unsigned proto_type,
                                    unsigned pkt_cnt,
                                    unsigned cmd,
                                    unsigned rw,
                                    const unsigned char *payload,
                                    unsigned payload_len,
                                    unsigned char *out)
{
    out[0] = 0xff;
    out[1] = 0xfe;
    out[2] = LIDAR_PROTO_VERSION;
    out[3] = (unsigned char)proto_type;
    out[4] = (unsigned char)pkt_cnt;
    out[5] = (unsigned char)cmd;
    out[6] = (unsigned char)rw;

    if (proto_type == LIDAR_PROTO_VARLEN) {
        unsigned len = payload_len;
        if (cmd == LIDAR_CMD_FLASH_SPI) {
            len = build_flash_spi_varlen_payload(payload, payload_len,
                                                 out + 9,
                                                 LIDARSIM_CONTROL_REPLY_MAX - 11u);
            write_le16(out + 7, len);
        } else if (cmd == LIDAR_CMD_LIDAR_FIRMWARE && rw == LIDAR_PROTO_READ) {
            static const char fw[] = LIDARSIM_FIRMWARE;
            len = sizeof(fw) - 1u;
            write_le16(out + 7, len);
            for (unsigned i = 0; i < len; i++) {
                out[9 + i] = (unsigned char)fw[i];
            }
        } else {
            if (len > (LIDARSIM_CONTROL_REPLY_MAX - 11u)) {
                len = LIDARSIM_CONTROL_REPLY_MAX - 11u;
            }
            write_le16(out + 7, len);
            for (unsigned i = 0; i < len; i++) {
                out[9 + i] = payload ? payload[i] : 0;
            }
        }
        out[9 + len] = 0xff;
        out[10 + len] = 0x9b;
        return 11u + len;
    }

    if (cmd == LIDAR_CMD_FLASH_SPI) {
        fill_flash_spi_fixed_payload(payload, out + 7);
    } else {
        fill_fixed_payload(cmd, rw, payload, out + 7);
    }
    out[23] = 0xff;
    out[24] = 0x9b;
    return 25u;
}

static err_t send_udp_bytes(struct udp_pcb *pcb, const ip_addr_t *addr,
                            u16_t port, const unsigned char *data,
                            unsigned len)
{
    struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (!reply) {
        return ERR_MEM;
    }

    copy_bytes((unsigned char *)reply->payload, data, len);
    err_t err = udp_sendto(pcb, reply, addr, port);
    pbuf_free(reply);
    return err;
}

static err_t send_udp_broadcast_bytes(struct udp_pcb *pcb, u16_t port,
                                      const unsigned char *data,
                                      unsigned len)
{
    return send_udp_bytes(pcb, IP_ADDR_BROADCAST, port, data, len);
}

static unsigned ipv4_header_checksum(const unsigned char *hdr, unsigned len)
{
    unsigned sum = 0;

    for (unsigned i = 0; i < len; i += 2u) {
        unsigned word = ((unsigned)hdr[i] << 8);
        if ((i + 1u) < len) {
            word |= hdr[i + 1u];
        }
        sum += word;
        while (sum >> 16) {
            sum = (sum & 0xffffu) + (sum >> 16);
        }
    }

    return (~sum) & 0xffffu;
}

static err_t send_udp_unicast_to_last_peer(u16_t src_port, u16_t dst_port,
                                           const unsigned char *data,
                                           unsigned len)
{
    unsigned char frame[14u + 20u + 8u + LIDAR_DISCOVERY_RESPONSE_SIZE];
    unsigned ip_len = 20u + 8u + len;
    unsigned frame_len = 14u + ip_len;

    if (!last_rx_peer_valid || len > LIDAR_DISCOVERY_RESPONSE_SIZE) {
        return ERR_BUF;
    }

    copy_bytes(frame, last_rx_peer_mac, 6);
    copy_bytes(frame + 6, runtime_config.mac, 6);
    frame[12] = 0x08u;
    frame[13] = 0x00u;

    frame[14] = 0x45u;
    frame[15] = 0x00u;
    write_be16(frame + 16, ip_len);
    write_be16(frame + 18, 0);
    write_be16(frame + 20, 0);
    frame[22] = 255u;
    frame[23] = 17u;
    write_be16(frame + 24, 0);
    copy_bytes(frame + 26, runtime_config.ip, 4);
    copy_bytes(frame + 30, last_rx_peer_ip, 4);
    write_be16(frame + 24, ipv4_header_checksum(frame + 14, 20u));

    write_be16(frame + 34, src_port);
    write_be16(frame + 36, dst_port);
    write_be16(frame + 38, 8u + len);
    write_be16(frame + 40, 0);
    copy_bytes(frame + 42, data, len);

    return send_raw_frame_bytes(frame, frame_len);
}

static err_t send_tcp_bytes(struct tcp_pcb *pcb, const unsigned char *data,
                            unsigned len)
{
    if (!pcb || tcp_sndbuf(pcb) < len) {
        return ERR_MEM;
    }

    err_t err = tcp_write(pcb, data, (u16_t)len, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(pcb);
    }
    return err;
}

static void maybe_report_sim_ok(void)
{
    if ((sim_progress_flags & SIM_FLAGS_DONE) == SIM_FLAGS_DONE) {
        sim_progress_flags = 0xffu;
        printf("lidarsim ok\n>");
    }
}

static unsigned crc32_update_state(unsigned state, const unsigned char *data,
                                   unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        state ^= data[i];
        for (unsigned bit = 0; bit < 8u; bit++) {
            if (state & 1u) {
                state = (state >> 1) ^ 0xedb88320u;
            } else {
                state >>= 1;
            }
        }
    }
    return state;
}

static unsigned crc32_update_byte(unsigned state, unsigned char value)
{
    state ^= value;
    for (unsigned bit = 0; bit < 8u; bit++) {
        if (state & 1u) {
            state = (state >> 1) ^ 0xedb88320u;
        } else {
            state >>= 1;
        }
    }
    return state;
}

static unsigned checksum16(const unsigned char *data, unsigned len)
{
    unsigned sum = 0;
    for (unsigned i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xffffu;
}

static unsigned build_firmware_frame(unsigned pkt_num,
                                     const unsigned char *payload,
                                     unsigned payload_len,
                                     unsigned char *out)
{
    unsigned pos = 0;
    out[pos++] = 0x12u;
    out[pos++] = 0x34u;
    write_le16(out + pos, payload_len); pos += 2u;
    write_le32(out + pos, pkt_num); pos += 4u;
    for (unsigned i = 0; i < payload_len; i++) {
        out[pos++] = payload[i];
    }
    out[pos++] = 0x55u;
    out[pos++] = 0xaau;
    write_le16(out + pos, checksum16(out, pos));
    pos += 2u;
    return pos;
}

static void firmware_send_status(struct FIRMWARE_CONN *conn, unsigned pkt_num,
                                 unsigned cmd, unsigned status,
                                 unsigned percent, unsigned with_percent)
{
    unsigned char payload[3];
    unsigned char frame[16];

    payload[0] = (unsigned char)cmd;
    payload[1] = (unsigned char)status;
    if (with_percent) {
        payload[2] = (unsigned char)percent;
    }

    unsigned frame_len = build_firmware_frame(pkt_num, payload,
                                              with_percent ? 3u : 2u,
                                              frame);
    if (conn && conn->pcb) {
        (void)send_tcp_bytes(conn->pcb, frame, frame_len);
    }
}

static void spibb_delay(void)
{
    static volatile unsigned sink;
    for (unsigned i = 0; i < 3u; i++) {
        sink++;
    }
}

static void spibb_write(unsigned value)
{
    io->oport = value;
    spibb_delay();
}

static void spibb_idle(void)
{
    spibb_write(SPIBB_EN | SPIBB_CSN | SPIBB_SCK | SPIBB_MOSI);
}

static void spibb_select(void)
{
    spibb_idle();
    spibb_write(SPIBB_EN | SPIBB_SCK | SPIBB_MOSI);
}

static void spibb_deselect(void)
{
    spibb_idle();
}

static unsigned char spibb_transfer_byte(unsigned char value)
{
    unsigned char rx = 0;

    for (int bit = 7; bit >= 0; bit--) {
        unsigned mosi = (value & (1u << bit)) ? SPIBB_MOSI : 0u;
        spibb_write(SPIBB_EN | mosi);
        spibb_write(SPIBB_EN | SPIBB_SCK | mosi);
        rx = (unsigned char)((rx << 1) |
             ((io->iport & SPIBB_MISO) ? 1u : 0u));
    }

    return rx;
}

static void flash_addr24(unsigned addr)
{
    spibb_transfer_byte((unsigned char)((addr >> 16) & 0xffu));
    spibb_transfer_byte((unsigned char)((addr >> 8) & 0xffu));
    spibb_transfer_byte((unsigned char)(addr & 0xffu));
}

static unsigned flash_read_jedec_id(unsigned char id[3])
{
    if (firmware_session.mock_flash) {
        id[0] = 0xefu;
        id[1] = 0x40u;
        id[2] = 0x18u;
        return 1;
    }

    spibb_select();
    spibb_transfer_byte(0x9fu);
    id[0] = spibb_transfer_byte(0x00u);
    id[1] = spibb_transfer_byte(0x00u);
    id[2] = spibb_transfer_byte(0x00u);
    spibb_deselect();

    if ((id[0] == 0x00u && id[1] == 0x00u && id[2] == 0x00u) ||
        (id[0] == 0xffu && id[1] == 0xffu && id[2] == 0xffu)) {
        return 0;
    }
    return 1;
}

static unsigned flash_read_status(void)
{
    if (firmware_session.mock_flash) {
        return 0;
    }

    spibb_select();
    spibb_transfer_byte(0x05u);
    unsigned status = spibb_transfer_byte(0x00u);
    spibb_deselect();
    return status;
}

static unsigned flash_wait_ready(unsigned timeout_ms)
{
    unsigned start = sys_now();
    do {
        if ((flash_read_status() & 0x01u) == 0u) {
            return 1;
        }
    } while ((sys_now() - start) < timeout_ms);
    return 0;
}

static void flash_write_enable(void)
{
    if (firmware_session.mock_flash) {
        return;
    }

    spibb_select();
    spibb_transfer_byte(0x06u);
    spibb_deselect();
}

static unsigned flash_erase_cmd(unsigned cmd, unsigned addr,
                                unsigned timeout_ms)
{
    if (firmware_session.mock_flash) {
        return 1;
    }

    flash_write_enable();
    spibb_select();
    spibb_transfer_byte((unsigned char)cmd);
    flash_addr24(addr);
    spibb_deselect();
    return flash_wait_ready(timeout_ms);
}

static unsigned flash_page_program(unsigned addr, const unsigned char *data,
                                   unsigned len)
{
    if (len == 0u) {
        return 1;
    }
    if (firmware_session.mock_flash) {
        return 1;
    }

    flash_write_enable();
    spibb_select();
    spibb_transfer_byte(0x02u);
    flash_addr24(addr);
    for (unsigned i = 0; i < len; i++) {
        spibb_transfer_byte(data[i]);
    }
    spibb_deselect();
    return flash_wait_ready(500u);
}

static unsigned firmware_flash_write(unsigned addr, const unsigned char *data,
                                     unsigned len)
{
    while (len) {
        unsigned page_room = FW_FLASH_PAGE_SIZE - (addr & (FW_FLASH_PAGE_SIZE - 1u));
        unsigned chunk = (len < page_room) ? len : page_room;
        if (!flash_page_program(addr, data, chunk)) {
            return 0;
        }
        addr += chunk;
        data += chunk;
        len -= chunk;
    }
    return 1;
}

static unsigned flash_read_bytes(unsigned addr, unsigned char *data,
                                 unsigned len)
{
    if (len == 0u) {
        return 1;
    }
    if (!data || addr >= FW_FLASH_MAX_SIZE ||
        len > (FW_FLASH_MAX_SIZE - addr)) {
        return 0;
    }
    if (firmware_session.mock_flash) {
        for (unsigned i = 0; i < len; i++) {
            data[i] = 0xffu;
        }
        return 1;
    }
    if (!flash_wait_ready(5000u)) {
        return 0;
    }

    spibb_select();
    spibb_transfer_byte(0x03u);
    flash_addr24(addr);
    for (unsigned i = 0; i < len; i++) {
        data[i] = spibb_transfer_byte(0x00u);
    }
    spibb_deselect();
    return 1;
}

static unsigned flash_erase_sector(unsigned addr)
{
    if (addr >= FW_FLASH_MAX_SIZE) {
        return 0;
    }
    return flash_erase_cmd(0x20u, addr & ~(FW_FLASH_SECTOR_SIZE - 1u), 1000u);
}

static void fill_flash_spi_fixed_payload(const unsigned char *request,
                                         unsigned char payload[16])
{
    unsigned op = request ? request[0] : 0xffu;
    unsigned addr = request ? read_le24(request + 2) : 0u;
    unsigned char value = request ? request[1] : 0u;
    unsigned ok = 0;

    for (unsigned i = 0; i < 16u; i++) {
        payload[i] = 0;
    }

    switch (op) {
    case 0u:
        ok = request && firmware_flash_write(addr, &value, 1u);
        payload[0] = 0u;
        payload[1] = ok ? value : 105u;
        write_le24(payload + 2, addr);
        break;
    case 1u:
        ok = request && flash_read_bytes(addr, &value, 1u);
        payload[0] = 1u;
        payload[1] = ok ? value : 105u;
        write_le24(payload + 2, addr);
        break;
    case 2u:
        ok = request && flash_erase_sector(addr);
        payload[0] = 2u;
        payload[1] = ok ? 0u : 105u;
        write_le24(payload + 2, addr);
        break;
    case 3u:
        payload[0] = (unsigned char)flash_read_status();
        write_le24(payload + 2, addr);
        break;
    default:
        payload[0] = (unsigned char)op;
        payload[1] = 105u;
        write_le24(payload + 2, addr);
        break;
    }
}

static unsigned build_flash_spi_varlen_payload(const unsigned char *payload,
                                               unsigned payload_len,
                                               unsigned char *out,
                                               unsigned out_capacity)
{
    if (!payload || payload_len < 6u || !out || out_capacity < 6u) {
        return 0;
    }

    unsigned op = payload[0];
    unsigned addr = read_le24(payload + 1);
    unsigned req_len = read_le16(payload + 4);
    unsigned max_data = out_capacity - 6u;
    unsigned data_len = req_len;

    if (data_len > max_data) {
        data_len = max_data;
    }
    if (addr >= FW_FLASH_MAX_SIZE) {
        data_len = 0;
    } else if (data_len > (FW_FLASH_MAX_SIZE - addr)) {
        data_len = FW_FLASH_MAX_SIZE - addr;
    }

    out[0] = (unsigned char)op;
    write_le24(out + 1, addr);
    write_le16(out + 4, data_len);

    if (op == 0u) {
        unsigned available = payload_len - 6u;
        if (data_len > available) {
            data_len = available;
        }
        if (firmware_flash_write(addr, payload + 6, data_len)) {
            for (unsigned i = 0; i < data_len; i++) {
                out[6 + i] = payload[6 + i];
            }
        } else {
            data_len = 0;
        }
        write_le16(out + 4, data_len);
        return 6u + data_len;
    }

    if (op == 1u) {
        if (!flash_read_bytes(addr, out + 6, data_len)) {
            data_len = 0;
            write_le16(out + 4, 0u);
        }
        return 6u + data_len;
    }

    write_le16(out + 4, 0u);
    return 6u;
}

static unsigned firmware_flash_crc32(unsigned size, unsigned *crc_out)
{
    unsigned state = 0xffffffffu;

    if (firmware_session.mock_flash) {
        *crc_out = firmware_session.fpga_crc_expected;
        return 1;
    }

    if (!flash_wait_ready(5000u)) {
        return 0;
    }

    spibb_select();
    spibb_transfer_byte(0x03u);
    flash_addr24(0);
    for (unsigned i = 0; i < size; i++) {
        state = crc32_update_byte(state, spibb_transfer_byte(0x00u));
    }
    spibb_deselect();

    *crc_out = state ^ 0xffffffffu;
    return 1;
}

static unsigned firmware_flash_erase(struct FIRMWARE_CONN *conn,
                                     unsigned pkt_num,
                                     unsigned size)
{
    unsigned erase_size = (size + FW_FLASH_SECTOR_SIZE - 1u) &
                          ~(FW_FLASH_SECTOR_SIZE - 1u);
    unsigned addr = 0;
    unsigned last_percent = 101u;

    if (erase_size == 0u || erase_size > FW_FLASH_MAX_SIZE) {
        return 0;
    }

    while (addr < erase_size) {
        unsigned remaining = erase_size - addr;
        unsigned step;
        unsigned ok;

        if ((addr & (FW_FLASH_BLOCK_SIZE - 1u)) == 0u &&
            remaining >= FW_FLASH_BLOCK_SIZE) {
            step = FW_FLASH_BLOCK_SIZE;
            ok = flash_erase_cmd(0xd8u, addr, 3000u);
        } else {
            step = FW_FLASH_SECTOR_SIZE;
            ok = flash_erase_cmd(0x20u, addr, 1000u);
        }

        if (!ok) {
            return 0;
        }

        addr += step;
        unsigned percent = (addr >= erase_size) ? 100u :
            ((addr * 100u) / erase_size);
        if (percent != last_percent) {
            firmware_send_status(conn, pkt_num, FW_CMD_CPU_PRG_BEGIN,
                                 FW_STATUS_ERASE_PERCENT, percent, 1);
            last_percent = percent;
        }
    }
    return 1;
}

static void firmware_session_clear(void)
{
    firmware_session.active = 0;
    firmware_session.stream_offset = 0;
    firmware_session.mcu_len = 0;
    firmware_session.fpga_len = 0;
    firmware_session.fpga_crc_expected = 0;
    firmware_session.fpga_start = 0;
    firmware_session.fpga_end = 0;
    firmware_session.bytes_written = 0;
    firmware_session.crc_state = 0xffffffffu;
}

static unsigned firmware_process_stream_data(const unsigned char *data,
                                             unsigned len)
{
    unsigned current = firmware_session.stream_offset;
    unsigned next = current + len;

    if (next < current) {
        return 0;
    }

    if (next > firmware_session.fpga_start && current < firmware_session.fpga_end) {
        unsigned write_start = current > firmware_session.fpga_start ?
            current : firmware_session.fpga_start;
        unsigned write_end = next < firmware_session.fpga_end ?
            next : firmware_session.fpga_end;
        unsigned data_offset = write_start - current;
        unsigned count = write_end - write_start;
        unsigned flash_addr = write_start - firmware_session.fpga_start;

        firmware_session.crc_state = crc32_update_state(
            firmware_session.crc_state, data + data_offset, count);

        if (!firmware_flash_write(flash_addr, data + data_offset, count)) {
            return 0;
        }
        firmware_session.bytes_written += count;
    }

    firmware_session.stream_offset = next;
    return 1;
}

static unsigned firmware_begin(struct FIRMWARE_CONN *conn, unsigned pkt_num,
                               const unsigned char *data, unsigned len)
{
    if (len < FW_HEADER_SIZE) {
        return FW_STATUS_WRONG_CONFIG;
    }

    firmware_session_clear();
    firmware_session.mock_flash = (io->board_id == 0u);
    firmware_session.mcu_len = read_le32(data + 0);
    firmware_session.fpga_len = read_le32(data + 8);
    firmware_session.fpga_crc_expected = read_le32(data + 12);
    unsigned magic = read_le32(data + 128);

    if (firmware_session.fpga_len == 0u ||
        firmware_session.fpga_len > FW_FLASH_MAX_SIZE ||
        (firmware_session.mcu_len & (FW_HEADER_SIZE - 1u)) != 0u ||
        (firmware_session.fpga_len & (FW_HEADER_SIZE - 1u)) != 0u ||
        magic != FW_FPGA_MAGIC) {
        return FW_STATUS_WRONG_CONFIG;
    }

    firmware_session.fpga_start = FW_HEADER_SIZE + firmware_session.mcu_len;
    firmware_session.fpga_end = firmware_session.fpga_start + firmware_session.fpga_len;
    if (firmware_session.fpga_end < firmware_session.fpga_start) {
        return FW_STATUS_WRONG_CONFIG;
    }

    unsigned char id[3];
    if (!flash_read_jedec_id(id)) {
        printf("fwloader flash id fail\n");
        return FW_STATUS_REJECTED;
    }

    firmware_session.flash_id0 = id[0];
    firmware_session.flash_id1 = id[1];
    firmware_session.flash_id2 = id[2];
    printf("fwloader begin mock=%d flash_id=%x%x%x fpga=%d crc=%x\n",
           firmware_session.mock_flash, id[0], id[1], id[2],
           firmware_session.fpga_len, firmware_session.fpga_crc_expected);

    if (!firmware_flash_erase(conn, pkt_num, firmware_session.fpga_len)) {
        printf("fwloader erase fail\n");
        return FW_STATUS_REJECTED;
    }

    firmware_session.active = 1;
    firmware_session.stream_offset = 0;
    if (!firmware_process_stream_data(data, len)) {
        firmware_session_clear();
        return FW_STATUS_REJECTED;
    }
    return FW_STATUS_NONE;
}

static unsigned firmware_end(void)
{
    if (!firmware_session.active) {
        return FW_STATUS_REJECTED;
    }

    unsigned actual_crc = firmware_session.crc_state ^ 0xffffffffu;
    unsigned flash_crc = 0;
    if (firmware_session.bytes_written != firmware_session.fpga_len) {
        printf("fwloader size mismatch got=%d exp=%d\n",
               firmware_session.bytes_written, firmware_session.fpga_len);
        firmware_session_clear();
        return FW_STATUS_WRONG_CONFIG;
    }
    if (actual_crc != firmware_session.fpga_crc_expected) {
        printf("fwloader crc mismatch got=%x exp=%x\n",
               actual_crc, firmware_session.fpga_crc_expected);
        firmware_session_clear();
        return FW_STATUS_CHECKSUM_ERROR;
    }
    if (!firmware_flash_crc32(firmware_session.fpga_len, &flash_crc)) {
        printf("fwloader verify read fail\n");
        firmware_session_clear();
        return FW_STATUS_REJECTED;
    }
    if (flash_crc != firmware_session.fpga_crc_expected) {
        printf("fwloader flash crc mismatch got=%x exp=%x\n",
               flash_crc, firmware_session.fpga_crc_expected);
        firmware_session_clear();
        return FW_STATUS_CHECKSUM_ERROR;
    }

    printf("fwloader end ok bytes=%d crc=%x flash_crc=%x\n",
           firmware_session.bytes_written, actual_crc, flash_crc);
    firmware_session_clear();
    return FW_STATUS_NONE;
}

static void firmware_process_payload(struct FIRMWARE_CONN *conn,
                                     unsigned pkt_num,
                                     const unsigned char *payload,
                                     unsigned payload_len)
{
    if (payload_len == 0u) {
        return;
    }

    unsigned cmd = payload[0];
    const unsigned char *data = payload + 1;
    unsigned data_len = payload_len - 1u;
    unsigned status = FW_STATUS_NONE;

    if (cmd == FW_CMD_JMP_BOOT) {
        firmware_session_clear();
        printf("fwloader boot\n");
    } else if (cmd == FW_CMD_CPU_PRG_BEGIN) {
        status = firmware_begin(conn, pkt_num, data, data_len);
    } else if (cmd == FW_CMD_CPU_PRG_DATA) {
        if (!firmware_session.active) {
            status = FW_STATUS_REJECTED;
        } else if (!firmware_process_stream_data(data, data_len)) {
            status = FW_STATUS_REJECTED;
        }
    } else if (cmd == FW_CMD_CPU_PRG_END) {
        status = firmware_end();
    } else {
        status = FW_STATUS_UNKNOWN_COMMAND;
    }

    firmware_send_status(conn, pkt_num, cmd, status, 0, 0);
}

static unsigned firmware_packet_size(const unsigned char *buf, unsigned len)
{
    if (len < 2u) {
        return 0;
    }
    if (buf[0] != 0x12u || buf[1] != 0x34u) {
        return 1u;
    }
    if (len < 8u) {
        return 0;
    }

    unsigned payload_len = read_le16(buf + 2);
    unsigned packet_len = 2u + 2u + 4u + payload_len + 2u + 2u;
    if (payload_len > (FW_FLASH_PAGE_SIZE + 1u) ||
        packet_len > LIDARSIM_FIRMWARE_BUF_MAX) {
        return 1u;
    }
    if (len < packet_len) {
        return 0;
    }
    if (buf[packet_len - 4u] != 0x55u ||
        buf[packet_len - 3u] != 0xaau) {
        return 1u;
    }

    unsigned expected = checksum16(buf, packet_len - 2u);
    unsigned actual = read_le16(buf + packet_len - 2u);
    return (expected == actual) ? packet_len : 1u;
}

static void parse_firmware_stream(struct FIRMWARE_CONN *conn)
{
    while (conn->len >= 2u) {
        if (conn->buf[0] != 0x12u || conn->buf[1] != 0x34u) {
            unsigned start = 1u;
            while (start + 1u < conn->len &&
                   !(conn->buf[start] == 0x12u &&
                     conn->buf[start + 1u] == 0x34u)) {
                start++;
            }
            memmove(conn->buf, conn->buf + start, conn->len - start);
            conn->len -= start;
            continue;
        }

        unsigned packet_len = firmware_packet_size(conn->buf, conn->len);
        if (packet_len == 0u) {
            return;
        }
        if (packet_len == 1u) {
            memmove(conn->buf, conn->buf + 1, conn->len - 1u);
            conn->len--;
            continue;
        }

        unsigned pkt_num = read_le32(conn->buf + 4);
        unsigned payload_len = read_le16(conn->buf + 2);
        firmware_process_payload(conn, pkt_num, conn->buf + 8,
                                 payload_len);

        memmove(conn->buf, conn->buf + packet_len, conn->len - packet_len);
        conn->len -= packet_len;
    }
}

static void firmware_selftest(void)
{
    if (io->board_id != 0u) {
        return;
    }

    unsigned char *buf = tcp_firmware.buf;

    unsigned crc_state = 0xffffffffu;
    for (unsigned i = 0; i < FW_FLASH_PAGE_SIZE; i++) {
        buf[i] = (unsigned char)i;
    }
    crc_state = crc32_update_state(crc_state, buf, FW_FLASH_PAGE_SIZE);
    for (unsigned i = 0; i < FW_FLASH_PAGE_SIZE; i++) {
        buf[i] = (unsigned char)(255u - i);
    }
    crc_state = crc32_update_state(crc_state, buf, FW_FLASH_PAGE_SIZE);
    unsigned crc = crc_state ^ 0xffffffffu;

    for (unsigned i = 0; i < FW_HEADER_SIZE; i++) {
        buf[i] = 0;
    }
    write_le32(buf + 8, FW_FLASH_PAGE_SIZE * 2u);
    write_le32(buf + 12, crc);
    write_le32(buf + 128, FW_FPGA_MAGIC);

    unsigned st = firmware_begin(0, 0, buf, FW_HEADER_SIZE);
    for (unsigned i = 0; i < FW_FLASH_PAGE_SIZE; i++) {
        buf[i] = (unsigned char)i;
    }
    unsigned ok0 = firmware_process_stream_data(buf, FW_FLASH_PAGE_SIZE);
    for (unsigned i = 0; i < FW_FLASH_PAGE_SIZE; i++) {
        buf[i] = (unsigned char)(255u - i);
    }
    unsigned ok1 = firmware_process_stream_data(buf, FW_FLASH_PAGE_SIZE);

    if (st == FW_STATUS_NONE && ok0 && ok1 &&
        firmware_end() == FW_STATUS_NONE) {
        printf("fwloader sim ok\n");
        if (sim_progress_flags != 0xffu) {
            sim_progress_flags |= SIM_FLAG_FWLOADER;
            maybe_report_sim_ok();
        }
        return;
    }

    printf("fwloader sim fail status=%d\n", st);
}

static void process_control_packet_tcp(struct tcp_pcb *pcb,
                                       const unsigned char *packet,
                                       unsigned packet_len)
{
    unsigned char reply[LIDARSIM_CONTROL_REPLY_MAX];
    unsigned proto_type = packet[3];
    unsigned payload_len = 16u;
    const unsigned char *payload = packet + 7;

    if (proto_type == LIDAR_PROTO_VARLEN) {
        payload_len = read_le16(packet + 7);
        payload = packet + 9;
    }

    unsigned reply_len = build_control_reply(proto_type, packet[4], packet[5],
                                             packet[6], payload, payload_len,
                                             reply);
    err_t err = send_tcp_bytes(pcb, reply, reply_len);
    printf("lidarsim tcp cmd=%x rw=%d reply=%d err=%d\n",
           packet[5], packet[6], reply_len, err);
    if (proto_type == LIDAR_PROTO_VARLEN &&
        packet[5] == LIDAR_CMD_NET_CONFIG &&
        packet[6] == LIDAR_PROTO_WRITE) {
        apply_net_config_payload(payload, payload_len);
    }
    (void)packet_len;
}

static void process_control_packet_udp(struct udp_pcb *pcb,
                                       const ip_addr_t *addr,
                                       u16_t port,
                                       const unsigned char *packet)
{
    unsigned char reply[LIDARSIM_CONTROL_REPLY_MAX];
    unsigned proto_type = packet[3];
    unsigned payload_len = 16u;
    const unsigned char *payload = packet + 7;

    if (proto_type == LIDAR_PROTO_VARLEN) {
        payload_len = read_le16(packet + 7);
        payload = packet + 9;
    }

    unsigned reply_len = build_control_reply(proto_type, packet[4], packet[5],
                                             packet[6], payload, payload_len,
                                             reply);
    err_t err = send_udp_bytes(pcb, addr, port, reply, reply_len);
    printf("lidarsim udp cmd=%x rw=%d reply=%d err=%d\n",
           packet[5], packet[6], reply_len, err);
    if (proto_type == LIDAR_PROTO_VARLEN &&
        packet[5] == LIDAR_CMD_NET_CONFIG &&
        packet[6] == LIDAR_PROTO_WRITE) {
        apply_net_config_payload(payload, payload_len);
    }
    if (err == ERR_OK && sim_progress_flags != 0xffu &&
        packet[5] != LIDAR_CMD_NET_CONFIG) {
        sim_progress_flags |= SIM_FLAG_CONTROL;
        maybe_report_sim_ok();
    }
}

static unsigned control_packet_size(const unsigned char *buf, unsigned len)
{
    if (len < 7u || buf[0] != 0xffu || buf[1] != 0xfeu) {
        return 0;
    }

    if (buf[3] == LIDAR_PROTO_FIXED) {
        if (len < 25u) {
            return 0;
        }
        return (buf[23] == 0xffu && buf[24] == 0x9bu) ? 25u : 1u;
    }

    if (buf[3] == LIDAR_PROTO_VARLEN) {
        if (len < 9u) {
            return 0;
        }
        unsigned payload_len = read_le16(buf + 7);
        unsigned packet_len = 11u + payload_len;
        if (packet_len > LIDARSIM_CONTROL_BUF_MAX ||
            packet_len > LIDARSIM_CONTROL_REPLY_MAX) {
            return 1u;
        }
        if (len < packet_len) {
            return 0;
        }
        return (buf[packet_len - 2u] == 0xffu &&
                buf[packet_len - 1u] == 0x9bu) ? packet_len : 1u;
    }

    return 1u;
}

static void parse_control_stream(struct CONTROL_CONN *conn)
{
    while (conn->len >= 2u) {
        if (conn->buf[0] != 0xffu || conn->buf[1] != 0xfeu) {
            unsigned start = 1u;
            while (start + 1u < conn->len &&
                   !(conn->buf[start] == 0xffu && conn->buf[start + 1u] == 0xfeu)) {
                start++;
            }
            memmove(conn->buf, conn->buf + start, conn->len - start);
            conn->len -= start;
            continue;
        }

        unsigned packet_len = control_packet_size(conn->buf, conn->len);
        if (packet_len == 0u) {
            return;
        }
        if (packet_len == 1u) {
            memmove(conn->buf, conn->buf + 1, conn->len - 1u);
            conn->len--;
            continue;
        }

        process_control_packet_tcp(conn->pcb, conn->buf, packet_len);
        memmove(conn->buf, conn->buf + packet_len, conn->len - packet_len);
        conn->len -= packet_len;
    }
}

static void udp_command_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                             const ip_addr_t *addr, u16_t port)
{
    (void)arg;

    if (!p) {
        return;
    }

    unsigned char packet[LIDARSIM_CONTROL_BUF_MAX];
    unsigned len = p->tot_len;
    if (len > LIDARSIM_CONTROL_BUF_MAX) {
        len = LIDARSIM_CONTROL_BUF_MAX;
    }
    pbuf_copy_partial(p, packet, (u16_t)len, 0);
    pbuf_free(p);

    unsigned packet_len = control_packet_size(packet, len);
    if (packet_len > 1u) {
        process_control_packet_udp(pcb, addr, port, packet);
    }
}

static unsigned is_discovery_request(const unsigned char *data, unsigned len)
{
    static const unsigned char expected[] = {
        0xff, 0xfe, 'L', 'I', 'D', 'A', 'R', '_', 'R', 'E', 'Q', 'S',
        0x00, 0x01, LIDAR_DISCOVERY_REQUEST_SIZE, 0xff, 0x9b
    };

    if (len < sizeof(expected)) {
        return 0;
    }
    return memcmp(data, expected, sizeof(expected)) == 0;
}

static void build_discovery_response(unsigned char out[LIDAR_DISCOVERY_RESPONSE_SIZE])
{
    static const char sig[] = "LIDAR_RESP";
    static const char model[] = LIDARSIM_MODEL;
    static const char fw[] = LIDARSIM_FIRMWARE;

    for (unsigned i = 0; i < LIDAR_DISCOVERY_RESPONSE_SIZE; i++) {
        out[i] = 0;
    }

    out[0] = 0xff;
    out[1] = 0xfe;
    for (unsigned i = 0; i < sizeof(sig); i++) {
        out[2 + i] = (unsigned char)sig[i];
    }
    out[13] = 1;
    out[14] = 1;
    out[15] = LIDAR_DISCOVERY_RESPONSE_SIZE;
    write_le32(out + 16, 0xffffffffu);
    copy_bytes(out + 20, runtime_config.mac, 6);
    copy_bytes(out + 26, runtime_config.ip, 4);
    copy_bytes(out + 30, runtime_config.remote_ip, 4);
    write_le16(out + 34, runtime_config.data_port);
    write_le16(out + 36, runtime_config.data_remote_port);
    write_le16(out + 38, runtime_config.cmd_port);
    write_le16(out + 40, runtime_config.cmd_remote_port);
    write_le16(out + 42, 0);
    write_le16(out + 44, runtime_config.discovery_port);
    for (unsigned i = 0; i < (sizeof(model) - 1u); i++) {
        out[46 + i] = (unsigned char)model[i];
    }
    for (unsigned i = 0; i < (sizeof(fw) - 1u); i++) {
        out[46 + sizeof(model) + i] = (unsigned char)fw[i];
    }
    out[78] = 0xff;
    out[79] = 0x9b;
}

static void udp_discovery_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                               const ip_addr_t *addr, u16_t port)
{
    (void)arg;

    if (!p) {
        return;
    }

    unsigned char packet[LIDARSIM_CONTROL_BUF_MAX];
    unsigned len = p->tot_len;
    if (len > LIDARSIM_CONTROL_BUF_MAX) {
        len = LIDARSIM_CONTROL_BUF_MAX;
    }
    pbuf_copy_partial(p, packet, (u16_t)len, 0);
    pbuf_free(p);

    printf("lidarsim discovery rx len=%d port=%d\n", len, port);

    if (is_discovery_request(packet, len)) {
        unsigned char reply[LIDAR_DISCOVERY_RESPONSE_SIZE];
        build_discovery_response(reply);
        err_t broadcast_err = send_udp_broadcast_bytes(pcb, port, reply, sizeof(reply));
        err_t direct_err = send_udp_unicast_to_last_peer(
            (u16_t)runtime_config.discovery_port, port, reply, sizeof(reply));
        printf("lidarsim discovery reply broadcast=%d direct=%d\n",
               broadcast_err, direct_err);
        if ((broadcast_err == ERR_OK || direct_err == ERR_OK) &&
            sim_progress_flags != 0xffu) {
            sim_progress_flags |= SIM_FLAG_DISCOVERY;
            maybe_report_sim_ok();
        }
        return;
    }

    unsigned packet_len = control_packet_size(packet, len);
    if (packet_len > 1u && packet[5] == LIDAR_CMD_NET_CONFIG) {
        process_control_packet_udp(pcb, addr, port, packet);
    }
}

static err_t tcp_control_recv(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    struct CONTROL_CONN *conn = (struct CONTROL_CONN *)arg;

    if (err != ERR_OK) {
        if (p) pbuf_free(p);
        return err;
    }

    if (!p) {
        tcp_close(tpcb);
        if (conn) {
            conn->pcb = 0;
            conn->len = 0;
        }
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *src = (unsigned char *)q->payload;
        for (u16_t i = 0; i < q->len; i++) {
            if (conn->len < sizeof(conn->buf)) {
                conn->buf[conn->len++] = src[i];
            } else {
                conn->len = 0;
            }
        }
    }
    pbuf_free(p);
    parse_control_stream(conn);
    return ERR_OK;
}

static void tcp_control_err(void *arg, err_t err)
{
    struct CONTROL_CONN *conn = (struct CONTROL_CONN *)arg;
    if (conn) {
        conn->pcb = 0;
        conn->len = 0;
    }
    printf("lidarsim tcp control err=%d\n", err);
}

static err_t tcp_command_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;

    if (err != ERR_OK || !newpcb) {
        return ERR_VAL;
    }

    if (tcp_control.pcb && tcp_control.pcb != newpcb) {
        tcp_abort(tcp_control.pcb);
    }

    tcp_control.pcb = newpcb;
    tcp_control.len = 0;
    tcp_arg(newpcb, &tcp_control);
    tcp_recv(newpcb, tcp_control_recv);
    tcp_err(newpcb, tcp_control_err);
    printf("lidarsim tcp command connected\n");
    return ERR_OK;
}

static err_t tcp_firmware_recv(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
    struct FIRMWARE_CONN *conn = (struct FIRMWARE_CONN *)arg;

    if (err != ERR_OK) {
        if (p) pbuf_free(p);
        return err;
    }

    if (!p) {
        tcp_close(tpcb);
        if (conn) {
            conn->pcb = 0;
            conn->len = 0;
        }
        firmware_session_clear();
        printf("fwloader tcp closed\n");
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *src = (unsigned char *)q->payload;
        for (u16_t i = 0; i < q->len; i++) {
            if (conn->len < sizeof(conn->buf)) {
                conn->buf[conn->len++] = src[i];
            } else {
                conn->len = 0;
                firmware_session_clear();
            }
        }
    }
    pbuf_free(p);
    parse_firmware_stream(conn);
    return ERR_OK;
}

static void tcp_firmware_err(void *arg, err_t err)
{
    struct FIRMWARE_CONN *conn = (struct FIRMWARE_CONN *)arg;
    if (conn) {
        conn->pcb = 0;
        conn->len = 0;
    }
    firmware_session_clear();
    printf("fwloader tcp err=%d\n", err);
}

static err_t tcp_firmware_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;

    if (err != ERR_OK || !newpcb) {
        return ERR_VAL;
    }

    if (tcp_firmware.pcb && tcp_firmware.pcb != newpcb) {
        tcp_abort(tcp_firmware.pcb);
    }

    firmware_session_clear();
    tcp_firmware.pcb = newpcb;
    tcp_firmware.len = 0;
    tcp_arg(newpcb, &tcp_firmware);
    tcp_recv(newpcb, tcp_firmware_recv);
    tcp_err(newpcb, tcp_firmware_err);
    printf("fwloader tcp connected\n");
    return ERR_OK;
}

static void msop_tx_reset(void)
{
    last_msop_ms = 0;
    msop_tx_head = 0;
    msop_tx_tail = 0;
    msop_tx_inflight = 0;
    for (unsigned i = 0; i < LIDARSIM_MSOP_TX_BUFFERS; i++) {
        msop_nocopy_unacked_bytes[i] = 0;
    }
}

static void msop_tx_acked(unsigned len)
{
    while (len && msop_tx_inflight) {
        unsigned pending = msop_nocopy_unacked_bytes[msop_tx_tail];

        if (len < pending) {
            msop_nocopy_unacked_bytes[msop_tx_tail] = pending - len;
            return;
        }

        len -= pending;
        msop_nocopy_unacked_bytes[msop_tx_tail] = 0;
        msop_tx_tail++;
        if (msop_tx_tail >= LIDARSIM_MSOP_TX_BUFFERS) {
            msop_tx_tail = 0;
        }
        msop_tx_inflight--;
    }
}

static err_t tcp_data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    (void)arg;
    (void)tpcb;
    msop_tx_acked(len);
    return ERR_OK;
}

static err_t tcp_data_recv(void *arg, struct tcp_pcb *tpcb,
                           struct pbuf *p, err_t err)
{
    (void)arg;
    if (err != ERR_OK) {
        if (p) pbuf_free(p);
        return err;
    }
    if (!p) {
        tcp_close(tpcb);
        if (tcp_data_client == tpcb) {
            tcp_data_client = 0;
        }
        msop_tx_reset();
        printf("lidarsim tcp data closed\n");
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static void tcp_data_err(void *arg, err_t err)
{
    (void)arg;
    tcp_data_client = 0;
    msop_tx_reset();
    printf("lidarsim tcp data err=%d\n", err);
}

static err_t tcp_data_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;

    if (err != ERR_OK || !newpcb) {
        return ERR_VAL;
    }

    if (tcp_data_client && tcp_data_client != newpcb) {
        tcp_abort(tcp_data_client);
    }

    tcp_data_client = newpcb;
    tcp_arg(newpcb, 0);
    tcp_nagle_disable(newpcb);
    tcp_recv(newpcb, tcp_data_recv);
    tcp_sent(newpcb, tcp_data_sent);
    tcp_err(newpcb, tcp_data_err);
    msop_tx_reset();
    printf("lidarsim tcp data connected\n");
    return ERR_OK;
}

static err_t udp_listener_init(struct udp_pcb **slot, unsigned port,
                               udp_recv_fn cb)
{
    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        return ERR_MEM;
    }

    err_t err = udp_bind(pcb, IP_ANY_TYPE, (u16_t)port);
    if (err != ERR_OK) {
        udp_remove(pcb);
        return err;
    }

    udp_recv(pcb, cb, 0);
    if (port == LIDARSIM_DISCOVERY_PORT || port == LIDARSIM_CMD_PORT) {
        ip_set_option(pcb, SOF_BROADCAST);
    }
    *slot = pcb;
    return ERR_OK;
}

static err_t tcp_listener_init(struct tcp_pcb **slot, unsigned port,
                               tcp_accept_fn accept_cb)
{
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        return ERR_MEM;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, (u16_t)port);
    if (err != ERR_OK) {
        tcp_abort(pcb);
        return err;
    }

    struct tcp_pcb *listener = tcp_listen(pcb);
    if (!listener) {
        tcp_abort(pcb);
        return ERR_MEM;
    }

    tcp_arg(listener, 0);
    tcp_accept(listener, accept_cb);
    *slot = listener;
    return ERR_OK;
}

static void service_pending_network_reconfig(void)
{
    if (!network_reconfig_pending) {
        return;
    }

    network_reconfig_pending = 0;
    if (netif_configured) {
        apply_netif_config();
    }
}

static unsigned angle_distance_deg(unsigned angle_deg, unsigned center_deg)
{
    unsigned diff = (angle_deg >= center_deg) ?
        (angle_deg - center_deg) : (center_deg - angle_deg);
    return (diff > 180u) ? (360u - diff) : diff;
}

static unsigned rotate_angle_deg(unsigned angle_deg, unsigned phase_deg)
{
    unsigned a = angle_deg + phase_deg;
    return (a >= 360u) ? (a - 360u) : a;
}

static unsigned angle_peak_slope_mm(unsigned angle_deg, unsigned center_deg,
                                    unsigned half_width_deg,
                                    unsigned slope_mm_per_deg)
{
    unsigned diff = angle_distance_deg(angle_deg, center_deg);
    if (diff >= half_width_deg) {
        return 0;
    }
    return (half_width_deg - diff) * slope_mm_per_deg;
}

#define PIG_SNOUT_CENTER_DEG      270u
#define PIG_SNOUT_HALF_WIDTH_DEG  50u
#define PIG_SNOUT_CENTER_MM       1860u
#define PIG_SNOUT_OUTER_SLOPE_MM  10u
#define PIG_SNOUT_INNER_SLOPE_MM  8u

static unsigned pig_head_distance_mm(unsigned angle_deg, unsigned phase_deg)
{
    unsigned a = rotate_angle_deg(angle_deg, phase_deg);
    unsigned dist = 1960u;
    unsigned saddle;

    dist += angle_peak_slope_mm(a, 60u, 24u, 30u);
    dist += angle_peak_slope_mm(a, 120u, 24u, 30u);
    dist += angle_peak_slope_mm(a, 220u, 32u, 4u);
    dist += angle_peak_slope_mm(a, 320u, 32u, 4u);

    saddle = angle_peak_slope_mm(a, 90u, 15u, 12u);
    return (dist > saddle) ? (dist - saddle) : 1700u;
}

static unsigned pig_snout_distance_mm(unsigned angle_deg, unsigned phase_deg)
{
    unsigned a = rotate_angle_deg(angle_deg, phase_deg);
    unsigned diff = angle_distance_deg(a, PIG_SNOUT_CENTER_DEG);
    unsigned span;
    unsigned inner;
    unsigned outer;

    if (diff >= PIG_SNOUT_HALF_WIDTH_DEG) {
        return LIDARSIM_MSOP_INVALID_DISTANCE;
    }

    span = PIG_SNOUT_HALF_WIDTH_DEG - diff;
    outer = PIG_SNOUT_CENTER_MM + span * PIG_SNOUT_OUTER_SLOPE_MM;
    inner = PIG_SNOUT_CENTER_MM - span * PIG_SNOUT_INNER_SLOPE_MM;

    /* One second-echo sample per angle: alternate outer/inner snout points. */
    return ((a >> 1) & 1u) ? outer : inner;
}

static unsigned pig_head_intensity(unsigned angle_deg, unsigned phase_deg,
                                   unsigned frame_num)
{
    unsigned a = rotate_angle_deg(angle_deg, phase_deg);
    unsigned value = 50u + ((angle_deg + frame_num) & 0x1fu);

    if (angle_distance_deg(a, 60u) <= 20u ||
        angle_distance_deg(a, 120u) <= 20u) {
        value = 112u + ((frame_num + angle_deg) & 0x1fu);
    }

    return value;
}

static unsigned pig_snout_intensity(unsigned angle_deg, unsigned phase_deg,
                                    unsigned frame_num)
{
    unsigned a = rotate_angle_deg(angle_deg, phase_deg);
    unsigned diff = angle_distance_deg(a, 270u);

    if (diff <= 10u) {
        return 205u + ((frame_num + angle_deg) & 0x1fu);
    }
    return 168u + ((frame_num + angle_deg) & 0x1fu);
}

static unsigned pig_head_phase_deg(unsigned now_ms)
{
    return ((now_ms % LIDARSIM_PIG_PERIOD_MS) * 360u) /
           LIDARSIM_PIG_PERIOD_MS;
}

static unsigned build_msop_packet(unsigned char *out)
{
    unsigned pos = 0;
    unsigned now = sys_now();
    unsigned phase = pig_head_phase_deg(now);

    out[pos++] = 0xff;
    out[pos++] = 0xfe;
    out[pos++] = 1;
    out[pos++] = 1;
    out[pos++] = 0;
    write_le16(out + pos, msop_frame_num & 0xffffu); pos += 2;
    write_le16(out + pos, LIDARSIM_MSOP_POINTS); pos += 2;
    write_le32(out + pos, now / 1000u); pos += 4;
    write_le32(out + pos, (now % 1000u) * 1000u); pos += 4;
    write_le32(out + pos, 0); pos += 4;
    out[pos++] = 1;
    write_le32(out + pos, 250000u); pos += 4;
    write_le32(out + pos, 360000u); pos += 4;
    write_le16(out + pos, 2000u); pos += 2;
    out[pos++] = LIDARSIM_MSOP_DISTANCE_BYTES;
    out[pos++] = LIDARSIM_MSOP_INTENSITY_BYTES;
    out[pos++] = LIDARSIM_MSOP_ECHO_MODE;
    out[pos++] = LIDARSIM_MSOP_ECHO_COUNT;

    for (unsigned i = 0; i < LIDARSIM_MSOP_POINTS; i++) {
        unsigned angle_deg = (i * 2u) % 360u;
        unsigned head_dist = pig_head_distance_mm(angle_deg, phase);
        unsigned snout_dist = pig_snout_distance_mm(angle_deg, phase);

        write_msop_distance(out + pos, head_dist);
        pos += LIDARSIM_MSOP_DISTANCE_BYTES;
        if (LIDARSIM_MSOP_INTENSITY_BYTES) {
            out[pos++] = (unsigned char)
                pig_head_intensity(angle_deg, phase, msop_frame_num);
        }

        write_msop_distance(out + pos, snout_dist);
        pos += LIDARSIM_MSOP_DISTANCE_BYTES;
        if (LIDARSIM_MSOP_INTENSITY_BYTES) {
            out[pos++] = (snout_dist == LIDARSIM_MSOP_INVALID_DISTANCE) ? 0u :
                (unsigned char)pig_snout_intensity(angle_deg, phase,
                                                   msop_frame_num);
        }
    }

    out[pos++] = 0xff;
    out[pos++] = 0x9b;
    return pos;
}

static void service_msop_tcp(void)
{
    static unsigned char packets[LIDARSIM_MSOP_TX_BUFFERS][LIDARSIM_MSOP_PACKET_MAX];

    if (!tcp_data_client) {
        return;
    }

    if (msop_tx_inflight >= LIDARSIM_MSOP_TX_BUFFERS) {
        return;
    }

    unsigned now = sys_now();
    if (last_msop_ms && (now - last_msop_ms) < LIDARSIM_MSOP_PERIOD_MS) {
        return;
    }

    unsigned buf_index = msop_tx_head;
    unsigned char *packet = packets[buf_index];
    unsigned len = build_msop_packet(packet);
    if (tcp_sndbuf(tcp_data_client) < len) {
        return;
    }

    err_t err = tcp_write(tcp_data_client, packet, (u16_t)len, 0);
    if (err == ERR_OK) {
        tcp_output(tcp_data_client);
    }
    if (err == ERR_OK) {
        msop_frame_num++;
        last_msop_ms = now;
        msop_nocopy_unacked_bytes[buf_index] = len;
        msop_tx_head++;
        if (msop_tx_head >= LIDARSIM_MSOP_TX_BUFFERS) {
            msop_tx_head = 0;
        }
        msop_tx_inflight++;
    }
}

static int is_space(char c)
{
    return c == ' ' || c == '\t';
}

static char *skip_spaces(char *text)
{
    while (is_space(*text)) {
        text++;
    }
    return text;
}

static int is_line_end(char c)
{
    return c == 0 || c == ' ' || c == '\t';
}

static int command_is(char *line, const char *cmd)
{
    while (*cmd && *line == *cmd) {
        line++;
        cmd++;
    }

    return *cmd == 0 && is_line_end(*line);
}

static char *command_arg(char *line)
{
    while (*line && !is_space(*line)) {
        line++;
    }
    return skip_spaces(line);
}

static int parse_dec(char **cursor, unsigned *value)
{
    char *p = skip_spaces(*cursor);
    unsigned parsed = 0;
    unsigned digits = 0;

    while (*p >= '0' && *p <= '9') {
        parsed = (parsed * 10u) + (unsigned)(*p - '0');
        digits++;
        p++;
    }

    if (!digits) {
        return 0;
    }

    *value = parsed;
    *cursor = p;
    return 1;
}

static int parse_ipv4(char *text, unsigned char out[4])
{
    char *p = text;

    for (unsigned i = 0; i < 4; i++) {
        unsigned octet = 0;
        if (!parse_dec(&p, &octet) || octet > 255u) {
            return 0;
        }
        out[i] = (unsigned char)octet;

        if (i != 3) {
            if (*p != '.') {
                return 0;
            }
            p++;
        }
    }

    p = skip_spaces(p);
    return *p == 0;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return -1;
}

static int parse_mac(char *text, unsigned char out[6])
{
    char *p = skip_spaces(text);

    for (unsigned i = 0; i < 6; i++) {
        int high = hex_nibble(*p++);
        int low = hex_nibble(*p++);

        if (high < 0 || low < 0) {
            return 0;
        }

        out[i] = (unsigned char)((high << 4) | low);

        if (i != 5 && (*p == ':' || *p == '-')) {
            p++;
        }
    }

    p = skip_spaces(p);
    return *p == 0;
}

static void handle_uart_command(char *line)
{
    line = skip_spaces(line);
    char *arg = command_arg(line);
    unsigned char parsed_bytes[6];

    if (*line == 0) {
        return;
    }

    if (command_is(line, "show") || command_is(line, "cfg")) {
        print_config();
        return;
    }

    if (command_is(line, "flashid") || command_is(line, "fwid")) {
        unsigned char id[3];
        firmware_session.mock_flash = (io->board_id == 0u);
        if (flash_read_jedec_id(id)) {
            printf("fwloader flash_id=");
            print_hex_byte(id[0]);
            print_hex_byte(id[1]);
            print_hex_byte(id[2]);
            printf(" ok=1\n");
        } else {
            printf("fwloader flash_id=");
            print_hex_byte(id[0]);
            print_hex_byte(id[1]);
            print_hex_byte(id[2]);
            printf(" ok=0\n");
        }
        return;
    }

    if (command_is(line, "mac")) {
        if (!parse_mac(arg, parsed_bytes)) {
            printf("lidarsim cfg bad mac\n");
            return;
        }
        copy_bytes(runtime_config.mac, parsed_bytes, 6);
        apply_netif_config();
        print_config();
        return;
    }

    if (command_is(line, "ip")) {
        if (!parse_ipv4(arg, parsed_bytes)) {
            printf("lidarsim cfg bad ip\n");
            return;
        }
        copy_bytes(runtime_config.ip, parsed_bytes, 4);
        apply_netif_config();
        print_config();
        return;
    }

    printf("lidarsim cfg unknown\n");
}

static void poll_uart_config(void)
{
    while (io->uart.stat & 2) {
        char c = io->uart.fifo;

        if (c == '\r' || c == '\n') {
            if (uart_line_len != 0) {
                uart_line[uart_line_len] = 0;
                handle_uart_command(uart_line);
                uart_line_len = 0;
            }
        } else if (c == '\b' || c == 127) {
            if (uart_line_len != 0) {
                uart_line_len--;
            }
        } else if (uart_line_len < (LIDARSIM_UART_LINE_MAX - 1u)) {
            uart_line[uart_line_len++] = c;
        } else {
            uart_line_len = 0;
            printf("lidarsim cfg line too long\n");
        }
    }
}

static void drain_rx_frame(unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        (void)eth->rx_data;
    }
    eth->rx_ctrl = ETH_RX_CTRL_RELEASE;
}

static err_t poll_rx_frame(void)
{
    unsigned status = eth->status;
    if (!(status & ETH_STATUS_RX_AVAILABLE)) {
        return ERR_OK;
    }

    unsigned len = eth->rx_len;

    if ((len == 0) || (len > LIDARSIM_MAX_FRAME)) {
        printf("lidarsim rx drop len=%d status=%x\n", len, status);
        drain_rx_frame(len);
        eth->rx_ctrl = ETH_RX_CTRL_CLEAR_FLAGS;
        return ERR_BUF;
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL);
    if (p == 0) {
        printf("lidarsim rx no pbuf len=%d\n", len);
        drain_rx_frame(len);
        return ERR_MEM;
    }

    unsigned remaining = len;
    unsigned header_len = 0;
    unsigned char header[34];
    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *dst = (unsigned char *)q->payload;
        for (u16_t i = 0; (i < q->len) && remaining; i++) {
            unsigned char value = (unsigned char)(eth->rx_data & 0xff);
            dst[i] = value;
            if (header_len < sizeof(header)) {
                header[header_len++] = value;
            }
            remaining--;
        }
    }
    capture_rx_ipv4_peer(header, header_len);

    err_t err = fpga_netif.input(p, &fpga_netif);
    if (err != ERR_OK) {
        printf("lidarsim input err=%d\n", err);
        pbuf_free(p);
    }

    if (eth->status & (ETH_STATUS_RX_OVERFLOW | ETH_STATUS_RX_DROPPED)) {
        printf("lidarsim rx flags=%x\n", eth->status);
        eth->rx_ctrl = ETH_RX_CTRL_CLEAR_FLAGS;
    }

    return err;
}

int main(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;

    printf("lidarsim start\n");

    lwip_init();
    print_config();
    firmware_selftest();

    ip4_from_config(&ipaddr, runtime_config.ip);
    ip4_from_config(&netmask, runtime_config.netmask);
    ip4_from_config(&gw, runtime_config.gateway);

    if (netif_add(&fpga_netif, &ipaddr, &netmask, &gw, 0,
                  darketh_netif_init, ethernet_input) == 0) {
        printf("lidarsim netif fail\n>");
        return 1;
    }

    netif_configured = 1;
    apply_netif_config();
    netif_set_default(&fpga_netif);
    netif_set_up(&fpga_netif);
    netif_set_link_up(&fpga_netif);

    err_t err = udp_listener_init(&udp_discovery_listener,
                                  runtime_config.discovery_port,
                                  udp_discovery_recv);
    printf("lidarsim udp discovery bind=%d port=%d\n", err,
           runtime_config.discovery_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    err = udp_listener_init(&udp_command_listener, runtime_config.cmd_port,
                            udp_command_recv);
    printf("lidarsim udp command bind=%d port=%d\n", err,
           runtime_config.cmd_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    err = tcp_listener_init(&tcp_data_listener, runtime_config.data_port,
                            tcp_data_accept);
    printf("lidarsim tcp data listen=%d port=%d\n", err,
           runtime_config.data_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    err = tcp_listener_init(&tcp_command_listener, runtime_config.cmd_port,
                            tcp_command_accept);
    printf("lidarsim tcp command listen=%d port=%d\n", err,
           runtime_config.cmd_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    err = tcp_listener_init(&tcp_firmware_listener, LIDARSIM_FIRMWARE_PORT,
                            tcp_firmware_accept);
    printf("fwloader tcp listen=%d port=%d\n", err,
           LIDARSIM_FIRMWARE_PORT);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    while (1) {
        poll_uart_config();
        service_pending_network_reconfig();
        poll_rx_frame();
        sys_check_timeouts();
        service_msop_tcp();
    }
}
