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
#define LIDARSIM_MSOP_POINTS      180u
#define LIDARSIM_MSOP_PACKET_MAX  (2u + 34u + LIDARSIM_MSOP_POINTS * 4u + 2u)
#define LIDARSIM_CONTROL_BUF_MAX  256u
#define LIDARSIM_CONTROL_REPLY_MAX 128u

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

static volatile struct DARKETH *eth = (volatile struct DARKETH *)DARKETH_BASE;
static struct netif fpga_netif;
static struct udp_pcb *udp_discovery_listener;
static struct udp_pcb *udp_command_listener;
static struct tcp_pcb *tcp_data_listener;
static struct tcp_pcb *tcp_command_listener;
static struct tcp_pcb *tcp_data_client;
static struct CONTROL_CONN tcp_control;
static unsigned netif_configured;
static char uart_line[LIDARSIM_UART_LINE_MAX];
static unsigned uart_line_len;
static unsigned msop_frame_num;
static unsigned last_msop_ms;
static unsigned target_speed_bits = 0x41a00000u;
static unsigned voltage_ld = 27u;
static unsigned voltage_pd = 150u;
static unsigned preset_pd = 3u;
static unsigned heater_mode;
static unsigned fpga_angle_res = 500u;
static unsigned view_sector_start = 25000u;
static unsigned view_sector_end = 167000u;
static unsigned sim_progress_flags;

static struct LIDARSIM_CONFIG runtime_config = {
    .mac = {0x02, 0x20, 0x20, 0x20, 0x20, 0x01},
    .ip = {192, 168, 20, 20},
    .netmask = {255, 255, 255, 0},
    .gateway = {192, 168, 20, 1},
    .remote_ip = {192, 168, 20, 10},
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

static void write_le24(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
    dst[2] = (unsigned char)((value >> 16) & 0xffu);
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

static void print_config(void)
{
    printf("lidarsim cfg mac=");
    print_mac(runtime_config.mac);
    printf(" ip=%d.%d.%d.%d data=%d cmd=%d discovery=%d model=R120_FAKE fw=pegus_1\n",
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
    netif_set_addr(&fpga_netif, &ipaddr, &netmask, &gw);
    darketh_apply_filter_config();
}

static void copy_sender_ip(const ip_addr_t *addr, unsigned char out[4])
{
    if (!addr || !IP_IS_V4(addr)) {
        return;
    }

    const ip4_addr_t *ip4 = ip_2_ip4(addr);
    out[0] = ip4_addr1(ip4);
    out[1] = ip4_addr2(ip4);
    out[2] = ip4_addr3(ip4);
    out[3] = ip4_addr4(ip4);
}

static void apply_net_config_payload(const unsigned char *payload, unsigned len)
{
    if (!payload || len < 24u || !mac_matches(payload)) {
        return;
    }

    copy_bytes(runtime_config.ip, payload + 6, 4);
    copy_bytes(runtime_config.remote_ip, payload + 10, 4);
    runtime_config.data_port = read_le16(payload + 15);
    runtime_config.data_remote_port = read_le16(payload + 17);
    runtime_config.cmd_port = read_le16(payload + 20);
    runtime_config.cmd_remote_port = read_le16(payload + 22);

    if (runtime_config.data_port == 0) runtime_config.data_port = LIDARSIM_DATA_PORT;
    if (runtime_config.data_remote_port == 0) runtime_config.data_remote_port = LIDARSIM_DATA_PORT;
    if (runtime_config.cmd_port == 0) runtime_config.cmd_port = LIDARSIM_CMD_PORT;
    if (runtime_config.cmd_remote_port == 0) runtime_config.cmd_remote_port = LIDARSIM_CMD_PORT;

    if (netif_configured) {
        apply_netif_config();
    }
    printf("lidarsim net_config applied\n");
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
        payload[0] = 1;
        payload[1] = 3;
        payload[2] = 1;
        break;
    default:
        break;
    }
}

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
        if (cmd == LIDAR_CMD_LIDAR_FIRMWARE && rw == LIDAR_PROTO_READ) {
            static const char fw[] = "pegus_1";
            len = sizeof(fw) - 1u;
            write_le16(out + 7, len);
            for (unsigned i = 0; i < len; i++) {
                out[9 + i] = (unsigned char)fw[i];
            }
        } else {
            if (cmd == LIDAR_CMD_NET_CONFIG && rw == LIDAR_PROTO_WRITE) {
                apply_net_config_payload(payload, payload_len);
            }
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

    fill_fixed_payload(cmd, rw, payload, out + 7);
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
    if (sim_progress_flags == 3u) {
        sim_progress_flags = 0xffu;
        printf("lidarsim ok\n>");
    }
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
    if (err == ERR_OK && sim_progress_flags != 0xffu) {
        sim_progress_flags |= 2u;
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
    if (len > sizeof(packet)) {
        len = sizeof(packet);
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
    static const char model[] = "R120_FAKE";

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
    if (len > sizeof(packet)) {
        len = sizeof(packet);
    }
    pbuf_copy_partial(p, packet, (u16_t)len, 0);
    pbuf_free(p);

    printf("lidarsim discovery rx len=%d port=%d\n", len, port);
    copy_sender_ip(addr, runtime_config.remote_ip);

    if (is_discovery_request(packet, len)) {
        unsigned char reply[LIDAR_DISCOVERY_RESPONSE_SIZE];
        build_discovery_response(reply);
        err_t err = send_udp_bytes(pcb, addr, port, reply, sizeof(reply));
        printf("lidarsim discovery reply err=%d\n", err);
        if (err == ERR_OK && sim_progress_flags != 0xffu) {
            sim_progress_flags |= 1u;
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
    tcp_recv(newpcb, tcp_data_recv);
    tcp_err(newpcb, tcp_data_err);
    last_msop_ms = 0;
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
    if (port == LIDARSIM_DISCOVERY_PORT) {
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

static unsigned square_distance_mm(unsigned angle_deg, unsigned phase_deg)
{
    unsigned a = (angle_deg + phase_deg) % 90u;
    unsigned edge = (a <= 45u) ? a : (90u - a);
    return 1800u + edge * 12u;
}

static unsigned build_msop_packet(unsigned char *out)
{
    unsigned pos = 0;
    unsigned now = sys_now();
    unsigned phase = (msop_frame_num * 3u) % 90u;

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
    out[pos++] = 3;
    out[pos++] = 1;
    out[pos++] = 1;
    out[pos++] = 1;

    for (unsigned i = 0; i < LIDARSIM_MSOP_POINTS; i++) {
        unsigned angle_deg = (i * 2u) % 360u;
        unsigned dist = square_distance_mm(angle_deg, phase);
        write_le24(out + pos, dist); pos += 3;
        out[pos++] = (unsigned char)(40u + ((i + msop_frame_num) & 0x3fu));
    }

    out[pos++] = 0xff;
    out[pos++] = 0x9b;
    return pos;
}

static void service_msop_tcp(void)
{
    static unsigned char packet[LIDARSIM_MSOP_PACKET_MAX];

    if (!tcp_data_client) {
        return;
    }

    unsigned now = sys_now();
    if (last_msop_ms && (now - last_msop_ms) < 50u) {
        return;
    }

    unsigned len = build_msop_packet(packet);
    if (tcp_sndbuf(tcp_data_client) < len) {
        return;
    }

    err_t err = send_tcp_bytes(tcp_data_client, packet, len);
    if (err == ERR_OK) {
        msop_frame_num++;
        last_msop_ms = now;
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

    if ((status & (ETH_STATUS_RX_OVERFLOW | ETH_STATUS_RX_DROPPED)) ||
        (len == 0) || (len > LIDARSIM_MAX_FRAME)) {
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
    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *dst = (unsigned char *)q->payload;
        for (u16_t i = 0; (i < q->len) && remaining; i++) {
            dst[i] = eth->rx_data & 0xff;
            remaining--;
        }
    }

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

    while (1) {
        poll_uart_config();
        poll_rx_frame();
        sys_check_timeouts();
        service_msop_tcp();
    }
}
