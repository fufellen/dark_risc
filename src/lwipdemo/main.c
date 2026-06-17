#include <io.h>
#include <stdio.h>

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
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

#define LWIPDEMO_MAX_FRAME      1518u
#define LWIPDEMO_UART_LINE_MAX  48u

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

static volatile struct DARKETH *eth = (volatile struct DARKETH *)DARKETH_BASE;
static struct netif fpga_netif;
static struct udp_pcb *udp_listener;
static volatile unsigned udp_seen;
static unsigned netif_configured;
static char uart_line[LWIPDEMO_UART_LINE_MAX];
static unsigned uart_line_len;

struct LWIPDEMO_CONFIG {
    unsigned char mac[6];
    unsigned char ip[4];
    unsigned char netmask[4];
    unsigned char gateway[4];
    unsigned udp_port;
};

static struct LWIPDEMO_CONFIG runtime_config = {
    .mac = {0x02, 0x20, 0x20, 0x20, 0x20, 0x01},
    .ip = {192, 168, 20, 20},
    .netmask = {255, 255, 255, 0},
    .gateway = {192, 168, 20, 1},
    .udp_port = 5005u,
};

u32_t sys_now(void)
{
    return io->timeus / 1000u;
}

static void ip4_from_config(ip4_addr_t *addr, const unsigned char octets[4])
{
    IP4_ADDR(addr, octets[0], octets[1], octets[2], octets[3]);
}

static void print_config(void)
{
    printf("lwipdemo cfg mac=");
    for (unsigned i = 0; i < 6; i++) {
        printf("%x", runtime_config.mac[i]);
    }

    printf(" ip=%d.%d.%d.%d port=%d\n",
           runtime_config.ip[0], runtime_config.ip[1],
           runtime_config.ip[2], runtime_config.ip[3],
           runtime_config.udp_port);
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

    if (p->tot_len > LWIPDEMO_MAX_FRAME) {
        printf("lwipdemo tx too big=%d\n", p->tot_len);
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
        printf("lwipdemo tx stage err=%x\n", staged);
        return ERR_IF;
    }

    eth->tx_ctrl = ETH_TX_CTRL_START;
    while (!(eth->tx_status & ETH_TX_STATUS_DONE) && timeout) {
        timeout--;
    }

    unsigned done = eth->tx_status;
    printf("lwipdemo tx len=%d status=%x\n", p->tot_len, done);

    if (!timeout || !(done & ETH_TX_STATUS_DONE) ||
        (done & (ETH_TX_STATUS_BUSY | ETH_TX_STATUS_OVERFLOW))) {
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

static void udp_recv_cb(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    (void)arg;

    if (p == 0) {
        return;
    }

    printf("lwipdemo udp len=%d port=%d data=", p->tot_len, port);
    for (struct pbuf *q = p; q != 0; q = q->next) {
        unsigned char *src = (unsigned char *)q->payload;
        for (u16_t i = 0; i < q->len; i++) {
            printf("%x", src[i]);
        }
    }
    printf("\n");

    err_t err = udp_sendto(upcb, p, addr, port);
    printf("lwipdemo udp echo=%d\n", err);
    udp_seen++;
    pbuf_free(p);
}

static err_t udp_server_init(void)
{
    if (udp_listener != 0) {
        udp_remove(udp_listener);
        udp_listener = 0;
    }

    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == 0) {
        return ERR_MEM;
    }

    err_t err = udp_bind(pcb, IP_ANY_TYPE, runtime_config.udp_port);
    if (err != ERR_OK) {
        udp_remove(pcb);
        return err;
    }

    udp_recv(pcb, udp_recv_cb, 0);
    udp_listener = pcb;
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

static err_t apply_runtime_config(void)
{
    if (!netif_configured) {
        return ERR_OK;
    }

    apply_netif_config();
    err_t err = udp_server_init();
    printf("lwipdemo cfg apply=%d port=%d\n", err, runtime_config.udp_port);
    print_config();
    return err;
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
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }
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

static void copy_bytes(unsigned char *dst, const unsigned char *src, unsigned len)
{
    for (unsigned i = 0; i < len; i++) {
        dst[i] = src[i];
    }
}

static void handle_uart_command(char *line)
{
    line = skip_spaces(line);
    char *arg = command_arg(line);
    unsigned char parsed_bytes[6];
    unsigned parsed_port = 0;

    if (*line == 0) {
        return;
    }

    if (command_is(line, "show") || command_is(line, "cfg")) {
        print_config();
        return;
    }

    if (command_is(line, "apply")) {
        (void)apply_runtime_config();
        return;
    }

    if (command_is(line, "mac")) {
        if (!parse_mac(arg, parsed_bytes)) {
            printf("lwipdemo cfg bad mac\n");
            return;
        }
        copy_bytes(runtime_config.mac, parsed_bytes, 6);
        (void)apply_runtime_config();
        return;
    }

    if (command_is(line, "ip")) {
        if (!parse_ipv4(arg, parsed_bytes)) {
            printf("lwipdemo cfg bad ip\n");
            return;
        }
        copy_bytes(runtime_config.ip, parsed_bytes, 4);
        (void)apply_runtime_config();
        return;
    }

    if (command_is(line, "mask") || command_is(line, "netmask")) {
        if (!parse_ipv4(arg, parsed_bytes)) {
            printf("lwipdemo cfg bad mask\n");
            return;
        }
        copy_bytes(runtime_config.netmask, parsed_bytes, 4);
        (void)apply_runtime_config();
        return;
    }

    if (command_is(line, "gw") || command_is(line, "gateway")) {
        if (!parse_ipv4(arg, parsed_bytes)) {
            printf("lwipdemo cfg bad gw\n");
            return;
        }
        copy_bytes(runtime_config.gateway, parsed_bytes, 4);
        (void)apply_runtime_config();
        return;
    }

    if (command_is(line, "port")) {
        if (!parse_dec(&arg, &parsed_port) || parsed_port > 65535u ||
            *skip_spaces(arg) != 0) {
            printf("lwipdemo cfg bad port\n");
            return;
        }
        runtime_config.udp_port = parsed_port;
        (void)apply_runtime_config();
        return;
    }

    printf("lwipdemo cfg unknown\n");
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
        } else if (uart_line_len < (LWIPDEMO_UART_LINE_MAX - 1u)) {
            uart_line[uart_line_len++] = c;
        } else {
            uart_line_len = 0;
            printf("lwipdemo cfg line too long\n");
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
    printf("lwipdemo rx len=%d status=%x\n", len, status);

    if ((status & (ETH_STATUS_RX_OVERFLOW | ETH_STATUS_RX_DROPPED)) ||
        (len == 0) || (len > LWIPDEMO_MAX_FRAME)) {
        drain_rx_frame(len);
        eth->rx_ctrl = ETH_RX_CTRL_CLEAR_FLAGS;
        return ERR_BUF;
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, (u16_t)len, PBUF_POOL);
    if (p == 0) {
        printf("lwipdemo rx no pbuf\n");
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
        printf("lwipdemo input err=%d\n", err);
        pbuf_free(p);
    }

    if (eth->status & (ETH_STATUS_RX_OVERFLOW | ETH_STATUS_RX_DROPPED)) {
        printf("lwipdemo rx flags=%x\n", eth->status);
        eth->rx_ctrl = ETH_RX_CTRL_CLEAR_FLAGS;
    }

    return err;
}

int main(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;
    unsigned timeout = 50000000;
    unsigned ok_reported = 0;
    unsigned timeout_reported = 0;

    printf("lwipdemo start\n");

    lwip_init();

    print_config();

    ip4_from_config(&ipaddr, runtime_config.ip);
    ip4_from_config(&netmask, runtime_config.netmask);
    ip4_from_config(&gw, runtime_config.gateway);

    if (netif_add(&fpga_netif, &ipaddr, &netmask, &gw, 0,
                  darketh_netif_init, ethernet_input) == 0) {
        printf("lwipdemo netif fail\n>");
        return 1;
    }
    netif_configured = 1;
    apply_netif_config();

    netif_set_default(&fpga_netif);
    netif_set_up(&fpga_netif);
    netif_set_link_up(&fpga_netif);

    err_t err = udp_server_init();
    printf("lwipdemo udp bind=%d port=%d\n", err, runtime_config.udp_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    while (1) {
        poll_uart_config();
        poll_rx_frame();
        sys_check_timeouts();

        if (udp_seen && !ok_reported) {
            if (!(eth->status & ETH_STATUS_RX_READY)) {
                printf("lwipdemo rx not ready=%x\n>", eth->status);
            } else {
                printf("lwipdemo ok\n>");
            }
            ok_reported = 1;
        }

        if (!udp_seen && timeout != 0) {
            timeout--;
        }

        if (!udp_seen && timeout == 0 && !timeout_reported) {
            printf("lwipdemo timeout status=%x\n>", eth->status);
            timeout_reported = 1;
        }
    }
}
