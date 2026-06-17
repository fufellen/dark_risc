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

#define LWIPDEMO_MAX_FRAME      1518u

struct DARKETH {
    unsigned status;
    unsigned rx_len;
    unsigned rx_data;
    unsigned rx_ctrl;
    unsigned tx_status;
    unsigned tx_len;
    unsigned tx_data;
    unsigned tx_ctrl;
};

static volatile struct DARKETH *eth = (volatile struct DARKETH *)DARKETH_BASE;
static struct netif fpga_netif;
static volatile unsigned udp_seen;

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
    return ERR_OK;
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
    unsigned timeout = 1000000;

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

    netif_set_default(&fpga_netif);
    netif_set_up(&fpga_netif);
    netif_set_link_up(&fpga_netif);

    err_t err = udp_server_init();
    printf("lwipdemo udp bind=%d port=%d\n", err, runtime_config.udp_port);
    if (err != ERR_OK) {
        printf(">");
        return 1;
    }

    while (!udp_seen && timeout) {
        poll_rx_frame();
        sys_check_timeouts();
        timeout--;
    }

    if (!udp_seen) {
        printf("lwipdemo timeout status=%x\n>", eth->status);
        return 1;
    }

    if (!(eth->status & ETH_STATUS_RX_READY)) {
        printf("lwipdemo rx not ready=%x\n>", eth->status);
        return 1;
    }

    printf("lwipdemo ok\n>");
    return 0;
}
