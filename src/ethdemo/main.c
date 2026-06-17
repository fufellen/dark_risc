#include <io.h>
#include <stdio.h>

#define DARKETH_BASE 0x80000000u

#define ETH_STATUS_RX_AVAILABLE 0x00000001u
#define ETH_STATUS_RX_OVERFLOW  0x00000002u
#define ETH_STATUS_RX_DROPPED   0x00000004u
#define ETH_STATUS_RX_READY     0x00000100u

#define ETH_TX_STATUS_READY     0x00000001u
#define ETH_TX_STATUS_BUSY      0x00000002u
#define ETH_TX_STATUS_OVERFLOW  0x00000004u
#define ETH_TX_STATUS_DONE      0x00000008u
#define ETH_TX_STATUS_WRITTEN   0x00000010u

#define ETH_TX_CTRL_START       0x00000001u
#define ETH_TX_CTRL_ABORT       0x00000002u
#define ETH_TX_CTRL_CLEAR_FLAGS 0x00000004u

#define ETHDEMO_MAX_FRAME       64u

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
static unsigned char frame[ETHDEMO_MAX_FRAME];

int main(void)
{
    unsigned timeout = 1000000;

    printf("ethdemo start\n");

    while (!(eth->status & ETH_STATUS_RX_AVAILABLE) && timeout) {
        timeout--;
    }

    if (!timeout) {
        printf("ethdemo timeout status=%x\n", eth->status);
        printf(">");
        return 1;
    }

    unsigned len = eth->rx_len;
    if (len > ETHDEMO_MAX_FRAME) {
        printf("ethdemo frame too big len=%d\n", len);
        printf(">");
        return 1;
    }

    printf("ethdemo len=%d data=", len);

    for (unsigned i = 0; i < len; i++) {
        frame[i] = eth->rx_data & 0xff;
        printf("%x", frame[i]);
    }

    printf("\nethdemo status=%x\n", eth->status);

    if (!(eth->status & ETH_STATUS_RX_READY)) {
        printf("ethdemo not ready\n");
        printf(">");
        return 1;
    }

    if (eth->status & (ETH_STATUS_RX_OVERFLOW | ETH_STATUS_RX_DROPPED)) {
        printf("ethdemo flags=%x\n", eth->status);
        printf(">");
        return 1;
    }

    eth->tx_ctrl = ETH_TX_CTRL_CLEAR_FLAGS | ETH_TX_CTRL_ABORT;
    eth->tx_len = len;

    for (unsigned i = 0; i < len; i++) {
        eth->tx_data = frame[i];
    }

    unsigned tx_status = eth->tx_status;
    printf("ethdemo tx staged=%x\n", tx_status);

    if ((tx_status & (ETH_TX_STATUS_READY | ETH_TX_STATUS_WRITTEN)) !=
        (ETH_TX_STATUS_READY | ETH_TX_STATUS_WRITTEN)) {
        printf("ethdemo tx not staged\n");
        printf(">");
        return 1;
    }

    eth->tx_ctrl = ETH_TX_CTRL_START;
    timeout = 1000000;

    while (!(eth->tx_status & ETH_TX_STATUS_DONE) && timeout) {
        timeout--;
    }

    tx_status = eth->tx_status;
    printf("ethdemo tx status=%x\n", tx_status);

    if (!timeout || !(tx_status & ETH_TX_STATUS_DONE)) {
        printf("ethdemo tx timeout\n");
        printf(">");
        return 1;
    }

    if (tx_status & (ETH_TX_STATUS_BUSY | ETH_TX_STATUS_OVERFLOW)) {
        printf("ethdemo tx flags=%x\n", tx_status);
        printf(">");
        return 1;
    }

    eth->tx_ctrl = ETH_TX_CTRL_CLEAR_FLAGS;

    printf("ethdemo ok\n>");
    return 0;
}
