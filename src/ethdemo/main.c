#include <io.h>
#include <stdio.h>

#define DARKETH_BASE 0x80000000u

#define ETH_STATUS_RX_AVAILABLE 0x00000001u
#define ETH_STATUS_RX_OVERFLOW  0x00000002u
#define ETH_STATUS_RX_DROPPED   0x00000004u
#define ETH_STATUS_RX_READY     0x00000100u

struct DARKETH {
    unsigned status;
    unsigned rx_len;
    unsigned rx_data;
    unsigned rx_ctrl;
};

static volatile struct DARKETH *eth = (volatile struct DARKETH *)DARKETH_BASE;

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
    printf("ethdemo len=%d data=", len);

    for (unsigned i = 0; i < len; i++) {
        printf("%x", eth->rx_data & 0xff);
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

    printf("ethdemo ok\n>");
    return 0;
}
