#include <io.h>
#include <stdio.h>

#define DARKDDR3_BASE 0x80000000u

#define DDR3_STATUS_INIT_DONE       0x00000001u
#define DDR3_STATUS_WRITE_LEVEL     0x00000002u
#define DDR3_STATUS_READ_CALIB      0x00000004u
#define DDR3_STATUS_DDR_BUSY        0x00000008u
#define DDR3_STATUS_OP_BUSY         0x00000010u
#define DDR3_STATUS_OP_DONE         0x00000020u
#define DDR3_STATUS_OP_ERROR        0x00000040u
#define DDR3_STATUS_READY_FOR_CMD   0x00000100u

#define DDR3_CTRL_START_READ        0x00000001u
#define DDR3_CTRL_START_WRITE       0x00000002u
#define DDR3_CTRL_START_REFRESH     0x00000004u
#define DDR3_CTRL_CLEAR_DONE        0x00000100u
#define DDR3_CTRL_CLEAR_ERROR       0x00000200u

struct DARKDDR3 {
    unsigned status;
    unsigned addr;
    unsigned wdata;
    unsigned rdata;
    unsigned ctrl;
    unsigned refresh_count;
};

static volatile struct DARKDDR3 *ddr3 = (volatile struct DARKDDR3 *)DARKDDR3_BASE;

static int wait_status(unsigned mask, unsigned expected, unsigned timeout, const char *what)
{
    while (((ddr3->status & mask) != expected) && timeout) {
        timeout--;
    }

    if (!timeout) {
        printf("ddr3demo timeout %s status=%x\n", what, ddr3->status);
        return -1;
    }

    return 0;
}

static int ddr3_clear_done(const char *what)
{
    ddr3->ctrl = DDR3_CTRL_CLEAR_DONE | DDR3_CTRL_CLEAR_ERROR;

    return wait_status(DDR3_STATUS_OP_DONE | DDR3_STATUS_OP_ERROR,
                       0,
                       1000000,
                       what);
}

static int ddr3_write32(unsigned addr, unsigned data)
{
    if (wait_status(DDR3_STATUS_READY_FOR_CMD,
                    DDR3_STATUS_READY_FOR_CMD,
                    1000000,
                    "write-ready")) {
        return -1;
    }

    if (ddr3_clear_done("write-clear")) {
        return -1;
    }

    ddr3->addr = addr;
    ddr3->wdata = data;
    ddr3->ctrl = DDR3_CTRL_START_WRITE;

    if (wait_status(DDR3_STATUS_OP_DONE | DDR3_STATUS_OP_BUSY,
                    DDR3_STATUS_OP_DONE,
                    1000000,
                    "write-done")) {
        return -1;
    }

    if (ddr3->status & DDR3_STATUS_OP_ERROR) {
        printf("ddr3demo write error status=%x\n", ddr3->status);
        return -1;
    }

    return 0;
}

static int ddr3_read32(unsigned addr, unsigned *data)
{
    if (wait_status(DDR3_STATUS_READY_FOR_CMD,
                    DDR3_STATUS_READY_FOR_CMD,
                    1000000,
                    "read-ready")) {
        return -1;
    }

    if (ddr3_clear_done("read-clear")) {
        return -1;
    }

    ddr3->addr = addr;
    ddr3->ctrl = DDR3_CTRL_START_READ;

    if (wait_status(DDR3_STATUS_OP_DONE | DDR3_STATUS_OP_BUSY,
                    DDR3_STATUS_OP_DONE,
                    1000000,
                    "read-done")) {
        return -1;
    }

    if (ddr3->status & DDR3_STATUS_OP_ERROR) {
        printf("ddr3demo read error status=%x\n", ddr3->status);
        return -1;
    }

    *data = ddr3->rdata;
    return 0;
}

int main(void)
{
    unsigned data = 0;
    unsigned refresh_before = 0;
    unsigned refresh_after = 0;

    printf("ddr3demo start\n");

    if (wait_status(DDR3_STATUS_INIT_DONE | DDR3_STATUS_READY_FOR_CMD,
                    DDR3_STATUS_INIT_DONE | DDR3_STATUS_READY_FOR_CMD,
                    1000000,
                    "init")) {
        printf(">");
        return 1;
    }

    printf("ddr3demo status=%x\n", ddr3->status);

    if (ddr3_write32(0x10, 0x55667788u)) {
        printf(">");
        return 1;
    }

    if (ddr3_read32(0x10, &data)) {
        printf(">");
        return 1;
    }

    printf("ddr3demo rw addr=10 data=%x\n", data);
    if (data != 0x55667788u) {
        printf("ddr3demo mismatch expected=55667788 actual=%x\n", data);
        printf(">");
        return 1;
    }

    if (ddr3_write32(0x40, 0xa5a55a5au)) {
        printf(">");
        return 1;
    }

    if (ddr3_read32(0x40, &data)) {
        printf(">");
        return 1;
    }

    printf("ddr3demo rw addr=40 data=%x\n", data);
    if (data != 0xa5a55a5au) {
        printf("ddr3demo mismatch expected=a5a55a5a actual=%x\n", data);
        printf(">");
        return 1;
    }

    refresh_before = ddr3->refresh_count;
    if (ddr3_clear_done("refresh-clear")) {
        printf(">");
        return 1;
    }

    ddr3->ctrl = DDR3_CTRL_START_REFRESH;

    if (wait_status(DDR3_STATUS_READY_FOR_CMD,
                    DDR3_STATUS_READY_FOR_CMD,
                    1000000,
                    "refresh")) {
        printf(">");
        return 1;
    }

    refresh_after = ddr3->refresh_count;
    printf("ddr3demo refresh %x to %x\n", refresh_before, refresh_after);

    if (refresh_after == refresh_before) {
        printf("ddr3demo refresh did not advance\n");
        printf(">");
        return 1;
    }

    printf("ddr3demo ok\n>");
    return 0;
}
