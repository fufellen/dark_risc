/*
 * w5500demo — MSOP TCP напрямую через WIZnet W5500 (hardware sockets),
 * без LwIP/RMII. Плата R120M.BM2BF1X1: W5500 на SPI ПЛИС (перемычки J10..J15).
 *
 * Шина — общий SPIBB bit-bang (как FLASH в lwipdemo): MOSI=OPORT[0],
 * SCK=OPORT[1], CS флеша=OPORT[2], EN=OPORT[3], MISO=IPORT[6].
 * Доп. биты этой прошивки: CS W5500=OPORT[8], RST W5500=OPORT[9]
 * (в прослойке платы OPORT[8] -> WIZNET_CS_N, OPORT[9] -> WIZNET_RST_N,
 * SCK/MOSI — те же пины SPI, MISO мультиплексируется по CS).
 *
 * Симуляция: sim/w5500_model.sv + DARKSIMV_DEFINES
 * '+define+SPI +define+SPIBB +define+DARKWIZNET_SIM'.
 * Справочник чипа: docs/skills/fpga-dev/w5500.md (суперпроект verilog).
 */

#include <io.h>
#include <stdio.h>

/* биты OPORT/IPORT */
#define SPIBB_MOSI   0x00000001u
#define SPIBB_SCK    0x00000002u
#define SPIBB_FLASH_CSN 0x00000004u
#define SPIBB_EN     0x00000008u
#define SPIBB_MISO   0x00000040u /* IPORT */
#define W5500_CSN    0x00000100u
#define W5500_RSTN   0x00000200u

/* W5500: control byte = BSB[4:0]<<3 | RWB<<2 | OM[1:0] (OM=00 — VDM) */
#define W5500_BSB_COMMON   0u
#define W5500_BSB_S0_REG   1u
#define W5500_BSB_S0_TX    2u
#define W5500_RWB_WRITE    0x04u

/* общие регистры */
#define W5500_GAR      0x0001u
#define W5500_SUBR     0x0005u
#define W5500_SHAR     0x0009u
#define W5500_SIPR     0x000Fu
#define W5500_VERSIONR 0x0039u

/* регистры сокета */
#define SN_MR     0x0000u
#define SN_CR     0x0001u
#define SN_IR     0x0002u
#define SN_SR     0x0003u
#define SN_PORT   0x0004u
#define SN_TX_FSR 0x0020u
#define SN_TX_WR  0x0024u

#define SN_MR_TCP     0x01u
#define SN_CR_OPEN    0x01u
#define SN_CR_LISTEN  0x02u
#define SN_CR_SEND    0x20u
#define SN_IR_SENDOK  0x10u
#define SOCK_INIT        0x13u
#define SOCK_LISTEN      0x14u
#define SOCK_ESTABLISHED 0x17u

#define MSOP_TCP_PORT 50100u
#define W5500_TXBUF_SIZE 2048u

/* MSOP — формат идентичен lwipdemo (758 байт при 180 точках) */
#ifndef W5500DEMO_MSOP_POINTS
#define W5500DEMO_MSOP_POINTS 180u
#endif
#define MSOP_DISTANCE_BYTES 2u
#define MSOP_ECHO_COUNT     2u
#define MSOP_ECHO_MODE      3u
#define MSOP_PACKET_LEN (36u + W5500DEMO_MSOP_POINTS * \
                         MSOP_ECHO_COUNT * MSOP_DISTANCE_BYTES + 2u)
#ifndef W5500DEMO_FRAMES
#define W5500DEMO_FRAMES 2u
#endif

static unsigned char msop_buf[MSOP_PACKET_LEN];
static unsigned msop_frame_num = 0;

/* --- bit-bang: состояние CS/RST W5500 входит в каждый io->oport --- */
static unsigned w5500_cs_bit = W5500_CSN;   /* 1 = деселект */
static unsigned w5500_rst_bit = 0;          /* 0 = чип в резете */

static void bb_out(unsigned sck, unsigned mosi)
{
    unsigned v = SPIBB_EN | SPIBB_FLASH_CSN | w5500_cs_bit | w5500_rst_bit;
    if (sck) {
        v |= SPIBB_SCK;
    }
    if (mosi) {
        v |= SPIBB_MOSI;
    }
    io->oport = v;
}

static void w5500_select(void)
{
    bb_out(0, 1);
    w5500_cs_bit = 0;
    bb_out(0, 1);
}

static void w5500_deselect(void)
{
    bb_out(0, 1);
    w5500_cs_bit = W5500_CSN;
    bb_out(0, 1);
}

static unsigned char w5500_xfer(unsigned char value)
{
    unsigned char rx = 0;

    /* идиом как в проверенном flash-драйвере lwipdemo: value & (1u << bit);
     * вариант (value >> bit) & 1 давал на DarkRISCV отрицание байта (-value) */
    for (int bit = 7; bit >= 0; bit--) {
        unsigned mosi = (value & (1u << bit)) ? 1u : 0u;
        bb_out(0, mosi);
        bb_out(1, mosi);
        rx = (unsigned char)((rx << 1) |
             ((io->iport & SPIBB_MISO) ? 1u : 0u));
    }
    return rx;
}

static void w5500_header(unsigned bsb, unsigned addr, unsigned write)
{
    w5500_xfer((unsigned char)(addr >> 8));
    w5500_xfer((unsigned char)(addr & 0xffu));
    w5500_xfer((unsigned char)((bsb << 3) | (write ? W5500_RWB_WRITE : 0u)));
}

static void w5500_write(unsigned bsb, unsigned addr,
                        const unsigned char *data, unsigned len)
{
    w5500_select();
    w5500_header(bsb, addr, 1);
    for (unsigned i = 0; i < len; i++) {
        w5500_xfer(data[i]);
    }
    w5500_deselect();
}

static void w5500_read(unsigned bsb, unsigned addr,
                       unsigned char *data, unsigned len)
{
    w5500_select();
    w5500_header(bsb, addr, 0);
    for (unsigned i = 0; i < len; i++) {
        data[i] = w5500_xfer(0x00u);
    }
    w5500_deselect();
}

static void w5500_wreg8(unsigned bsb, unsigned addr, unsigned value)
{
    unsigned char b = (unsigned char)value;
    w5500_write(bsb, addr, &b, 1);
}

static unsigned w5500_rreg8(unsigned bsb, unsigned addr)
{
    unsigned char b;
    w5500_read(bsb, addr, &b, 1);
    return b;
}

static void w5500_wreg16(unsigned bsb, unsigned addr, unsigned value)
{
    unsigned char b[2];
    b[0] = (unsigned char)(value >> 8);
    b[1] = (unsigned char)(value & 0xffu);
    w5500_write(bsb, addr, b, 2);
}

static unsigned w5500_rreg16(unsigned bsb, unsigned addr)
{
    unsigned char b[2];
    w5500_read(bsb, addr, b, 2);
    return ((unsigned)b[0] << 8) | b[1];
}

static void short_delay(unsigned loops)
{
    static volatile unsigned sink;
    for (unsigned i = 0; i < loops; i++) {
        sink++;
    }
}

/* --- MSOP: заголовок/точки/терминатор как в lwipdemo build_msop_packet --- */
static void write_le16(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
}

static void write_le32(unsigned char *dst, unsigned value)
{
    dst[0] = (unsigned char)(value & 0xffu);
    dst[1] = (unsigned char)((value >> 8) & 0xffu);
    dst[2] = (unsigned char)((value >> 16) & 0xffu);
    dst[3] = (unsigned char)((value >> 24) & 0xffu);
}

static unsigned build_msop_packet(unsigned char *out)
{
    unsigned pos = 0;
    unsigned now = io->timeus / 1000u;

    out[pos++] = 0xff;
    out[pos++] = 0xfe;
    out[pos++] = 1;
    out[pos++] = 1;
    out[pos++] = 0;
    write_le16(out + pos, msop_frame_num & 0xffffu); pos += 2;
    write_le16(out + pos, W5500DEMO_MSOP_POINTS); pos += 2;
    write_le32(out + pos, now / 1000u); pos += 4;
    write_le32(out + pos, (now % 1000u) * 1000u); pos += 4;
    write_le32(out + pos, 0); pos += 4;
    out[pos++] = 1;
    write_le32(out + pos, 250000u); pos += 4;
    write_le32(out + pos, 360000u); pos += 4;
    write_le16(out + pos, 2000u); pos += 2;
    out[pos++] = MSOP_DISTANCE_BYTES;
    out[pos++] = 0; /* intensity bytes */
    out[pos++] = MSOP_ECHO_MODE;
    out[pos++] = MSOP_ECHO_COUNT;

    for (unsigned i = 0; i < W5500DEMO_MSOP_POINTS; i++) {
        unsigned dist1 = 1000u + i;          /* синтетика вместо TDC */
        unsigned dist2 = 2000u + i;
        write_le16(out + pos, dist1); pos += 2;
        write_le16(out + pos, dist2); pos += 2;
    }

    out[pos++] = 0xff;
    out[pos++] = 0x9b;
    return pos;
}

/* ожидание значения регистра с ограничением попыток */
static int wait_sr(unsigned expected, unsigned tries)
{
    for (unsigned i = 0; i < tries; i++) {
        if (w5500_rreg8(W5500_BSB_S0_REG, SN_SR) == expected) {
            return 0;
        }
        short_delay(50);
    }
    return -1;
}

static int fail(const char *what)
{
    printf("w5500demo FAIL: %s\n", what);
    printf(">");
    return 1;
}

int main(void)
{
    /* без io->timer TIMEUS/sys_now стоят — грабли darkio (см. базу навыков) */
    io->timer = io->board_cm * 2000000u - 1u;
    printf("w5500demo start\n");

    /* аппаратный reset W5500 (на плате ~RST ещё и подтянут к GND) */
    w5500_rst_bit = 0;
    bb_out(0, 1);
    short_delay(200);
    w5500_rst_bit = W5500_RSTN;
    bb_out(0, 1);
    short_delay(500);

    unsigned ver = w5500_rreg8(W5500_BSB_COMMON, W5500_VERSIONR);
    printf("w5500 version=%x\n", ver);
    if (ver != 0x04u) {
        return fail("VERSIONR != 0x04");
    }

    /* сетевая конфигурация — как у лидара на шаренном свиче */
    {
        static const unsigned char mac[6] = {0x02, 0x20, 0x20, 0x20, 0x20, 0x01};
        static const unsigned char gw[4] = {192, 168, 2, 1};
        static const unsigned char mask[4] = {255, 255, 255, 0};
        static const unsigned char ip[4] = {192, 168, 2, 240};
        unsigned char chk[4];

        w5500_write(W5500_BSB_COMMON, W5500_SHAR, mac, 6);
        w5500_write(W5500_BSB_COMMON, W5500_GAR, gw, 4);
        w5500_write(W5500_BSB_COMMON, W5500_SUBR, mask, 4);
        w5500_write(W5500_BSB_COMMON, W5500_SIPR, ip, 4);

        w5500_read(W5500_BSB_COMMON, W5500_SIPR, chk, 4);
        for (unsigned i = 0; i < 4; i++) {
            if (chk[i] != ip[i]) {
                return fail("SIPR readback");
            }
        }
        printf("w5500 netcfg ok ip=%x.%x.%x.%x\n",
               chk[0], chk[1], chk[2], chk[3]);
    }

    /* socket 0: TCP-сервер MSOP на 50100 */
    w5500_wreg8(W5500_BSB_S0_REG, SN_MR, SN_MR_TCP);
    w5500_wreg16(W5500_BSB_S0_REG, SN_PORT, MSOP_TCP_PORT);
    w5500_wreg8(W5500_BSB_S0_REG, SN_CR, SN_CR_OPEN);
    if (wait_sr(SOCK_INIT, 100)) {
        return fail("SOCK_INIT");
    }
    printf("w5500 sock0 init\n");

    w5500_wreg8(W5500_BSB_S0_REG, SN_CR, SN_CR_LISTEN);
    if (wait_sr(SOCK_LISTEN, 100)) {
        return fail("SOCK_LISTEN");
    }
    printf("w5500 sock0 listen port=%x\n", MSOP_TCP_PORT);

    if (wait_sr(SOCK_ESTABLISHED, 100000)) {
        return fail("ESTABLISHED timeout");
    }
    printf("w5500 sock0 established\n");

    /* поток MSOP-кадров: TX-буфер -> SEND -> SEND_OK */
    for (unsigned n = 0; n < W5500DEMO_FRAMES; n++) {
        msop_frame_num = n;
        unsigned len = build_msop_packet(msop_buf);
        unsigned free_sz = w5500_rreg16(W5500_BSB_S0_REG, SN_TX_FSR);
        if (free_sz < len) {
            return fail("TX_FSR too small");
        }

        unsigned wr = w5500_rreg16(W5500_BSB_S0_REG, SN_TX_WR);
        w5500_write(W5500_BSB_S0_TX, wr, msop_buf, len);
        w5500_wreg16(W5500_BSB_S0_REG, SN_TX_WR, (wr + len) & 0xffffu);
        w5500_wreg8(W5500_BSB_S0_REG, SN_CR, SN_CR_SEND);

        unsigned ok = 0;
        for (unsigned i = 0; i < 10000; i++) {
            if (w5500_rreg8(W5500_BSB_S0_REG, SN_IR) & SN_IR_SENDOK) {
                ok = 1;
                break;
            }
            short_delay(20);
        }
        if (!ok) {
            return fail("SEND_OK timeout");
        }
        w5500_wreg8(W5500_BSB_S0_REG, SN_IR, SN_IR_SENDOK);
        printf("w5500 msop sent frame=%x len=%x\n", n, len);
    }

    printf("w5500demo ok\n");
    printf(">");
    return 0;
}
