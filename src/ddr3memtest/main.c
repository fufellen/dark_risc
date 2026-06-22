#include <io.h>
#include <stdio.h>

#ifndef DARKDDR3_BASE
#define DARKDDR3_BASE 0x80000000u
#endif

#ifndef DDR3MEMTEST_WORDS
#define DDR3MEMTEST_WORDS 1024
#endif

#ifndef DDR3MEMTEST_RANDOM_OPS
#define DDR3MEMTEST_RANDOM_OPS 256
#endif

#ifndef DDR3MEMTEST_MAX_ADDR_BIT
#define DDR3MEMTEST_MAX_ADDR_BIT 25
#endif

#ifndef DDR3MEMTEST_RETENTION_REFRESHES
#define DDR3MEMTEST_RETENTION_REFRESHES 64
#endif

#ifndef DDR3MEMTEST_BASE_ADDR
#define DDR3MEMTEST_BASE_ADDR 0x30u
#endif

#ifndef DDR3MEMTEST_LOW_PROBE
#define DDR3MEMTEST_LOW_PROBE 1
#endif

#if (DDR3MEMTEST_WORDS & (DDR3MEMTEST_WORDS - 1)) != 0
#error DDR3MEMTEST_WORDS must be a power of two
#endif

#if DDR3MEMTEST_RANDOM_OPS > DDR3MEMTEST_WORDS
#error DDR3MEMTEST_RANDOM_OPS must not exceed DDR3MEMTEST_WORDS
#endif

#if (DDR3MEMTEST_BASE_ADDR & 1) != 0
#error DDR3MEMTEST_BASE_ADDR must be an even 16-bit DDR word address
#endif

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

#define WORD_MASK                   (DDR3MEMTEST_WORDS - 1u)

struct DARKDDR3 {
    unsigned status;
    unsigned addr;
    unsigned wdata;
    unsigned rdata;
    unsigned ctrl;
    unsigned refresh_count;
};

static volatile struct DARKDDR3 *ddr3 = (volatile struct DARKDDR3 *)DARKDDR3_BASE;
static unsigned expected[DDR3MEMTEST_WORDS];

static unsigned word_addr(unsigned word)
{
    return DDR3MEMTEST_BASE_ADDR + (word << 1);
}

static unsigned xorshift32(unsigned value)
{
    if (!value) {
        value = 0x1u;
    }

    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    return value;
}

static unsigned pattern_for_word(unsigned word, unsigned salt)
{
    unsigned value = word ^ salt ^ 0x9e3779b9u;
    value = xorshift32(value);
    value ^= word << 16;
    value ^= word >> 3;
    return value;
}

static int wait_status(unsigned mask, unsigned expected_status, unsigned timeout, const char *what)
{
    while (((ddr3->status & mask) != expected_status) && timeout) {
        timeout--;
    }

    if (!timeout) {
        printf("ddr3memtest timeout %s status=%x\n", what, ddr3->status);
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
        printf("ddr3memtest write error addr=%x status=%x\n", addr, ddr3->status);
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
        printf("ddr3memtest read error addr=%x status=%x\n", addr, ddr3->status);
        return -1;
    }

    *data = ddr3->rdata;
    return 0;
}

static int check_word(const char *phase, unsigned addr, unsigned expected_value)
{
    unsigned actual = 0;

    if (ddr3_read32(addr, &actual)) {
        return -1;
    }

    if (actual != expected_value) {
        printf("ddr3memtest mismatch %s addr=%x expected=%x actual=%x\n",
               phase, addr, expected_value, actual);
        return -1;
    }

    return 0;
}

static int phase_done(const char *phase)
{
    printf("ddr3memtest %s ok\n", phase);
    return 0;
}

static int probe_low_addresses(void)
{
#if DDR3MEMTEST_LOW_PROBE
    static const unsigned probe_addrs[] = {
        0x00u, 0x02u, 0x04u, 0x08u,
        0x10u, 0x20u, DDR3MEMTEST_BASE_ADDR, 0x40u
    };
    unsigned i;
    unsigned data;
    unsigned actual;
    unsigned warnings = 0;

    for (i = 0; i < (sizeof(probe_addrs) / sizeof(probe_addrs[0])); i++) {
        data = pattern_for_word(probe_addrs[i], 0x10ad0000u ^ i);
        actual = 0;

        if (ddr3_write32(probe_addrs[i], data) ||
            ddr3_read32(probe_addrs[i], &actual)) {
            return -1;
        }

        if (actual != data) {
            printf("ddr3memtest low-probe warn addr=%x expected=%x actual=%x\n",
                   probe_addrs[i], data, actual);
            warnings++;
        }
    }

    printf("ddr3memtest low-probe warnings=%x\n", warnings);
#endif
    return 0;
}

static int test_data_bus(void)
{
    unsigned bit;
    unsigned data;

    for (bit = 0; bit < 32; bit++) {
        data = 1u << bit;
        if (ddr3_write32(word_addr(0), data) ||
            check_word("data-one", word_addr(0), data)) {
            return -1;
        }

        data = ~data;
        if (ddr3_write32(word_addr(0), data) ||
            check_word("data-zero", word_addr(0), data)) {
            return -1;
        }
    }

    return phase_done("data-bus");
}

static int test_address_bus(void)
{
    unsigned bit;
    unsigned base = DDR3MEMTEST_BASE_ADDR;
    unsigned addr;
    unsigned data;

    if (ddr3_write32(base, 0xaaaaaaaau)) {
        return -1;
    }

    for (bit = 1; bit <= DDR3MEMTEST_MAX_ADDR_BIT; bit++) {
        addr = base ^ (1u << bit);
        data = pattern_for_word(addr, 0x13572468u);

        if (ddr3_write32(addr, data) ||
            check_word("addr-base", base, 0xaaaaaaaau)) {
            return -1;
        }
    }

    for (bit = 1; bit <= DDR3MEMTEST_MAX_ADDR_BIT; bit++) {
        addr = base ^ (1u << bit);
        data = pattern_for_word(addr, 0x13572468u);

        if (check_word("addr-line", addr, data)) {
            return -1;
        }
    }

    return phase_done("address-bus");
}

static int test_sparse_boundaries(void)
{
    unsigned bit;
    unsigned boundary;
    unsigned addr;
    unsigned data;

    for (bit = 4; bit <= DDR3MEMTEST_MAX_ADDR_BIT; bit++) {
        boundary = 1u << bit;

        addr = boundary - 2u;
        data = pattern_for_word(addr, 0x0b000001u ^ bit);
        if (ddr3_write32(addr, data) || check_word("boundary-before", addr, data)) {
            return -1;
        }

        addr = boundary;
        data = pattern_for_word(addr, 0x0b000002u ^ bit);
        if (ddr3_write32(addr, data) || check_word("boundary-at", addr, data)) {
            return -1;
        }

        addr = boundary + 2u;
        data = pattern_for_word(addr, 0x0b000003u ^ bit);
        if (ddr3_write32(addr, data) || check_word("boundary-after", addr, data)) {
            return -1;
        }
    }

    return phase_done("sparse-boundary");
}

static int test_read_after_write(void)
{
    unsigned i;
    unsigned word = 0;
    unsigned data = 0x12345678u;

    for (i = 0; i < 64; i++) {
        word = (word + 29u) & WORD_MASK;
        data = xorshift32(data ^ i);

        if (ddr3_write32(word_addr(word), data) ||
            check_word("raw", word_addr(word), data)) {
            return -1;
        }
    }

    return phase_done("read-after-write");
}

static int sweep_address_data(void)
{
    unsigned i;
    unsigned data;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = pattern_for_word(i, 0x01010101u);
        if (ddr3_write32(word_addr(i), data)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = pattern_for_word(i, 0x01010101u);
        if (check_word("addr-data", word_addr(i), data)) {
            return -1;
        }
    }

    return phase_done("sweep-address");
}

static int sweep_inverted_address(void)
{
    unsigned i;
    unsigned data;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = ~pattern_for_word(i, 0x10203040u);
        if (ddr3_write32(word_addr(i), data)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = ~pattern_for_word(i, 0x10203040u);
        if (check_word("inv-address", word_addr(i), data)) {
            return -1;
        }
    }

    return phase_done("sweep-invert");
}

static int sweep_checkerboard(void)
{
    unsigned i;
    unsigned data;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = (i & 1u) ? 0x55555555u : 0xaaaaaaaau;
        if (ddr3_write32(word_addr(i), data)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = (i & 1u) ? 0x55555555u : 0xaaaaaaaau;
        if (check_word("checker", word_addr(i), data)) {
            return -1;
        }
    }

    return phase_done("checkerboard");
}

static int sweep_prbs(void)
{
    unsigned i;
    unsigned data = 0x1aceb00cu;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = xorshift32(data);
        if (ddr3_write32(word_addr(i), data ^ i)) {
            return -1;
        }
    }

    data = 0x1aceb00cu;
    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        data = xorshift32(data);
        if (check_word("prbs", word_addr(i), data ^ i)) {
            return -1;
        }
    }

    return phase_done("prbs");
}

static int test_march_mats(void)
{
    unsigned i;
    unsigned idx;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        if (ddr3_write32(word_addr(i), 0x00000000u)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        if (check_word("march-r0-up", word_addr(i), 0x00000000u) ||
            ddr3_write32(word_addr(i), 0xffffffffu)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        if (check_word("march-r1-up", word_addr(i), 0xffffffffu) ||
            ddr3_write32(word_addr(i), 0x00000000u)) {
            return -1;
        }
    }

    for (i = DDR3MEMTEST_WORDS; i != 0; i--) {
        idx = i - 1u;
        if (check_word("march-r0-down", word_addr(idx), 0x00000000u) ||
            ddr3_write32(word_addr(idx), 0x55555555u)) {
            return -1;
        }
    }

    for (i = DDR3MEMTEST_WORDS; i != 0; i--) {
        idx = i - 1u;
        if (check_word("march-r55-down", word_addr(idx), 0x55555555u) ||
            ddr3_write32(word_addr(idx), 0xaaaaaaaau)) {
            return -1;
        }
    }

    for (i = DDR3MEMTEST_WORDS; i != 0; i--) {
        idx = i - 1u;
        if (check_word("march-raa-down", word_addr(idx), 0xaaaaaaaau) ||
            ddr3_write32(word_addr(idx), 0x00000000u)) {
            return -1;
        }
    }

    return phase_done("march-mats");
}

static int test_stride_access(void)
{
    static const unsigned strides[] = {1u, 3u, 5u, 17u};
    unsigned stride_idx;
    unsigned step;
    unsigned word;
    unsigned data;

    for (stride_idx = 0; stride_idx < (sizeof(strides) / sizeof(strides[0])); stride_idx++) {
        step = strides[stride_idx];
        word = step & WORD_MASK;

        do {
            data = pattern_for_word(word, 0x5a000000u ^ step);
            if (ddr3_write32(word_addr(word), data)) {
                return -1;
            }
            word = (word + step) & WORD_MASK;
        } while (word != (step & WORD_MASK));

        word = step & WORD_MASK;
        do {
            data = pattern_for_word(word, 0x5a000000u ^ step);
            if (check_word("stride", word_addr(word), data)) {
                return -1;
            }
            word = (word + step) & WORD_MASK;
        } while (word != (step & WORD_MASK));
    }

    return phase_done("stride");
}

static int test_random_access(void)
{
    unsigned i;
    unsigned word;
    unsigned data = 0x31415926u;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        expected[i] = pattern_for_word(i, 0x2468ace0u);
        if (ddr3_write32(word_addr(i), expected[i])) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_RANDOM_OPS; i++) {
        data = xorshift32(data ^ i);
        word = data & WORD_MASK;
        data = xorshift32(data ^ 0xa5a5a5a5u);
        expected[word] = data;

        if (ddr3_write32(word_addr(word), data) ||
            check_word("random-raw", word_addr(word), data)) {
            return -1;
        }
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        if (check_word("random-final", word_addr(i), expected[i])) {
            return -1;
        }
    }

    return phase_done("random");
}

static int read_byte_model(unsigned byte_addr, unsigned *value)
{
    unsigned word;
    unsigned shift;
    unsigned data;

    word = byte_addr >> 2;
    shift = (byte_addr & 3u) << 3;

    if (ddr3_read32(word_addr(word), &data)) {
        return -1;
    }

    *value = (data >> shift) & 0xffu;
    return 0;
}

static int write_byte_model(unsigned byte_addr, unsigned value)
{
    unsigned word;
    unsigned shift;
    unsigned data;
    unsigned mask;

    word = byte_addr >> 2;
    shift = (byte_addr & 3u) << 3;
    mask = 0xffu << shift;

    if (ddr3_read32(word_addr(word), &data)) {
        return -1;
    }

    data = (data & ~mask) | ((value & 0xffu) << shift);
    return ddr3_write32(word_addr(word), data);
}

static int read_packed_model(unsigned byte_addr, unsigned size, unsigned *value)
{
    unsigned i;
    unsigned byte_value;
    unsigned result = 0;

    for (i = 0; i < size; i++) {
        if (read_byte_model(byte_addr + i, &byte_value)) {
            return -1;
        }
        result |= byte_value << (i << 3);
    }

    *value = result;
    return 0;
}

static int write_packed_model(unsigned byte_addr, unsigned size, unsigned value)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        if (write_byte_model(byte_addr + i, value >> (i << 3))) {
            return -1;
        }
    }

    return 0;
}

static int test_subword_model(void)
{
    unsigned base_word = DDR3MEMTEST_WORDS - 16u;
    unsigned base_byte = base_word << 2;
    unsigned i;
    unsigned offset;
    unsigned data;
    unsigned expected_value;

    for (i = 0; i < 16; i++) {
        if (ddr3_write32(word_addr(base_word + i), 0x00000000u)) {
            return -1;
        }
    }

    for (offset = 0; offset < 4; offset++) {
        expected_value = 0xa0u | offset;
        if (write_packed_model(base_byte + offset, 1, expected_value) ||
            read_packed_model(base_byte + offset, 1, &data) ||
            data != expected_value) {
            printf("ddr3memtest mismatch subword-byte off=%x expected=%x actual=%x\n",
                   offset, expected_value, data);
            return -1;
        }
    }

    for (offset = 0; offset < 4; offset++) {
        expected_value = 0xb100u | offset;
        if (write_packed_model(base_byte + 16u + offset, 2, expected_value) ||
            read_packed_model(base_byte + 16u + offset, 2, &data) ||
            data != expected_value) {
            printf("ddr3memtest mismatch subword-half off=%x expected=%x actual=%x\n",
                   offset, expected_value, data);
            return -1;
        }
    }

    for (offset = 0; offset < 4; offset++) {
        expected_value = 0xc0def00du ^ offset;
        if (write_packed_model(base_byte + 32u + offset, 4, expected_value) ||
            read_packed_model(base_byte + 32u + offset, 4, &data) ||
            data != expected_value) {
            printf("ddr3memtest mismatch subword-word off=%x expected=%x actual=%x\n",
                   offset, expected_value, data);
            return -1;
        }
    }

    return phase_done("subword-model");
}

static int test_retention(void)
{
    unsigned i;
    unsigned before;
    unsigned after;
    unsigned wait_guard = 2000000u;

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        expected[i] = pattern_for_word(i, 0x0f0f0f0fu);
        if (ddr3_write32(word_addr(i), expected[i])) {
            return -1;
        }
    }

    before = ddr3->refresh_count;
    after = before;
    while (((after - before) < DDR3MEMTEST_RETENTION_REFRESHES) && wait_guard) {
        after = ddr3->refresh_count;
        wait_guard--;
    }

    if (!wait_guard) {
        printf("ddr3memtest timeout retention-refresh before=%x after=%x\n", before, after);
        return -1;
    }

    for (i = 0; i < DDR3MEMTEST_WORDS; i++) {
        if (check_word("retention", word_addr(i), expected[i])) {
            return -1;
        }
    }

    printf("ddr3memtest retention refresh %x to %x\n", before, after);
    return phase_done("retention");
}

int main(void)
{
    printf("ddr3memtest start words=%x maxbit=%x random=%x\n",
           DDR3MEMTEST_WORDS,
           DDR3MEMTEST_MAX_ADDR_BIT,
           DDR3MEMTEST_RANDOM_OPS);
    printf("ddr3memtest base=%x\n", DDR3MEMTEST_BASE_ADDR);

    if (wait_status(DDR3_STATUS_INIT_DONE | DDR3_STATUS_READY_FOR_CMD,
                    DDR3_STATUS_INIT_DONE | DDR3_STATUS_READY_FOR_CMD,
                    1000000,
                    "init")) {
        printf(">");
        return 1;
    }

    printf("ddr3memtest status=%x\n", ddr3->status);

    if (probe_low_addresses() ||
        test_data_bus() ||
        test_address_bus() ||
        test_sparse_boundaries() ||
        test_read_after_write() ||
        sweep_address_data() ||
        sweep_inverted_address() ||
        sweep_checkerboard() ||
        sweep_prbs() ||
        test_march_mats() ||
        test_stride_access() ||
        test_random_access() ||
        test_subword_model() ||
        test_retention()) {
        printf(">");
        return 1;
    }

    printf("ddr3memtest ok\n>");
    return 0;
}
