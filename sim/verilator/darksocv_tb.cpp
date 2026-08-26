// Безголовый стенд DarkRISCV под Verilator.
//
// Зачем отдельно от sim/cosim.mk: тот стенд собран под Linux и требует ImGui,
// SDL2, GLEW и GLFW ради интерактивного окна. Здесь проверяется другое — что
// наш SoC вообще проходит через Verilator и выполняет прошивку. Для такой
// проверки окно только мешает, а зависимостей на Windows нет.
//
// Вывод UART печатает сам RTL: darkuart.v под `SIMULATION` делает
// $write("%c", ...), поэтому декодировать последовательный поток здесь не надо.
//
// Образ прошивки грузится из ../src/darksocv.mem — путь зашит в darkram.v,
// поэтому двоичный файл запускается из каталога sim/.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Vdarksocv.h"
#include "verilated.h"

// Волны подключаются только в сборке с --trace: без него исполняемого кода
// VCD в модели нет, и безусловная ссылка на него ломает компоновку.
#if VM_TRACE
#include "verilated_vcd_c.h"
#endif

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    // Сколько тактов крутить и писать ли волны — из командной строки, чтобы
    // одну и ту же сборку можно было гонять и коротко, и с дампом.
    long long max_cycles = 2000000;
    const char* vcd_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (!std::strncmp(argv[i], "+cycles=", 8)) {
            max_cycles = std::atoll(argv[i] + 8);
        } else if (!std::strncmp(argv[i], "+vcd=", 5)) {
            vcd_path = argv[i] + 5;
        }
    }

    Vdarksocv* dut = new Vdarksocv;
#if VM_TRACE
    VerilatedVcdC* vcd = nullptr;
    if (vcd_path) {
        Verilated::traceEverOn(true);
        vcd = new VerilatedVcdC;
        dut->trace(vcd, 99);
        vcd->open(vcd_path);
    }
#else
    if (vcd_path) {
        std::fprintf(stderr, "[стенд] сборка без --trace: волны не пишутся\n");
    }
#endif

    dut->XCLK = 0;
    dut->XRES = 1;
    dut->UART_RXD = 1;      // линия приёма в покое высокая
    dut->IPORT = 0;

    // Сброс держим 64 такта: darksocv поднимает свой внутренний сброс сам,
    // внешнего импульса достаточно короткого.
    const long long reset_cycles = 64;

    long long cycle = 0;
    for (; cycle < max_cycles; ++cycle) {
        if (cycle == reset_cycles) {
            dut->XRES = 0;
        }

        dut->XCLK = 0;
        dut->eval();
#if VM_TRACE
        if (vcd) vcd->dump(static_cast<uint64_t>(2 * cycle));
#endif

        dut->XCLK = 1;
        dut->eval();
#if VM_TRACE
        if (vcd) vcd->dump(static_cast<uint64_t>(2 * cycle + 1));
#endif

        if (Verilated::gotFinish()) {
            break;
        }
    }

#if VM_TRACE
    if (vcd) {
        vcd->close();
        delete vcd;
    }
#endif
    dut->final();
    delete dut;

    std::fprintf(stderr, "\n[стенд] отработано тактов: %lld%s\n", cycle,
                 Verilated::gotFinish() ? " (RTL завершил симуляцию)" : "");
    return 0;
}
