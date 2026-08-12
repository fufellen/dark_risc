`timescale 1ns / 1ps

/*
    Сколько тактов запаса нужно ускоренному bit-bang до микросхемы FLASH.

    Вопрос конкретный. В прошивке между двумя записями в порт стояла выдержка
    из трёх обращений к volatile-переменной — она стоила больше самой шины.
    Убрать её очевидно выгодно, но неочевидно безопасно: бит с микросхемы идёт
    путём «шина SoC → пад → микросхема → пад → входной регистр», и если читать
    линию раньше, чем он дойдёт, данные приезжают сдвинутыми. На осциллограмме
    такая шина выглядит совершенно здоровой, поэтому ловить это на железе —
    самый дорогой способ.

    Тестбенч воспроизводит ровно то, что делает процессор: две записи в порт на
    такт шины и одно чтение входного порта. Расстояния между ними заданы
    параметрами и перебираются, а сверка идёт с моделью микросхемы. На выходе
    не «работает / не работает», а граница: при каком запасе чтение верное.

    Запуск: do spibb_flash_tb.do
*/

module spibb_flash_tb;
    timeunit 1ns; timeprecision 1ps;

    localparam int OPORT_MOSI = 0;
    localparam int OPORT_SCK  = 1;
    localparam int OPORT_CSN  = 2;
    localparam int OPORT_EN   = 3;
    localparam int IPORT_MISO = 6;

    logic clk = 0;
    always #10 clk = ~clk;      // 50 МГц, как ядро soft-MCU

    logic [31:0] oport = 32'h0000000C;   // EN | CSN
    wire  [31:0] iport;

    wire csn, sck, miso;
    wire mosi;

    spi_master_bb dut (
        .CLK(clk),
        .RES(1'b0),
        .IPORT(iport),
        .OPORT(oport),
        .CSN(csn),
        .SCK(sck),
        .MOSI(mosi),
        .MISO(miso)
    );

    spi_flash_model flash (
        .cs_n(csn),
        .sck(sck),
        .mosi(mosi),
        .miso(miso)
    );

    /*
        Модель процессора. gap — тактов между записями в порт, lat — тактов от
        второй записи до чтения входного порта. Именно эти два расстояния и
        задаёт скомпилированный код: запись, запись, загрузка.
    */
    int gap = 2;
    int lat = 2;

    task automatic bb_write(input logic [31:0] value);
        @(posedge clk);
        oport <= value;
        repeat (gap) @(posedge clk);
    endtask

    task automatic bb_byte(input logic [7:0] tx, output logic [7:0] rx);
        logic [31:0] base = (1 << OPORT_EN);
        rx = 8'h00;
        for (int b = 7; b >= 0; b--) begin
            logic [31:0] m = tx[b] ? (1 << OPORT_MOSI) : 32'h0;
            bb_write(base | m);                        // спад такта
            bb_write(base | m | (1 << OPORT_SCK));     // фронт
            repeat (lat) @(posedge clk);
            rx = {rx[6:0], iport[IPORT_MISO]};
        end
    endtask

    task automatic bb_select();
        bb_write((1 << OPORT_EN) | (1 << OPORT_CSN) | (1 << OPORT_SCK) | (1 << OPORT_MOSI));
        bb_write((1 << OPORT_EN) | (1 << OPORT_SCK) | (1 << OPORT_MOSI));
    endtask

    task automatic bb_deselect();
        bb_write((1 << OPORT_EN) | (1 << OPORT_CSN) | (1 << OPORT_SCK) | (1 << OPORT_MOSI));
    endtask

    function automatic logic [7:0] expected(input int addr);
        return 8'(addr) ^ 8'(addr >> 8) ^ 8'(addr >> 16) ^ 8'h5A;
    endfunction

    // Один прогон при заданных gap/lat: идентификатор плюс блок данных.
    task automatic try_case(input int g, input int l, output int errors,
                            output time per_byte);
        logic [7:0] id0, id1, id2, d;
        time t0;
        gap = g;
        lat = l;
        errors = 0;

        bb_select();
        bb_byte(8'h9F, d);
        bb_byte(8'h00, id0);
        bb_byte(8'h00, id1);
        bb_byte(8'h00, id2);
        bb_deselect();
        if (id0 !== 8'h0B || id1 !== 8'h40 || id2 !== 8'h16) errors++;

        bb_select();
        bb_byte(8'h03, d);
        bb_byte(8'h00, d);
        bb_byte(8'h12, d);
        bb_byte(8'h34, d);
        t0 = $time;
        for (int i = 0; i < 8; i++) begin
            bb_byte(8'h00, d);
            if (d !== expected(32'h001234 + i)) errors++;
        end
        per_byte = ($time - t0) / 8;
        bb_deselect();
    endtask

    int    errs;
    time   per_byte;
    int    best_gap = 0;
    time   best_time = 0;

    initial begin
        repeat (5) @(posedge clk);

        $display("[TB] перебор запаса: gap — такты между записями в порт, lat — до чтения");
        for (int g = 1; g <= 4; g++) begin
            for (int l = 1; l <= 4; l++) begin
                try_case(g, l, errs, per_byte);
                $display("  gap=%0d lat=%0d: %0s, %0t на байт",
                         g, l, (errs == 0) ? "ok " : "BAD", per_byte);
                if (errs == 0 && best_gap == 0) begin
                    best_gap = g;
                    best_time = per_byte;
                end
            end
        end

        // Прежний вариант с выдержкой: заведомо большой запас, эталон скорости.
        try_case(12, 12, errs, per_byte);
        $display("  прежняя выдержка (gap=lat=12): %0s, %0t на байт",
                 (errs == 0) ? "ok " : "BAD", per_byte);

        if (best_gap == 0) begin
            $display("[FAIL] верного чтения не нашлось ни при каком запасе");
        end else begin
            $display("[PASS] минимальный запас gap=%0d, %0t на байт против %0t прежних",
                     best_gap, best_time, per_byte);
        end

        $finish;
    end

endmodule
