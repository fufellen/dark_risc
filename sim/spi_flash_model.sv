`timescale 1ns / 1ps

/*
    Модель конфигурационной микросхемы FLASH на шине SPI soft-MCU.

    Зачем она нужна. До сих пор в симуляции обращения к флешу подменялись
    заглушкой в самой прошивке (`mock_flash`), то есть шина SPI не работала
    вовсе. Проверить на ней было нечего: ни порядок фронтов, ни выдержки, ни
    скорость. Всё это выяснялось только на железе, циклами сборки и заливки.

    Модель повторяет XT25F32B (32 Мбит), тот самый, что стоит на плате:
    идентификатор 0B 40 16 и набор команд, которым пользуется прошивка.

    Режим 0: адрес и данные от ведущего защёлкиваются по нарастающему фронту,
    ответ выдвигается по спадающему. Именно этот порядок и проверяется —
    ускоренный bit-bang легко сделать так, что на железе он читает сдвинутые
    на бит данные, а по осциллограмме выглядит здоровым.

    Содержимое незаписанных ячеек детерминировано и зависит от адреса, поэтому
    тест сверяет прочитанное без предварительной заливки образа.
*/
module spi_flash_model #(parameter
    // Задержка выдачи бита после спадающего фронта, аналог tCLQV даташита.
    // Ускоренный bit-bang обязан её переживать, иначе он читает мусор.
    time TCLQV = 8ns
)(
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output wire miso
);

    localparam byte unsigned CMD_READ      = 8'h03;
    localparam byte unsigned CMD_FAST_READ = 8'h0B;
    localparam byte unsigned CMD_RDSR      = 8'h05;
    localparam byte unsigned CMD_WREN      = 8'h06;
    localparam byte unsigned CMD_PP        = 8'h02;
    localparam byte unsigned CMD_SE        = 8'h20;
    localparam byte unsigned CMD_JEDEC     = 8'h9F;

    // Записанные ячейки. Незаписанные отдаются по правилу ниже.
    byte unsigned cells [int];

    function automatic byte unsigned cell_at(input int addr);
        if (cells.exists(addr)) return cells[addr];
        return byte'(addr) ^ byte'(addr >> 8) ^ byte'(addr >> 16) ^ 8'h5A;
    endfunction

    byte unsigned sr_status = 8'h00;
    byte unsigned cmd    = 8'h00;
    int           addr   = 0;
    int           phase  = 0;   // 0 команда, 1..3 адрес, 4 холостой, 5 данные
    byte unsigned shift_in = 0;
    int           bit_cnt  = 0;

    logic         miso_r = 1'bz;
    byte unsigned out_byte = 8'hFF;
    int           out_bit  = 7;

    assign miso = cs_n ? 1'bz : miso_r;

    // Счётчики для отчёта: сколько байт отдано за прогон.
    int unsigned bytes_read = 0;

    always @(negedge cs_n) begin
        phase    = 0;
        bit_cnt  = 0;
        shift_in = 0;
        miso_r   = 1'bz;
    end

    always @(posedge cs_n) begin
        miso_r = 1'bz;
    end

    // Приём: ведущий выставляет бит до фронта, модель защёлкивает по фронту.
    always @(posedge sck) begin
        if (!cs_n) begin
            shift_in = byte'((shift_in << 1) | mosi);
            bit_cnt++;
            if (bit_cnt == 8) begin
                bit_cnt = 0;
                case (phase)
                    0: begin
                        cmd = shift_in;
                        case (cmd)
                            CMD_JEDEC: begin phase = 5; addr = 0; out_byte = 8'h0B; end
                            CMD_RDSR:  begin phase = 5; addr = 0; out_byte = sr_status; end
                            CMD_WREN:  begin sr_status = sr_status | 8'h02; phase = 9; end
                            CMD_READ, CMD_FAST_READ, CMD_PP, CMD_SE: begin
                                phase = 1; addr = 0;
                            end
                            default: phase = 9;
                        endcase
                    end
                    1, 2: begin
                        addr  = (addr << 8) | int'(shift_in);
                        phase = phase + 1;
                    end
                    3: begin
                        addr = (addr << 8) | int'(shift_in);
                        case (cmd)
                            CMD_READ: begin
                                phase = 5;
                                out_byte = cell_at(addr);
                            end
                            CMD_FAST_READ: phase = 4;   // впереди холостой байт
                            CMD_PP: phase = 6;
                            CMD_SE: begin
                                for (int i = 0; i < 4096; i++)
                                    cells[(addr & ~32'hFFF) + i] = 8'hFF;
                                sr_status = sr_status & ~8'h02;
                                phase = 9;
                            end
                            default: phase = 9;
                        endcase
                    end
                    4: begin
                        phase = 5;
                        out_byte = cell_at(addr);
                    end
                    5: begin
                        // Ведущий дотактировал байт — готовим следующий.
                        addr++;
                        bytes_read++;
                        case (cmd)
                            CMD_JEDEC: out_byte = (addr == 1) ? 8'h40 :
                                                  (addr == 2) ? 8'h16 : 8'hFF;
                            CMD_RDSR:  out_byte = sr_status;
                            default:   out_byte = cell_at(addr);
                        endcase
                    end
                    6: begin
                        // Запись страницы: адрес заворачивается внутри 256 байт.
                        cells[addr] = shift_in;
                        addr = (addr & ~32'hFF) | ((addr + 1) & 32'hFF);
                        sr_status = sr_status & ~8'h02;
                    end
                    default: ;
                endcase
            end
        end
    end

    // Выдача: бит появляется на линии после спада, с выдержкой.
    always @(negedge sck) begin
        if (!cs_n && (phase == 5)) begin
            if (bit_cnt == 0) out_bit = 7;
            miso_r <= #TCLQV out_byte[out_bit];
            if (out_bit > 0) out_bit--;
        end
    end

endmodule
