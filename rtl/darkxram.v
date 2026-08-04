/*
 * Дополнительная память данных для DarkRISCV на свободном слоте XADDR[31:30].
 *
 * Зачем: образ прошивки живёт в единственной BRAM размером 2**MLEN, и когда
 * .text подрастает, стек упирается в .bss — счёт идёт на сотни байт, а
 * поведение при переполнении неотличимо от зависания. Свободные блоки BSRAM
 * в кристалле при этом остаются. Этот модуль отдаёт их процессору как
 * отдельную область ДАННЫХ: линкер уводит туда стек и .bss, а все 2**MLEN
 * остаются под код и инициализированные данные.
 *
 * Только порт данных: код по-прежнему читается из darkram по IADDR.
 * Инициализируется нулями, поэтому .bss можно размещать здесь без обнуления
 * в boot — как и в основной памяти.
 *
 * Интерфейс и тайминги повторяют порт данных darkram, включая XDACK и
 * побайтную запись по XBE.
 */
`timescale 1ns / 1ps

module darkxram
#(
    parameter XLEN = 14     // размер области: 2**XLEN байт (14 = 16 КиБ)
)
(
    input             CLK,
    input             RES,
    input             XDREQ,
    input             XRD,
    input             XWR,
    input      [3:0]  XBE,
    input      [31:0] XADDR,
    input      [31:0] XATAI,
    output     [31:0] XATAO,
    output            XDACK
);

    reg [31:0] MEM [0:2**XLEN/4-1];

`ifdef SIMULATION
    // В железе BSRAM стартует нулями сама; GowinSynthesis на таком цикле
    // упирается в лимит развёртки (EX3934), поэтому он только для симуляции.
    integer i;

    initial
    begin
        for(i = 0; i != 2**XLEN/4; i = i + 1)
        begin
            MEM[i] = 32'd0;
        end
    end
`endif

    reg [3:0]  DTACK = 0;
    reg [31:0] RAMFF = 0;

    always@(posedge CLK)
    begin
        DTACK <= RES ? 0 : DTACK ? DTACK-1 : XDREQ && XRD ? 1 : 0;

        RAMFF <= MEM[XADDR[XLEN-1:2]];

        if(XWR && XDREQ && XBE[3]) MEM[XADDR[XLEN-1:2]][3 * 8 + 7: 3 * 8] <= XATAI[3 * 8 + 7: 3 * 8];
        if(XWR && XDREQ && XBE[2]) MEM[XADDR[XLEN-1:2]][2 * 8 + 7: 2 * 8] <= XATAI[2 * 8 + 7: 2 * 8];
        if(XWR && XDREQ && XBE[1]) MEM[XADDR[XLEN-1:2]][1 * 8 + 7: 1 * 8] <= XATAI[1 * 8 + 7: 1 * 8];
        if(XWR && XDREQ && XBE[0]) MEM[XADDR[XLEN-1:2]][0 * 8 + 7: 0 * 8] <= XATAI[0 * 8 + 7: 0 * 8];
    end

    assign XATAO = RAMFF;
    assign XDACK = DTACK==1 || (XDREQ && XWR);

endmodule
