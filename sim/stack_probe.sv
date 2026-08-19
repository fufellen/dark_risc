`timescale 1ns / 1ps

/*
    Наблюдатель за глубиной стека soft-MCU.

    Нужен, чтобы отвечать замером, а не арифметикой по карте линковки. Карта
    говорит, где кончаются переменные; сколько на деле забирает стек — видно
    только по указателю в работающей программе.

    Следит за регистром sp (x2) и запоминает минимум. Печатает его в конце
    прогона вместе с границей области переменных, которую надо передать
    параметром.
*/
module stack_probe #(parameter
    int unsigned BSS_END = 32'h0000FF48,   // конец переменных по карте линковки
    int unsigned STACK_TOP = 32'h00010000
);
    int unsigned sp_min = 32'hFFFFFFFF;

    always @(posedge darksimv.CLK) begin
        automatic int unsigned sp = darksimv.soc0.bridge0.core0.REGS[2];
        if (sp != 0 && sp < sp_min) begin
            sp_min = sp;
        end
    end

    final begin
        $display("[stack] вершина %0h, переменные до %0h, свободно %0d Б",
                 STACK_TOP, BSS_END, STACK_TOP - BSS_END);
        $display("[stack] минимум указателя %0h, глубина %0d Б",
                 sp_min, STACK_TOP - sp_min);
        if (sp_min < BSS_END) begin
            $display("[stack] СТЕК ЗАШЁЛ В ПЕРЕМЕННЫЕ на %0d Б", BSS_END - sp_min);
        end else begin
            $display("[stack] в переменные не заходил, запас %0d Б", sp_min - BSS_END);
        end
    end
endmodule
