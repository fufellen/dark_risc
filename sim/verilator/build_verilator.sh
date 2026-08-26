#!/usr/bin/env bash
#
# Сборка и запуск безголового стенда DarkRISCV под Verilator.
#
# Повторяет ровно ту последовательность, которая отработала 26.08.2026, и
# закрепляет шесть обходных мер — каждая из них в одиночку валит сборку.
#
# 1. Обёртка `verilator` — скрипт на Perl, а в поставке oss-cad-suite нет
#    модуля Pod::Usage. Зовём verilator_bin.exe напрямую.
# 2. Двоичный файл ищет свой корень по зашитому пути /yosyshq/share/verilator.
#    Нужен VERILATOR_ROOT.
# 3. Исходники подключают "../rtl/config.vh" — путь относительно ТЕКУЩЕГО
#    каталога, а не подключающего файла. Поэтому и трансляция, и потом сам
#    двоичный файл запускаются из sim/. По той же причине там же лежит и
#    "../src/darksocv.mem", который грузит darkram.v.
# 4. Ключ --build не используется: Verilator тогда зовёт make сам, и запущенный
#    так из PowerShell с перенаправленным выводом он зависает наглухо. Сборка
#    отдельным шагом.
# 5. Makefile Verilator зовёт python3, которого в MSYS нет. Рядом кладётся
#    копия python.exe под этим именем.
# 6. При -Os GCC 16.2 из MSYS2 UCRT64 включает -fdeclone-ctor-dtor и ссылается
#    на конструкторы с меткой C4/D4, которых нет в поставленном libstdc++.
#    Не собирается ЛЮБАЯ программа на C++, не только Verilator — проверено на
#    трёхстрочном примере со std::string. Лечится -fno-declone-ctor-dtor.
#
# Плюс седьмое, не флаг, а свойство среды: native-инструменты в MSYS-оболочке
# наследуют TEMP=C:\Windows и не могут создать временный файл. Поэтому шаг
# сборки идёт через cmd.exe, где TEMP задаётся нативно.
#
#     ./build_verilator.sh                 # собрать и прогнать 20 млн тактов
#     ./build_verilator.sh --trace         # то же, но с записью волн
#     ./build_verilator.sh --cycles 100000 # короткий прогон

set -euo pipefail

OSS=${OSS:-/c/workspace/verilog/1k/oss-cad-suite}
MSYS_BIN=${MSYS_BIN:-/c/msys64/ucrt64/bin}
MSYS_USR=${MSYS_USR:-/c/msys64/usr/bin}
PYTHON=${PYTHON:-/c/Users/User/AppData/Local/Programs/Python/Python312/python.exe}

CYCLES=20000000
TRACE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --trace)  TRACE=1; shift ;;
        --cycles) CYCLES="$2"; shift 2 ;;
        *) echo "неизвестный ключ: $1" >&2; exit 2 ;;
    esac
done

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)   # .../sim/verilator
SIM=$(dirname "$HERE")                                # .../sim
ROOT=$(dirname "$SIM")                                # .../dark_risc
OBJ="$HERE/obj_dir"

# --- подмена python3 ---
SHIM="${TMPDIR:-/c/Users/User/AppData/Local/Temp}/vlshim"
mkdir -p "$SHIM"
[ -x "$SHIM/python3.exe" ] || cp "$PYTHON" "$SHIM/python3.exe"

export VERILATOR_ROOT="$(cygpath -m "$OSS/share/verilator")"
export PATH="$SHIM:$MSYS_BIN:$MSYS_USR:$PATH"

VL="$OSS/bin/verilator_bin.exe"
[ -x "$VL" ] || { echo "нет verilator_bin.exe: $VL" >&2; exit 1; }

RTL=""
for f in darksocv darkriscv darkbridge darkram darkio darkuart darkspi darkpll darkcache darkmac; do
    RTL="$RTL ../rtl/$f.v"
done

rm -rf "$OBJ"

echo "--- трансляция ---"
cd "$SIM"
# shellcheck disable=SC2086
"$VL" --cc --exe \
    -DSIMULATION -DM20K_DEV_BOARD \
    -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-PINMISSING -Wno-CASEX \
    -Wno-CASEINCOMPLETE -Wno-MULTIDRIVEN -Wno-UNOPTFLAT -Wno-REALCVT \
    -Wno-BLKANDNBLK -Wno-COMBDLY -Wno-INITIALDLY \
    --x-assign fast --x-initial fast \
    -CFLAGS "-fno-declone-ctor-dtor" \
    $( [ "$TRACE" = 1 ] && echo --trace ) \
    -I../rtl -I../rtl/lib \
    --Mdir verilator/obj_dir --top-module darksocv \
    $RTL "$(cygpath -m "$HERE/darksocv_tb.cpp")" \
    -o Vdarksocv

echo "--- сборка ---"
# Через cmd.exe: в MSYS-оболочке native-компоновщик наследует TEMP=C:\Windows
# и падает на создании временного файла.
OBJ_WIN=$(cygpath -w "$OBJ")
cmd.exe //c "set TEMP=%LOCALAPPDATA%\\Temp&& set TMP=%LOCALAPPDATA%\\Temp&& cd /d $OBJ_WIN && make -f Vdarksocv.mk -j 8" > /dev/null

[ -x "$OBJ/Vdarksocv.exe" ] || { echo "двоичный файл не собран" >&2; exit 1; }
echo "собрано: $(( $(stat -c%s "$OBJ/Vdarksocv.exe") / 1024 )) КБ"

echo "--- прогон ---"
cd "$SIM"
START=$(date +%s%N)
"$OBJ/Vdarksocv.exe" "+cycles=$CYCLES" $( [ "$TRACE" = 1 ] && echo "+vcd=darksocv.vcd" )
END=$(date +%s%N)
MS=$(( (END - START) / 1000000 ))
# Целочисленное деление здесь врало бы на единицу вниз, а число идёт в отчёты.
RATE=$(awk -v c="$CYCLES" -v m="$MS" 'BEGIN{ printf "%.2f", (m>0 ? c/(m/1000.0)/1e6 : 0) }')
echo "--- $CYCLES тактов за ${MS} мс = ${RATE} млн тактов/с ---"
