<#
    Сборка и запуск безголового стенда DarkRISCV под Verilator.

    Зачем скрипт, а не строчка в README: на этой машине связка собирается только
    при четырёх обходных мерах, и каждая из них выяснена отдельным разбором.
    Держать их в голове нельзя, поэтому они закреплены здесь.

    1. Обёртка verilator (скрипт на Perl) падает: в поставке oss-cad-suite нет
       модуля Pod::Usage. Вызывается напрямую verilator_bin.exe.
    2. Двоичный файл ищет свой корень по зашитому пути /yosyshq/share/verilator.
       Нужен VERILATOR_ROOT.
    3. Makefile Verilator собирает с -Os. На MSYS2 UCRT64 с GCC 16.2 при -Os
       включается -fdeclone-ctor-dtor, и объектники начинают ссылаться на
       конструкторы с меткой C4/D4, которых в поставленном libstdc++ нет.
       Ломается ЛЮБАЯ программа на C++, не только Verilator: проверено на
       трёхстрочном примере со std::string. Лечится -fno-declone-ctor-dtor.
    4. Makefile Verilator зовёт python3, которого в MSYS нет. Рядом кладётся
       копия python.exe под именем python3.exe.

    Плюс пятое: native-инструменты в некоторых оболочках наследуют TEMP=C:\Windows
    и не могут создать временный файл. Здесь TEMP задан явно.

    Строки латиницей намеренно: PowerShell 5.1 читает .ps1 как ANSI, и кириллица
    без метки порядка байтов ломает разбор файла. Пояснения — в этом блоке,
    он читается как комментарий целиком.
#>
param(
    [string]$OssCadSuite = 'C:\workspace\verilog\1k\oss-cad-suite',
    [string]$MsysBin     = 'C:\msys64\ucrt64\bin',
    [string]$MsysUsrBin  = 'C:\msys64\usr\bin',
    [string]$PythonExe   = 'C:\Users\User\AppData\Local\Programs\Python\Python312\python.exe',
    [switch]$Trace,
    [long]$Cycles = 20000000
)

$ErrorActionPreference = 'Stop'
$simDir  = Split-Path -Parent (Split-Path -Parent $PSCommandPath)   # ...\dark_risc\sim
$root    = Split-Path -Parent $simDir                                # ...\dark_risc
$objDir  = Join-Path $simDir 'verilator\obj_dir'
$shimDir = Join-Path $env:TEMP 'vlshim'

# --- python3 shim ---
if (-not (Test-Path $shimDir)) { New-Item -ItemType Directory -Path $shimDir | Out-Null }
$shim = Join-Path $shimDir 'python3.exe'
if (-not (Test-Path $shim)) { Copy-Item $PythonExe $shim }

$env:VERILATOR_ROOT = ($OssCadSuite + '\share\verilator') -replace '\\', '/'
$env:PATH = "$shimDir;$MsysBin;$MsysUsrBin"
$env:TMP  = [System.IO.Path]::GetTempPath().TrimEnd('\')
$env:TEMP = $env:TMP

$vl = Join-Path $OssCadSuite 'bin\verilator_bin.exe'
if (-not (Test-Path $vl)) { throw "verilator_bin.exe not found: $vl" }

$rtl = @(
    'darksocv.v','darkriscv.v','darkbridge.v','darkram.v','darkio.v',
    'darkuart.v','darkspi.v','darkpll.v','darkcache.v','darkmac.v'
) | ForEach-Object { Join-Path $root ('rtl\' + $_) }

$args = @(
    '--cc','--exe','--build','-j','8',
    '-DSIMULATION','-DM20K_DEV_BOARD',
    '-Wno-WIDTHTRUNC','-Wno-WIDTHEXPAND','-Wno-PINMISSING','-Wno-CASEX',
    '-Wno-CASEINCOMPLETE','-Wno-MULTIDRIVEN','-Wno-UNOPTFLAT','-Wno-REALCVT',
    '-Wno-BLKANDNBLK','-Wno-COMBDLY','-Wno-INITIALDLY',
    '--x-assign','fast','--x-initial','fast',
    '-CFLAGS','-fno-declone-ctor-dtor',
    '-I' + (Join-Path $root 'rtl'),
    '-I' + (Join-Path $root 'rtl\lib'),
    '--Mdir', $objDir,
    '--top-module','darksocv'
)
if ($Trace) { $args += '--trace' }
$args += $rtl
$args += (Join-Path $simDir 'verilator\darksocv_tb.cpp')
$args += @('-o','Vdarksocv')

Remove-Item -Recurse -Force $objDir -ErrorAction SilentlyContinue
Write-Host '--- verilator ---'
& $vl @args
if ($LASTEXITCODE -ne 0) { throw "verilator exit $LASTEXITCODE" }

$exe = Join-Path $objDir 'Vdarksocv.exe'
if (-not (Test-Path $exe)) { throw "binary not built: $exe" }
Write-Host ("built: {0} KB" -f [int]((Get-Item $exe).Length / 1KB))

# The firmware image path is hardcoded in darkram.v as ../src/darksocv.mem,
# so the binary must be started from the sim directory.
Push-Location $simDir
try {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    & $exe "+cycles=$Cycles"
    $sw.Stop()
    Write-Host ('--- {0} cycles in {1:N2} s = {2:N2} Mcycle/s ---' -f `
        $Cycles, $sw.Elapsed.TotalSeconds, ($Cycles / $sw.Elapsed.TotalSeconds / 1e6))
}
finally { Pop-Location }
