@echo off
setlocal enabledelayedexpansion

set UV=C:\Keil_v5\UV4\UV4.exe
set UV_PRO_PATH=./USER/Firmware_F411.uvprojx

echo Init building ...

:: 方法1
:: echo .>build_log.txt
%UV% -j0 -r %UV_PRO_PATH% 
:: -o %cd%\build_log.txt

:: 方法2
:: 执行编译命令，并将输出直接显示在终端上
:: %UV% -j0 -r %UV_PRO_PATH%
:: 在新命令提示符窗口中运行UV4.exe，并显示实时输出
:: start cmd /k %UV% -j0 -r %UV_PRO_PATH%

type build_log.txt
echo Done.
pause