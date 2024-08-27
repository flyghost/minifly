@echo off
setlocal enabledelayedexpansion

:: 指定你的文件夹路径
set "source_folder=./FLIGHT"

:: 指定clang-format的样式
@REM set "format_style=file"  :: 或者选择其他的样式，如 "LLVM", "Google", "Chromium", "Mozilla", "WebKit"
set "format_style=file"  :: 使用当前目录下的 .clang-format 文件

:: 递归查找文件夹中的所有 .c 和 .cpp 文件
for /r "%source_folder%" %%f in (*.c *.h) do (
  echo Formatting file: %%f
  clang-format -i -style=!format_style! "%%f"
)

echo All files formatted.
pause