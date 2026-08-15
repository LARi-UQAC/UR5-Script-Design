@echo off
REM Build the RTDE fallback monitor.
REM
REM Run this on a normal networked machine that has MinGW-w64 gcc
REM (x86_64-w64-mingw32); only the resulting rtde_fallback_monitor.exe is
REM copied to the lab computer. -static covers the MinGW C runtime, so the
REM executable carries no DLL of its own; ws2_32.dll is a core Windows
REM system library present on every install.
setlocal
cd /d "%~dp0"

gcc -O2 -static -Wall -Wextra -o rtde_fallback_monitor.exe rtde_fallback_monitor.c -lws2_32
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)

echo Built rtde_fallback_monitor.exe
exit /b 0
